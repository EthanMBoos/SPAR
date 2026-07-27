import copy
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import urllib.error

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import generate_world as worldgen  # noqa: E402


VALID_PLAN = {
    "ground": "concrete",
    "props": [
        {
            "kind": "rack",
            "color": "gray",
            "region": "northeast",
            "orientation": "east_west",
        },
        {
            "kind": "container",
            "color": "blue",
            "region": "northwest",
            "orientation": "north_south",
        },
        {
            "kind": "anomaly_drum",
            "color": "red",
            "region": "southeast",
            "orientation": "east_west",
        },
        {
            "kind": "fence",
            "color": "white",
            "region": "southwest",
            "orientation": "east_west",
        },
    ],
}


class WorldgenTest(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory()
        root = Path(self.scratch.name)
        self.root = root
        self.worlds = root / "sim" / "worlds"
        self.worlds.mkdir(parents=True)
        os.symlink(REPO / "sim" / "robots", root / "sim" / "robots")
        os.symlink(REPO / "third_party", root / "third_party")

    def tearDown(self):
        self.scratch.cleanup()

    def test_valid_plan_renders_and_lints(self):
        self.assertEqual(worldgen.validate_plan(VALID_PLAN), [])
        world_path = self.worlds / "yard.xml"
        world_path.write_text(worldgen.render_world(VALID_PLAN, "yard"))

        failures, _, counts = worldgen.validate_world(
            "husky", str(world_path), worldgen.GROUND_AUTONOMY)

        self.assertEqual(failures, [])
        self.assertEqual(counts["static_solids"], 4)
        text = world_path.read_text()
        self.assertIn('size="8 8 1"', text)
        self.assertIn('file="../robots/husky.xml"', text)
        self.assertIn('file="../robots/x2.xml"', text)

    def test_validator_rejects_bad_anomaly_and_duplicate_region(self):
        plan = copy.deepcopy(VALID_PLAN)
        plan["props"][2]["color"] = "blue"
        plan["props"][0]["region"] = "northwest"

        errors = worldgen.validate_plan(plan)

        self.assertIn("anomaly_drum must be red", errors)
        self.assertIn("duplicate prop region: northwest", errors)

    def test_validator_reserves_red_for_anomaly(self):
        plan = copy.deepcopy(VALID_PLAN)
        plan["props"][0]["color"] = "red"

        self.assertIn(
            "red is reserved for anomaly_drum",
            worldgen.validate_plan(plan),
        )

    def test_review_retry_then_publish(self):
        responses = iter([
            copy.deepcopy(VALID_PLAN),
            {
                "decision": "semantic_mismatch",
                "reason": "The props do not resemble a loading yard.",
            },
            copy.deepcopy(VALID_PLAN),
            {"decision": "approve", "reason": "The layout matches the request."},
        ])

        def chat(*_args):
            return next(responses)

        world, _ = worldgen.generate(
            "a loading yard", "yard", "test-model", "localhost",
            chat=chat, worlds_dir=str(self.worlds))

        self.assertTrue(Path(world).exists())
        self.assertEqual(list(self.root.rglob("*.yaml")), [])

    def test_malformed_review_retries_then_publishes(self):
        responses = iter([
            copy.deepcopy(VALID_PLAN),
            worldgen.InvalidModelResponse("truncated JSON"),
            copy.deepcopy(VALID_PLAN),
            {"decision": "approve", "reason": "The layout matches the request."},
        ])

        def chat(*_args):
            response = next(responses)
            if isinstance(response, Exception):
                raise response
            return response

        world, _ = worldgen.generate(
            "a loading yard", "yard", "test-model", "localhost",
            chat=chat, worlds_dir=str(self.worlds))

        self.assertTrue(Path(world).exists())

    def test_exhaustion_publishes_nothing(self):
        def chat(*args):
            if args[-1] == worldgen.PLAN_SCHEMA:
                return copy.deepcopy(VALID_PLAN)
            return {"decision": "approve", "reason": "The layout matches the request."}

        with mock.patch.object(
                worldgen, "validate_world",
                return_value=(["interpenetration"], [], {})):
            with self.assertRaisesRegex(RuntimeError, "no valid world"):
                worldgen.generate(
                    "a loading yard", "yard", "test-model", "localhost",
                    chat=chat, worlds_dir=str(self.worlds))

        self.assertFalse((self.worlds / "yard.xml").exists())
        self.assertEqual(list(self.worlds.glob(".worldgen_*")), [])

    def test_existing_output_requires_force(self):
        (self.worlds / "yard.xml").write_text("user data")

        with self.assertRaisesRegex(FileExistsError, "--force"):
            worldgen.generate(
                "a loading yard", "yard", "test-model", "localhost",
                chat=lambda *_: self.fail("Ollama should not be called"),
                worlds_dir=str(self.worlds))

        self.assertEqual((self.worlds / "yard.xml").read_text(), "user data")

    def test_seeded_positions_and_metadata_are_reproducible(self):
        first = worldgen.render_world(
            VALID_PLAN, "yard", seed=7,
            description="a loading yard", model_tag="test-model")
        second = worldgen.render_world(
            VALID_PLAN, "yard", seed=7,
            description="a loading yard", model_tag="test-model")
        different = worldgen.render_world(
            VALID_PLAN, "yard", seed=8,
            description="a loading yard", model_tag="test-model")

        self.assertEqual(first, second)
        self.assertNotEqual(first, different)
        self.assertIn('name="worldgen.plan"', first)
        self.assertIn('name="worldgen.description" data="a loading yard"', first)
        self.assertIn('name="worldgen.model" data="test-model"', first)
        self.assertIn('name="worldgen.seed" data="7"', first)

    def test_seed_appears_in_planner_prompt(self):
        messages = worldgen.planner_messages("a loading yard", seed=17)

        self.assertIn("Variation ID: 17", messages[-1]["content"])

    def test_success_record_contains_experiment_facts(self):
        record = self.root / "worldgen.jsonl"
        responses = iter([
            copy.deepcopy(VALID_PLAN),
            {"decision": "approve", "reason": "The layout matches the request."},
        ])

        worldgen.generate(
            "a loading yard", "yard", "test-model", "localhost",
            chat=lambda *_: next(responses), worlds_dir=str(self.worlds),
            seed=4, record_path=str(record))

        entry = json.loads(record.read_text())
        self.assertEqual(entry["status"], "accepted")
        self.assertEqual(entry["model"], "test-model")
        self.assertEqual(entry["seed"], 4)
        self.assertEqual(entry["successful_attempt"], 1)
        self.assertEqual(entry["lint"]["static_solids"], 4)
        world_bytes = (self.worlds / "yard.xml").read_bytes()
        self.assertEqual(
            entry["world_sha256"],
            hashlib.sha256(world_bytes).hexdigest(),
        )
        self.assertEqual(entry["plan"], VALID_PLAN)

    def test_ollama_metrics_are_extracted(self):
        response_body = {
            "message": {"content": json.dumps(VALID_PLAN)},
            "load_duration": 12_500_000,
            "prompt_eval_count": 101,
            "prompt_eval_duration": 250_000_000,
            "eval_count": 42,
            "eval_duration": 1_500_000_000,
            "total_duration": 1_800_000_000,
        }
        response = mock.MagicMock()
        response.__enter__.return_value = io.BytesIO(
            json.dumps(response_body).encode())

        with mock.patch(
                "urllib.request.urlopen", return_value=response) as urlopen:
            result = worldgen.ollama_chat(
                "localhost", "test-model", [], worldgen.PLAN_SCHEMA)

        self.assertEqual(result.value, VALID_PLAN)
        self.assertEqual(result.metrics, {
            "load_ms": 12.5,
            "prompt_eval_ms": 250.0,
            "generated_eval_ms": 1500.0,
            "total_ms": 1800.0,
            "prompt_tokens": 101,
            "generated_tokens": 42,
        })
        request = urlopen.call_args.args[0]
        request_body = json.loads(request.data)
        self.assertEqual(request_body["options"]["temperature"], 0)

    def test_attempt_records_include_ollama_metrics(self):
        record = self.root / "worldgen.jsonl"
        metrics = {
            "load_ms": 1.0,
            "prompt_tokens": 10,
            "prompt_eval_ms": 2.0,
            "generated_tokens": 5,
            "generated_eval_ms": 3.0,
            "total_ms": 6.0,
        }
        responses = iter([
            worldgen.OllamaResult(copy.deepcopy(VALID_PLAN), metrics),
            worldgen.OllamaResult({
                "decision": "approve",
                "reason": "The layout matches the request.",
            }, metrics),
        ])

        worldgen.generate(
            "a loading yard", "yard", "test-model", "localhost",
            chat=lambda *_: next(responses), worlds_dir=str(self.worlds),
            record_path=str(record))

        attempts = json.loads(record.read_text())["attempts"]
        self.assertEqual(attempts[0]["stage"], "planner")
        self.assertEqual(attempts[0]["ollama"], metrics)
        self.assertEqual(attempts[1]["stage"], "reviewer")
        self.assertEqual(attempts[1]["ollama"], metrics)

    def test_ollama_connection_error_is_clear(self):
        with mock.patch(
                "urllib.request.urlopen",
                side_effect=urllib.error.URLError("connection refused")):
            with self.assertRaisesRegex(
                    RuntimeError, "cannot reach Ollama at http://localhost"):
                worldgen.ollama_chat(
                    "localhost", "missing", [], worldgen.PLAN_SCHEMA)

    def test_ollama_model_error_includes_server_message(self):
        error = urllib.error.HTTPError(
            "http://localhost:11434/api/chat", 404, "not found", {},
            io.BytesIO(b'{"error":"model missing not found"}'))
        with mock.patch("urllib.request.urlopen", side_effect=error):
            with self.assertRaisesRegex(RuntimeError, "model missing not found"):
                worldgen.ollama_chat(
                    "localhost", "missing", [], worldgen.PLAN_SCHEMA)


if __name__ == "__main__":
    unittest.main()
