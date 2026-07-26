import copy
import io
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
    "summary": "A compact fenced loading yard.",
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

    def test_duplicate_regions_are_normalized(self):
        plan = copy.deepcopy(VALID_PLAN)
        plan["props"][1]["region"] = "northeast"

        worldgen.normalize_plan(plan)

        self.assertEqual(plan["props"][0]["region"], "northeast")
        self.assertEqual(plan["props"][1]["region"], "northwest")
        self.assertEqual(worldgen.validate_plan(plan), [])

    def test_review_retry_then_publish(self):
        responses = iter([
            copy.deepcopy(VALID_PLAN),
            {"decision": "semantic_mismatch"},
            copy.deepcopy(VALID_PLAN),
            {"decision": "approve"},
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
            {"decision": "approve"},
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
            return {"decision": "approve"}

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
