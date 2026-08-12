"""Host-only tests for world-generation recipes and prompt context."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from worldgen import generation, stage
from worldgen.georeference import family_georeference


class GenerationTest(unittest.TestCase):
    def test_world_names_are_lowercased_at_the_boundary(self) -> None:
        self.assertEqual(
            stage.normalize_world_name("Utility_Depot_Trial_01"),
            "utility_depot_trial_01",
        )
        with self.assertRaisesRegex(ValueError, "letters, digits, and underscores"):
            stage.normalize_world_name("utility-depot")

    def test_stage_seeds_are_stable_and_independent(self) -> None:
        first = generation.derive_seed(42, "utility_depot", "01_plan_topology")
        self.assertEqual(
            first,
            generation.derive_seed(42, "utility_depot", "01_plan_topology"),
        )
        self.assertNotEqual(
            first,
            generation.derive_seed(42, "utility_depot", "02_plan_infrastructure"),
        )
        self.assertNotEqual(
            first,
            generation.derive_seed(43, "utility_depot", "01_plan_topology"),
        )

    def test_seed_and_brief_validation(self) -> None:
        self.assertEqual(generation.parse_seed("42"), 42)
        self.assertIsNone(generation.parse_seed(""))
        with self.assertRaisesRegex(ValueError, "unsigned 64-bit"):
            generation.parse_seed("-1")
        with self.assertRaisesRegex(ValueError, "must not exceed"):
            generation.validate_brief("x" * (generation.MAX_BRIEF_CHARS + 1))

    def test_spawn_defaults_are_stable_and_vary_by_seed(self) -> None:
        first = generation.family_spawn_defaults("utility_depot", 42)
        self.assertEqual(
            first, generation.family_spawn_defaults("utility_depot", 42)
        )
        self.assertNotEqual(
            first, generation.family_spawn_defaults("utility_depot", 43)
        )
        self.assertEqual(first["dock"], first["husky_spawn"])
        self.assertTrue(generation._valid_spawn_defaults(first))
        self.assertFalse(generation._valid_spawn_defaults({
            "husky_spawn": {"position": [0.0, 0.0, float("nan")], "yaw": 0.0}
        }))

    def test_georeference_is_fixed_and_explicit(self) -> None:
        reference = family_georeference("utility_depot")
        self.assertEqual(reference["map_frame"], "ENU")
        self.assertEqual(reference["projection"], "wgs84_local_tangent")
        self.assertEqual(reference, family_georeference("utility_depot"))

    def test_recipe_round_trip_and_mismatch_rejection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            root = repo / "worldgen"
            shared = root / "prompts" / "shared_contract.md"
            prompt = root / "families" / "test_family" / "prompts" / "01_plan.md"
            contract = root / "families" / "test_family" / "variation_contract.md"
            for path, contents in (
                (shared, "shared"),
                (prompt, "stage"),
                (contract, "contract"),
            ):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(contents, encoding="utf-8")
            stages = {"01_plan": prompt}
            patches = (
                mock.patch.object(generation, "REPO", repo),
                mock.patch.object(generation, "ROOT", root),
                mock.patch.object(
                    generation, "_git_provenance", return_value={"commit": "abc", "dirty": False}
                ),
                mock.patch.object(generation, "_claude_version", return_value="test"),
            )
            with patches[0], patches[1], patches[2], patches[3]:
                manifest = generation.resolve_manifest(
                    "trial",
                    stages,
                    family="test_family",
                    seed=42,
                    brief="dense west side",
                    model="sonnet",
                    effort="medium",
                    shared_prompt=shared,
                )
                saved = json.loads(
                    generation.manifest_path("trial").read_text(encoding="utf-8")
                )
                self.assertEqual(saved, manifest)
                loaded = generation.resolve_manifest(
                    "trial", stages, existing=True, shared_prompt=shared
                )
                self.assertEqual(loaded, manifest)
                with self.assertRaisesRegex(ValueError, "seed conflicts"):
                    generation.resolve_manifest(
                        "trial", stages, seed=43, existing=True, shared_prompt=shared
                    )
                prompt.write_text("changed", encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "prompt source changed"):
                    generation.resolve_manifest(
                        "trial", stages, existing=True, shared_prompt=shared
                    )

                dry_manifest = generation.resolve_manifest(
                    "dry_trial",
                    stages,
                    family="test_family",
                    seed=7,
                    persist=False,
                    shared_prompt=shared,
                )
                self.assertEqual(dry_manifest["seed"], 7)
                self.assertFalse(generation.manifest_path("dry_trial").exists())

    def test_rendered_prompt_contains_recipe_context(self) -> None:
        stages = stage.available_stages("utility_depot")
        manifest = {
            "world": "test_world",
            "family": "utility_depot",
            "seed": 42,
            "brief": "denser west-side storage",
            "model": "sonnet",
            "effort": "medium",
            "spawn_defaults": generation.family_spawn_defaults(
                "utility_depot", 42
            ),
            "georeference": family_georeference("utility_depot"),
            "stages": {
                name: {"seed": generation.derive_seed(42, "utility_depot", name)}
                for name in stages
            },
        }
        rendered = stage.render_prompt(manifest, "01_plan_topology")
        self.assertIn("World seed: `42`", rendered)
        self.assertIn("denser west-side storage", rendered)
        self.assertIn("Utility-depot variation contract", rendered)
        self.assertIn("Repo-sampled spawn defaults", rendered)
        self.assertIn('"husky_spawn"', rendered)
        self.assertIn("placement override", rendered)
        self.assertNotIn("<WORLD>", rendered)
        later = stage.render_prompt(manifest, "02_plan_infrastructure")
        self.assertNotIn("Repo-sampled spawn defaults", later)


if __name__ == "__main__":
    unittest.main()
