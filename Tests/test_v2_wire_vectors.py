from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import check_m6_v2_vectors as wire  # noqa: E402


class M6V2WireVectorTests(unittest.TestCase):
    def test_registry_schema_vectors_and_generated_include(self):
        counts = wire.validate_repository_files()
        self.assertEqual(counts["crc_vectors"], 3)
        self.assertEqual(counts["escape_vectors"], 1)
        self.assertEqual(counts["session_vectors"], 2)
        self.assertEqual(counts["legacy_ascii_vectors"], 1)
        self.assertEqual(counts["resync_vectors"], 1)
        self.assertEqual(counts["frame_vectors"], 10)
        self.assertEqual(counts["stream_vectors"], 22)

    def test_checker_cli_is_read_only(self):
        include = ROOT / "Tests" / "generated" / "m6_v2_wire_vectors.inc"
        before = include.read_bytes()
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "check_m6_v2_vectors.py"), "--check"],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(include.read_bytes(), before)

    def test_write_inc_cannot_bless_changed_frozen_revision(self):
        registry = wire.strict_load(wire.DEFAULT_REGISTRY)
        registry["telemetry"]["streams"][3]["period_max_ms"] = 5100
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            changed_registry = directory_path / "changed.json"
            output_include = directory_path / "changed.inc"
            changed_registry.write_text(json.dumps(registry), encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "check_m6_v2_vectors.py"),
                    "--registry", str(changed_registry),
                    "--schema", str(wire.DEFAULT_SCHEMA),
                    "--inc", str(output_include),
                    "--write-inc",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(output_include.exists())

    def test_schema_digest_rejects_a_valid_but_changed_schema(self):
        schema = wire.strict_load(wire.DEFAULT_SCHEMA)
        schema["title"] += " changed"
        with tempfile.TemporaryDirectory() as directory:
            changed_schema = Path(directory) / "changed-schema.json"
            changed_schema.write_text(json.dumps(schema), encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "check_m6_v2_vectors.py"),
                    "--registry", str(wire.DEFAULT_REGISTRY),
                    "--schema", str(changed_schema),
                    "--inc", str(wire.DEFAULT_INC),
                    "--check",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("schema canonical SHA256 changed", result.stderr)

    def test_strict_loader_rejects_duplicate_and_nonfinite_json(self):
        cases = ('{"a": 1, "a": 2}', '{"value": NaN}', '{"value": Infinity}', '{"value": 1e999}')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for payload in cases:
                path.write_text(payload, encoding="utf-8")
                with self.subTest(payload=payload):
                    with self.assertRaises(wire.RegistryError):
                        wire.strict_load(path)

    def test_parser_boundary_vector_inventory(self):
        registry = wire.strict_load(wire.DEFAULT_REGISTRY)
        names = {item["id"] for item in registry["vectors"]["streams"]}
        required = {
            "fragmented_hello",
            "concatenated_frames",
            "leading_noise",
            "crc_flip",
            "short_frame",
            "illegal_escape",
            "interbyte_49_accepts",
            "interbyte_50_rejects",
            "total_99_accepts",
            "total_100_rejects",
            "decoded_111_valid",
            "wire_224_exact",
            "wire_225_rejected",
            "wire_overflow_transition_refreshes_discard",
            "decoded_112_rejected",
            "discard_resync",
            "discard_interbyte_timeout",
            "discard_total_timeout",
            "emergency_epoch_clears_candidate",
            "overflow_epoch_clears_candidate",
            "u32_wrap_49_accepts",
            "u32_wrap_50_rejects",
        }
        self.assertTrue(required.issubset(names))


if __name__ == "__main__":
    unittest.main()
