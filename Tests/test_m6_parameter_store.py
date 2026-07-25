from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class M6ParameterStoreTests(unittest.TestCase):
    def test_record_codec_and_selection(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "m6_parameter_store_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DPARAMETER_STORE_HOST_TEST",
                "-I",
                str(ROOT / "System"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "m6_parameter_store_test.c"),
                str(ROOT / "System" / "control_params.c"),
                str(ROOT / "System" / "parameter_store.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_flash_path_is_ram_resident_and_commit_last(self):
        source = (ROOT / "System" / "parameter_store.c").read_text(
            encoding="utf-8"
        )
        self.assertIn('section(".ramfunc")', source)
        self.assertIn("WWDT0->WWDTCNTRST", source)
        self.assertIn("write_sector_from_ram", source)
        self.assertIn("final 64-bit commit word is last", source)
        self.assertIn("PARAMETER_STORE_FLASH_POLL_LIMIT", source)
        loop = source.index("for (word_index")
        reprotect = source.index("unprotect_parameter_group_from_ram();", loop)
        program = source.index("FLASHCTL->GEN.CMDTYPE =", reprotect)
        self.assertLess(reprotect, program)
        self.assertIn("DL_FlashCTL_unprotectSector", source)
        self.assertIn("DL_FlashCTL_protectSector", source)

    def test_boot_report_exposes_degraded_storage_without_a_motion_fault(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("PARAMETER_STORE_HEALTH_OK", diagnostics)
        self.assertIn('"DEGRADED"', diagnostics)
        store = (ROOT / "System" / "parameter_store.c").read_text(
            encoding="utf-8"
        )
        init_body = store[store.index("void ParameterStore_Init(void)") :]
        self.assertNotIn("Safety_RaiseFault", init_body)


if __name__ == "__main__":
    unittest.main()
