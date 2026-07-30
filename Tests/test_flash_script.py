import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
FLASH_SCRIPT = ROOT / "scripts" / "flash.ps1"
PROJECT = ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
POWERSHELL = shutil.which("powershell") or shutil.which("pwsh")


def ps_literal(value):
    return "'" + str(value).replace("'", "''") + "'"


def run_harness(body):
    if not POWERSHELL:
        raise unittest.SkipTest("PowerShell is unavailable")
    with tempfile.TemporaryDirectory() as directory:
        harness = Path(directory) / "harness.ps1"
        harness.write_text(
            "$ErrorActionPreference = 'Stop'\n"
            "$env:M6_FLASH_LIBRARY_ONLY = '1'\n"
            f". {ps_literal(FLASH_SCRIPT)}\n"
            + body,
            encoding="utf-8",
        )
        completed = subprocess.run(
            [
                POWERSHELL,
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(harness),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=20,
        )
    if completed.returncode != 0:
        raise AssertionError(
            f"PowerShell harness failed ({completed.returncode}):\n"
            f"stdout={completed.stdout}\nstderr={completed.stderr}"
        )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    return json.loads(lines[-1])


class FlashScriptTests(unittest.TestCase):
    def test_source_has_required_preflight_and_no_transport_fallback(self):
        source = FLASH_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("Assert-HorcoCmsisDap", source)
        self.assertIn("Assert-KeilDebugConfiguration", source)
        self.assertIn(".uvoptx", source)
        self.assertIn("4041840e", source)
        self.assertIn("6220000::V6\\.22::ARMCLANG", source)
        self.assertIn("UL2CM3\\.DLL", source)
        self.assertIn("0 Error\\(s\\), 0 Warning\\(s\\)", source)
        self.assertIn("-WindowStyle Hidden", source)
        self.assertIn("Verify OK", source)
        self.assertIn("m6_tune.py", source)
        self.assertIn("for ($attempt = 1; $attempt -le 2; $attempt++)", source)
        self.assertNotIn("-WindowStyle Normal", source)
        self.assertNotIn("start-process -filepath $uv4", source.lower())

    def test_real_production_project_passes_configuration_contract(self):
        result = run_harness(
            f"$target = Assert-KeilProjectConfiguration -ProjectPath {ps_literal(PROJECT)}\n"
            "[pscustomobject]@{Name=[string]$target.TargetName; Device=[string]$target.TargetOption.TargetCommonOption.Device} | ConvertTo-Json -Compress\n"
        )

        self.assertEqual(result["Name"], "empty_LP_MSPM0G3507_nortos_keil")
        self.assertEqual(result["Device"], "MSPM0G3507")

    def test_probe_preflight_requires_matching_horco_name_and_serial(self):
        result = run_harness(
            "$good = { @(\n"
            "  [pscustomobject]@{FriendlyName='USB Composite Device'; InstanceId='USB\\VID_FAED&PID_4873\\4041840e'; Status='OK'},\n"
            "  [pscustomobject]@{FriendlyName='Horco CMSIS-DAP v2'; InstanceId='USB\\VID_FAED&PID_4873&MI_00\\child'; ParentInstanceId='USB\\VID_FAED&PID_4873\\4041840e'; Status='OK'}\n"
            ") }\n"
            "$bad = { @(\n"
            "  [pscustomobject]@{FriendlyName='USB Composite Device'; InstanceId='USB\\VID_FAED&PID_4873\\different'; Status='OK'},\n"
            "  [pscustomobject]@{FriendlyName='Horco CMSIS-DAP v2'; InstanceId='USB\\VID_FAED&PID_4873&MI_00\\child'; ParentInstanceId='USB\\VID_FAED&PID_4873\\different'; Status='OK'}\n"
            ") }\n"
            "$matched = Assert-HorcoCmsisDap -SerialNumber '4041840e' -DeviceProvider $good\n"
            "$rejected = $false\n"
            "try { Assert-HorcoCmsisDap -SerialNumber '4041840e' -DeviceProvider $bad | Out-Null } catch { $rejected = $true }\n"
            "[pscustomobject]@{Name=$matched.FriendlyName; Rejected=$rejected} | ConvertTo-Json -Compress\n"
        )

        self.assertEqual(result["Name"], "Horco CMSIS-DAP v2")
        self.assertTrue(result["Rejected"])

    def test_debug_preflight_requires_use_target_and_matching_registry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            good_project = root / "good.uvprojx"
            bad_project = root / "bad.uvprojx"
            good_project.write_text("<Project/>", encoding="utf-8")
            bad_project.write_text("<Project/>", encoding="utf-8")

            template = """<?xml version="1.0" encoding="UTF-8"?>
<ProjectOpt><Target>
  <TargetName>empty_LP_MSPM0G3507_nortos_keil</TargetName>
  <TargetOption>
    <OPTTT><RunTarget>1</RunTarget></OPTTT>
    <OPTFL><IsCurrentTarget>1</IsCurrentTarget></OPTFL>
    <DebugOpt><uSim>{use_simulator}</uSim><uTrg>{use_target}</uTrg><pMon>BIN\\CMSIS_AGDI.dll</pMon></DebugOpt>
    <TargetDriverDllRegistry><SetRegEntry><Key>CMSIS_AGDI</Key>
      <Name>-X&quot;Horco CMSIS-DAP v2&quot; -U4041840e -S0 -FF0MSPM0G1X0X_G3X0X_MAIN_128KB.FLM</Name>
    </SetRegEntry></TargetDriverDllRegistry>
    <DebugDescription><Enable>1</Enable><EnableFlashSeq>1</EnableFlashSeq><Protocol>2</Protocol><DbgClock>1000000</DbgClock></DebugDescription>
  </TargetOption>
</Target></ProjectOpt>
"""
            good_project.with_suffix(".uvoptx").write_text(
                template.format(use_simulator=0, use_target=1), encoding="utf-8"
            )
            bad_project.with_suffix(".uvoptx").write_text(
                template.format(use_simulator=1, use_target=0), encoding="utf-8"
            )
            result = run_harness(
                "$target = 'empty_LP_MSPM0G3507_nortos_keil'\n"
                f"$good = Assert-KeilDebugConfiguration -ProjectPath {ps_literal(good_project)} -TargetName $target -SerialNumber '4041840e'\n"
                "$rejected = $false\n"
                f"try {{ Assert-KeilDebugConfiguration -ProjectPath {ps_literal(bad_project)} -TargetName $target -SerialNumber '4041840e' | Out-Null }} catch {{ $rejected = $true }}\n"
                "[pscustomobject]@{Target=[string]$good.TargetName; Rejected=$rejected} | ConvertTo-Json -Compress\n"
            )

        self.assertEqual(result["Target"], "empty_LP_MSPM0G3507_nortos_keil")
        self.assertTrue(result["Rejected"])

    def test_first_failure_retries_identical_command_once(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "flash.log"
            body = (
                "$script:calls = @()\n"
                "$script:attempt = 0\n"
                f"$log = {ps_literal(log_path)}\n"
                "$runner = { param($Executable, $Arguments)\n"
                "  $script:attempt++\n"
                "  $script:calls += ([string]::Join('|', [string[]]$Arguments))\n"
                "  if ($script:attempt -eq 1) { Set-Content -LiteralPath $log -Value 'Programming failed'; [pscustomobject]@{ExitCode=1} }\n"
                "  else { Set-Content -LiteralPath $log -Value 'Erase Done. Programming Done. Verify OK.'; [pscustomobject]@{ExitCode=0} }\n"
                "}\n"
                "$arguments = @('-f','\"project.uvprojx\"','-o','\"flash.log\"')\n"
                "$result = Invoke-KeilDownloadWithRetry -UV4Path 'UV4.exe' -ArgumentList $arguments -LogPath $log -ProcessRunner $runner\n"
                "[pscustomobject]@{Attempts=$result.Attempts; Calls=$script:calls.Count; Same=($script:calls[0] -eq $script:calls[1])} | ConvertTo-Json -Compress\n"
            )
            result = run_harness(body)

        self.assertEqual(result["Attempts"], 2)
        self.assertEqual(result["Calls"], 2)
        self.assertTrue(result["Same"])

    def test_two_failures_stop_without_third_attempt(self):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "flash.log"
            body = (
                "$script:calls = 0\n"
                f"$log = {ps_literal(log_path)}\n"
                "$runner = { param($Executable, $Arguments) $script:calls++; Set-Content -LiteralPath $log -Value 'failed'; [pscustomobject]@{ExitCode=1} }\n"
                "$message = ''\n"
                "try { Invoke-KeilDownloadWithRetry -UV4Path 'UV4.exe' -ArgumentList @('-f','project') -LogPath $log -ProcessRunner $runner | Out-Null } catch { $message = $_.Exception.Message }\n"
                "[pscustomobject]@{Calls=$script:calls; Message=$message} | ConvertTo-Json -Compress\n"
            )
            result = run_harness(body)

        self.assertEqual(result["Calls"], 2)
        self.assertIn("failed twice", result["Message"])
        self.assertIn("download manually", result["Message"])

    def test_post_flash_probe_requires_identity_schema_revision_and_source(self):
        result = run_harness(
            f"$root = {ps_literal(ROOT)}\n"
            "$runner = { param($Arguments) [pscustomobject]@{ExitCode=0; Output=@('{\"firmware\":\"M6-test\",\"schema\":1,\"revision\":9,\"source\":\"persisted\"}')} }\n"
            "$identity = Invoke-M6ApplicationProbe -RepositoryRoot $root -FirmwarePrefix 'M6' -Schema 1 -PythonRunner $runner\n"
            "$badRunner = { param($Arguments) [pscustomobject]@{ExitCode=0; Output=@('{\"firmware\":\"M6-test\",\"schema\":2,\"revision\":9,\"source\":\"flash-a\"}')} }\n"
            "$rejected = $false\n"
            "try { Invoke-M6ApplicationProbe -RepositoryRoot $root -FirmwarePrefix 'M6' -Schema 1 -PythonRunner $badRunner | Out-Null } catch { $rejected = $true }\n"
            "[pscustomobject]@{Revision=$identity.revision; Source=$identity.source; Rejected=$rejected} | ConvertTo-Json -Compress\n"
        )

        self.assertEqual(result["Revision"], 9)
        self.assertEqual(result["Source"], "persisted")
        self.assertTrue(result["Rejected"])


if __name__ == "__main__":
    unittest.main()
