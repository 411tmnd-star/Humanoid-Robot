# run_qa.ps1 (humanoid_qa_v01.0) - full read-only QA pipeline for Windows.
#
#   powershell -ExecutionPolicy Bypass -File scripts\humanoid_qa\run_qa.ps1 `
#       [-Blender "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"] `
#       [-Source blender\robot_armature_v15.blend] [-Python python] [-NoRender] `
#       [-Experiments] [-BaselineRun humanoid_inspect_v01_v15_20261012T124107Z]
#
# Steps: 1 pure unit tests (system Python)  2 Blender fixture tests (factory
# startup, no project file)  3 run_qa.py on the source  4 regression compare
# against the previous QA run (written to blender\inspection\comparisons\)
# 5 optional isolated pose/export experiments on a disposable COPY (-Experiments).
# A failed test does not stop step 3: it downgrades the affected methods to
# FAILED_VALIDATION there, so the failure stays visible in the evidence.
# Exit code: 1 if any step failed, 2 if the comparison found regressions
# (and nothing else failed), 0 otherwise. Detected design anomalies never
# change the exit code.

param(
    [string]$Blender = "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe",
    [string]$Source = "",
    [string]$Python = "python",
    [string]$BaselineRun = "humanoid_inspect_v01_v15_20261012T124107Z",
    [switch]$NoRender,
    [switch]$Experiments
)

$ErrorActionPreference = "Stop"
$QaDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Root = Resolve-Path (Join-Path $QaDir "..\..")
if (-not $Source) { throw "-Source <file.blend> is required: scene files are not stored in git" }
if (-not (Test-Path $Blender)) { throw "Blender not found at $Blender (pass -Blender)" }
if (-not (Test-Path $Source)) { throw "Source not found at $Source" }

$Stamp = (Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")
$Tag = [IO.Path]::GetFileNameWithoutExtension($Source) -replace "^robot_armature_", ""
$RunId = "humanoid_qa_v01_${Tag}_$Stamp"
$InspDir = Join-Path $Root "blender\inspection"
$Ledger = Join-Path $InspDir "evidence_ledger.jsonl"
$RunDir = Join-Path $InspDir $RunId
$Stage = Join-Path ([IO.Path]::GetTempPath()) "hqa_$RunId"
New-Item -ItemType Directory -Force $Stage | Out-Null
$Unit = Join-Path $Stage "unit_results.json"
$Fix = Join-Path $Stage "fixture_results.json"
$failed = @()
$regressed = $false

# previous QA run = newest existing humanoid_qa_* directory with a manifest
$Prev = Get-ChildItem $InspDir -Directory -Filter "humanoid_qa_*" -ErrorAction SilentlyContinue |
    Where-Object { Test-Path (Join-Path $_.FullName "run_manifest.json") } |
    Sort-Object Name | Select-Object -Last 1

Write-Host "== 1/5 unit tests"
& $Python (Join-Path $QaDir "tests\test_core.py") $Unit
if ($LASTEXITCODE -ne 0) { $failed += "unit_tests" }

Write-Host "== 2/5 Blender fixture tests"
& $Blender --background --factory-startup --python-exit-code 1 `
    --python (Join-Path $QaDir "tests\blender_fixture_tests.py") -- $Fix
if ($LASTEXITCODE -ne 0) { $failed += "fixture_tests" }

Write-Host "== 3/5 QA run $RunId"
$qaArgs = @("--background", "--factory-startup", "--disable-autoexec", $Source,
    "--python-exit-code", "1", "--python", (Join-Path $QaDir "run_qa.py"), "--",
    "--out-dir", $RunDir, "--run-id", $RunId, "--ledger", $Ledger,
    "--unit-results", $Unit, "--fixture-results", $Fix, "--baseline-run", $BaselineRun)
if ($NoRender) { $qaArgs += "--no-render" }
& $Blender @qaArgs
if ($LASTEXITCODE -ne 0) { $failed += "qa_run" }

Write-Host "== 4/5 regression compare"
if ($Prev -and (Test-Path (Join-Path $RunDir "run_manifest.json"))) {
    $CmpDir = Join-Path $InspDir "comparisons"
    New-Item -ItemType Directory -Force $CmpDir | Out-Null
    $CmpFile = Join-Path $CmpDir "$($Prev.Name)__vs__$RunId.json"
    & $Python (Join-Path $QaDir "compare_runs.py") $Prev.FullName $RunDir $CmpFile
    if ($LASTEXITCODE -eq 2) { $regressed = $true }
    elseif ($LASTEXITCODE -ne 0) { $failed += "compare" }
    Write-Host "COMPARISON=$CmpFile"
} else { Write-Host "no previous QA run to compare against" }

if ($Experiments) {
    Write-Host "== 5/5 isolated experiments"
    $ExpId = "humanoid_exp_v01_${Tag}_$Stamp"
    $ExpDir = Join-Path $InspDir $ExpId
    & $Blender --background --factory-startup --disable-autoexec --python-exit-code 1 `
        --python (Join-Path $QaDir "run_experiments.py") -- --source $Source `
        --out-dir $ExpDir --run-id $ExpId --ledger $Ledger `
        --unit-results $Unit --fixture-results $Fix
    if ($LASTEXITCODE -ne 0) { $failed += "experiments" }
    Write-Host "EXPERIMENT_DIR=$ExpDir"
}

Write-Host "RUN_DIR=$RunDir"
if ($failed.Count) { Write-Host "FAILED STEPS: $($failed -join ', ')"; exit 1 }
if ($regressed) { Write-Host "REGRESSIONS FOUND (see comparison file)"; exit 2 }
Write-Host "ALL STEPS OK"
exit 0
