[CmdletBinding()]
param(
    [int]$Epochs = 40,
    [int]$Batch = 16,
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

function Invoke-PythonStep {
    param([Parameter(Mandatory = $true)][string[]]$Arguments)

    & $Python @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Python command failed with exit code $LASTEXITCODE"
    }
}

Push-Location $repoRoot
try {
    Invoke-PythonStep @(
        "vision/scripts/prepare_hard_case_finetune.py"
    )
    Invoke-PythonStep @(
        "vision/scripts/check_yolo_labels.py",
        "--images", "vision/hard_cases/finetune/images/train",
        "--labels", "vision/hard_cases/finetune/labels/train"
    )
    Invoke-PythonStep @(
        "vision/scripts/check_yolo_labels.py",
        "--images", "vision/hard_cases/finetune/images/val",
        "--labels", "vision/hard_cases/finetune/labels/val"
    )
    Invoke-PythonStep @(
        "vision/scripts/train_yolo.py",
        "--data", "vision/hard_cases/finetune/data.yaml",
        "--model", "vision/models/steel_ball_yolo11n.pt",
        "--imgsz", "416",
        "--epochs", $Epochs.ToString(),
        "--batch", $Batch.ToString(),
        "--project", "vision/runs",
        "--name", "steel_ball_yolo11n_hardcase_ft",
        "--finetune"
    )
}
finally {
    Pop-Location
}
