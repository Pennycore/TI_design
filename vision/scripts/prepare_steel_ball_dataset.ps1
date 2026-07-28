param(
    [string]$InputRoot = "C:\Users\28457\Desktop\dataset",
    [string]$OutRoot = "C:\Users\28457\Desktop\TI_design\vision\dataset",
    [double]$ValRatio = 0.2
)

$ErrorActionPreference = "Stop"

$source1Root = Get-ChildItem -LiteralPath $InputRoot -Directory |
    Where-Object { (Test-Path -LiteralPath (Join-Path $_.FullName "dataset\classes.txt")) -and (Test-Path -LiteralPath (Join-Path $_.FullName "dataset\images")) } |
    Select-Object -First 1

$source2Root = Get-ChildItem -LiteralPath $InputRoot -Directory |
    Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName "dataset\data.yaml") } |
    Select-Object -First 1

$source3Root = Join-Path $InputRoot "ganqiu\ganqiu"

$sources = New-Object System.Collections.Generic.List[object]

if ($source1Root) {
    $sources.Add(@{
        Name = "source1"
        Images = Join-Path $source1Root.FullName "dataset\images"
        Labels = Join-Path $source1Root.FullName "dataset\labels"
    })
}

if ($source2Root) {
    $sources.Add(@{
        Name = "source2"
        Images = Join-Path $source2Root.FullName "dataset\images"
        Labels = Join-Path $source2Root.FullName "dataset\labels"
    })
}

if (Test-Path -LiteralPath $source3Root) {
    $sources.Add(@{
        Name = "source3"
        Images = Join-Path $source3Root "image"
        Labels = Join-Path $source3Root "labels"
    })
}

if ($sources.Count -eq 0) {
    throw "No supported source datasets found under $InputRoot"
}

$imageExts = @(".jpg", ".jpeg", ".png", ".bmp")
$items = New-Object System.Collections.Generic.List[object]
$missingLabels = New-Object System.Collections.Generic.List[string]

foreach ($source in $sources) {
    $imageRoot = $source.Images
    $labelRoot = $source.Labels

    if (!(Test-Path -LiteralPath $imageRoot)) {
        throw "Image path not found: $imageRoot"
    }
    if (!(Test-Path -LiteralPath $labelRoot)) {
        throw "Label path not found: $labelRoot"
    }

    $images = Get-ChildItem -LiteralPath $imageRoot -Recurse -File |
        Where-Object { $imageExts -contains $_.Extension.ToLowerInvariant() } |
        Sort-Object FullName

    foreach ($image in $images) {
        $rootWithSlash = $imageRoot.TrimEnd("\") + "\"
        $relative = $image.FullName.Substring($rootWithSlash.Length)
        $relativeDir = [System.IO.Path]::GetDirectoryName($relative)
        $relativeBase = [System.IO.Path]::GetFileNameWithoutExtension($relative)
        $relativeNoExt = if ([string]::IsNullOrEmpty($relativeDir)) { $relativeBase } else { Join-Path $relativeDir $relativeBase }
        $labelPath = Join-Path $labelRoot ($relativeNoExt + ".txt")

        if (!(Test-Path -LiteralPath $labelPath)) {
            $missingLabels.Add($image.FullName)
            continue
        }

        $items.Add([pscustomobject]@{
            SourceName = $source.Name
            ImagePath = $image.FullName
            LabelPath = $labelPath
            Extension = $image.Extension.ToLowerInvariant()
        })
    }
}

if ($items.Count -eq 0) {
    throw "No image/label pairs found."
}

$dirs = @(
    "images\train",
    "images\val",
    "labels\train",
    "labels\val"
)

foreach ($dir in $dirs) {
    $path = Join-Path $OutRoot $dir
    New-Item -ItemType Directory -Force -Path $path | Out-Null
    Get-ChildItem -LiteralPath $path -File | Remove-Item -Force
}

$trainCount = 0
$valCount = 0
$index = 0
$valEvery = [Math]::Max(2, [int][Math]::Round(1.0 / $ValRatio))

foreach ($item in $items) {
    $isVal = (($index % $valEvery) -eq 0)
    $split = if ($isVal) { "val" } else { "train" }
    $name = "{0}_{1:D6}{2}" -f $item.SourceName, $index, $item.Extension
    $labelName = [System.IO.Path]::ChangeExtension($name, ".txt")

    Copy-Item -LiteralPath $item.ImagePath -Destination (Join-Path $OutRoot ("images\$split\$name"))
    Copy-Item -LiteralPath $item.LabelPath -Destination (Join-Path $OutRoot ("labels\$split\$labelName"))

    if ($isVal) {
        $valCount++
    } else {
        $trainCount++
    }
    $index++
}

$dataYaml = @"
path: $OutRoot
train: images/train
val: images/val
names:
  0: steel_ball
"@

Set-Content -LiteralPath (Join-Path $OutRoot "data.yaml") -Value $dataYaml -Encoding UTF8

[pscustomobject]@{
    TotalPairs = $items.Count
    Train = $trainCount
    Val = $valCount
    MissingLabels = $missingLabels.Count
    OutRoot = $OutRoot
}

if ($missingLabels.Count -gt 0) {
    $missingPath = Join-Path $OutRoot "missing_labels.txt"
    Set-Content -LiteralPath $missingPath -Value $missingLabels -Encoding UTF8
    Write-Warning "Some images were skipped because labels were missing. See $missingPath"
}
