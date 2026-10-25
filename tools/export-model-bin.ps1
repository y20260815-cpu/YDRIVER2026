param(
    [Parameter(Mandatory = $true)]
    [string]$ElfPath,
    [string]$Objcopy = 'arm-none-eabi-objcopy'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$headerPath = Join-Path $projectRoot 'Core\Inc\system_setup.h'
$header = Get-Content -LiteralPath $headerPath -Raw
$definitions = [regex]::Matches(
    $header,
    '(?m)^\s*#define\s+SYSTEM_MODEL_NAME\s+"([^"]+)"\s*(?://[^\r\n]*)?\r?$'
)
if ($definitions.Count -ne 1) {
    throw 'Define exactly one SYSTEM_MODEL_NAME string in Core\Inc\system_setup.h.'
}
$model = $definitions[0].Groups[1].Value
if ($model -notmatch '^[A-Za-z0-9][A-Za-z0-9_-]*$') {
    throw 'SYSTEM_MODEL_NAME must contain only ASCII letters, digits, underscores or hyphens.'
}

$elf = Get-Item -LiteralPath $ElfPath
if ($elf.PSIsContainer -or $elf.Extension -ne '.elf') {
    throw "Expected an ELF file: $ElfPath"
}
$converter = Get-Command -Name $Objcopy -CommandType Application -ErrorAction Stop
$buildDate = $elf.LastWriteTime.ToString('yyyyMMdd')
$outputPath = Join-Path $elf.DirectoryName ($model + '_' + $buildDate + '.bin')
$temporaryPath = $outputPath + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
try {
    & $converter.Source '-O' 'binary' $elf.FullName $temporaryPath
    if ($LASTEXITCODE -ne 0) {
        throw "objcopy failed with exit code $LASTEXITCODE."
    }
    if (-not (Test-Path -LiteralPath $temporaryPath -PathType Leaf)) {
        throw 'objcopy did not create the binary file.'
    }
    if ((Get-Item -LiteralPath $temporaryPath).Length -eq 0) {
        throw 'objcopy created an empty binary file.'
    }
    Move-Item -LiteralPath $temporaryPath -Destination $outputPath -Force
    Write-Host "Model binary: $outputPath"
}
finally {
    if (Test-Path -LiteralPath $temporaryPath) {
        Remove-Item -LiteralPath $temporaryPath -Force
    }
}
