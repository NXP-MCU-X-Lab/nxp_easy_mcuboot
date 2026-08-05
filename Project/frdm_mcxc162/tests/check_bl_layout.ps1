$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$projectFile = Join-Path $projectRoot "mcxc162_bl/.cproject"
$assertFile = Join-Path $projectRoot "mcxc162_bl/source/bl_size_assert.ld"
$projectText = Get-Content $projectFile -Raw
$linkerOption = '-T${ProjDirPath}/source/bl_size_assert.ld'

if ($projectText -notmatch 'id="PROGRAM_FLASH" location="0x0" size="0x4000"')
{
    throw "PROGRAM_FLASH is not limited to 16 KB"
}

if (([regex]::Matches($projectText, [regex]::Escape($linkerOption))).Count -ne 2)
{
    throw "Debug and Release must both include the bootloader size assertion"
}

if (-not (Test-Path $assertFile))
{
    throw "Bootloader size assertion file is missing"
}

$assertText = Get-Content $assertFile -Raw
if ($assertText -notmatch 'ASSERT\s*\(\s*_image_end\s*<=\s*0x4000')
{
    throw "Bootloader size assertion does not enforce the 16 KB boundary"
}

Write-Output "PASS bootloader layout contract"
