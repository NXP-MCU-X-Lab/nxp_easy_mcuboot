$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$projectFile = Join-Path $projectRoot "mcxc162_bl/.cproject"
$appProjectFile = Join-Path $projectRoot "mcxc162_example_app/.cproject"
$trialHeaderFile = Join-Path $projectRoot "common/bl_trial.h"
$appLinkerFile = Join-Path $projectRoot "mcxc162_example_app/linker/mcxc162_example_app.ld"
$assertFile = Join-Path $projectRoot "mcxc162_bl/source/bl_size_assert.ld"
$projectText = Get-Content $projectFile -Raw
$appProjectText = Get-Content $appProjectFile -Raw
$appLinkerText = Get-Content $appLinkerFile -Raw
$linkerOption = '-T${ProjDirPath}/source/bl_size_assert.ld'
$commonInclude = '${ProjDirPath}/../common'

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

if ($appProjectText -notmatch 'platform\.drivers\.wwdt\.MCXC162')
{
    throw "App project does not include the MCXC162 WWDT SDK component"
}

if (-not (Test-Path $trialHeaderFile))
{
    throw "Shared trial header is missing"
}

$trialHeaderText = Get-Content $trialHeaderFile -Raw
if ($trialHeaderText -notmatch 'BL_TRIAL_STATE_ADDR\s+\(0x20001000u\)')
{
    throw "Shared trial state is not fixed at 0x20001000"
}

if (([regex]::Matches($projectText, [regex]::Escape($commonInclude))).Count -ne 2)
{
    throw "Bootloader Debug and Release must include the shared trial header path"
}

if (([regex]::Matches($appProjectText, [regex]::Escape($commonInclude))).Count -ne 2)
{
    throw "App Debug and Release must include the shared trial header path"
}

if ($assertText -notmatch '\(_pvHeapLimit\s*\+\s*_StackSize\)\s*<=\s*0x20001000')
{
    throw "Bootloader low RAM reservations can overlap the shared trial state"
}

if ($assertText -notmatch '_vStackBase\s*>=\s*0x20001010')
{
    throw "Bootloader stack can overlap the shared trial state"
}

if ($appLinkerText -notmatch '\(_pvHeapLimit\s*\+\s*_StackSize\)\s*<=\s*0x20001000')
{
    throw "App low RAM reservations can overlap the shared trial state"
}

if ($appLinkerText -notmatch '_vStackBase\s*>=\s*0x20001010')
{
    throw "App stack can overlap the shared trial state"
}

if ($assertText -notmatch 'g_bl_trial_state\s*=\s*ABSOLUTE\(0x20001000\)')
{
    throw "Bootloader map cannot expose the shared trial state address"
}

if ($appLinkerText -notmatch 'g_bl_trial_state\s*=\s*ABSOLUTE\(0x20001000\)')
{
    throw "App map cannot expose the shared trial state address"
}

Write-Output "PASS bootloader layout contract"
