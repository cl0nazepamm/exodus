# Registers a plugin folder with 3ds Max through the per-user Plugin.UserSettings.ini (UTF-16),
# as `<Name>=<Folder>\` under [Directories]. Idempotent: an existing entry is updated in place.
param(
    [Parameter(Mandatory)][string]$Ini,
    [Parameter(Mandatory)][string]$Folder,
    [string]$Name = 'udimpacker'
)

$entry = "$Name=" + $Folder.TrimEnd('\', '/') + '\'
$lines = [System.Collections.Generic.List[string]]::new()
if (Test-Path -LiteralPath $Ini) {
    foreach ($line in Get-Content -LiteralPath $Ini -Encoding Unicode) { $lines.Add($line) }
}

$at = $lines.FindIndex([Predicate[string]] { param($l) $l.StartsWith("$Name=") })
if ($at -ge 0) {
    if ($lines[$at] -eq $entry) { Write-Output "  already registered: $entry"; exit 0 }
    $lines[$at] = $entry
} else {
    $section = $lines.FindIndex([Predicate[string]] { param($l) $l.Trim() -eq '[Directories]' })
    if ($section -lt 0) { $lines.Insert(0, '[Directories]'); $section = 0 }
    $i = $section + 1
    while ($i -lt $lines.Count -and $lines[$i].Trim() -and -not $lines[$i].StartsWith('[')) { $i++ }
    $lines.Insert($i, $entry)
}
New-Item -ItemType Directory -Force -Path (Split-Path -LiteralPath $Ini) | Out-Null
Set-Content -LiteralPath $Ini -Value $lines -Encoding Unicode
Write-Output "  registered: $entry"
