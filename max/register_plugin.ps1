# Registers a plugin folder with 3ds Max through the per-user Plugin.UserSettings.ini (UTF-16),
# as `<Name>=<Folder>\` under [Directories]. Idempotent: an existing entry is updated in place.
# An entry for the same folder under another name (the toolkit was renamed) is replaced, since
# Max would otherwise load the folder's plugins twice.
param(
    [Parameter(Mandatory)][string]$Ini,
    [Parameter(Mandatory)][string]$Folder,
    [string]$Name = 'exodus'
)

$value = $Folder.TrimEnd('\', '/') + '\'
$entry = "$Name=$value"
$lines = [System.Collections.Generic.List[string]]::new()
if (Test-Path -LiteralPath $Ini) {
    foreach ($line in Get-Content -LiteralPath $Ini -Encoding Unicode) { $lines.Add($line) }
}
$before = $lines -join "`n"

$sameFolder = {
    param($l)
    $eq = $l.IndexOf('=')
    $eq -gt 0 -and -not $l.StartsWith('[') -and $l.Substring($eq + 1).TrimEnd('\', '/') -ieq $value.TrimEnd('\')
}
$at = $lines.FindIndex([Predicate[string]] { param($l) $l.StartsWith("$Name=") })
if ($at -lt 0) { $at = $lines.FindIndex([Predicate[string]] $sameFolder) }
if ($at -ge 0) {
    $lines[$at] = $entry
} else {
    $section = $lines.FindIndex([Predicate[string]] { param($l) $l.Trim() -eq '[Directories]' })
    if ($section -lt 0) { $lines.Insert(0, '[Directories]'); $section = 0 }
    $at = $section + 1
    while ($at -lt $lines.Count -and $lines[$at].Trim() -and -not $lines[$at].StartsWith('[')) { $at++ }
    $lines.Insert($at, $entry)
}
for ($k = $lines.Count - 1; $k -ge 0; $k--) {
    if ($k -ne $at -and (& $sameFolder $lines[$k])) {
        Write-Output "  removed: $($lines[$k])"
        $lines.RemoveAt($k)
        if ($k -lt $at) { $at-- }
    }
}

if (($lines -join "`n") -eq $before) { Write-Output "  already registered: $entry"; exit 0 }
New-Item -ItemType Directory -Force -Path (Split-Path -LiteralPath $Ini) | Out-Null
Set-Content -LiteralPath $Ini -Value $lines -Encoding Unicode
Write-Output "  registered: $entry"
