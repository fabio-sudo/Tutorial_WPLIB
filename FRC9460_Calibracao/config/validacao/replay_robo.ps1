param(
    [Parameter(Mandatory = $true)]
    [string]$Log
)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$sourceLog = (Resolve-Path -LiteralPath $Log.Trim('"', "'")).Path
if ([IO.Path]::GetExtension($sourceLog) -ne '.wpilog') {
    throw 'Selecione um arquivo .wpilog do AdvantageKit.'
}
# Com a simulação aberta o log ainda está sendo gravado: o Windows nega a leitura,
# e um replay feito assim sairia incompleto. Log antigo já fechado passa normalmente.
try {
    $reader = [IO.File]::OpenRead($sourceLog)
} catch {
    Write-Output 'Esse log ainda esta sendo gravado: feche a simulacao e rode de novo.'
    exit 1
}
try {
    $header = New-Object byte[] 6
    if ($reader.Read($header, 0, 6) -ne 6 -or [Text.Encoding]::ASCII.GetString($header) -ne 'WPILOG') {
        throw 'O arquivo nao tem um cabecalho WPILOG valido.'
    }
} finally {
    $reader.Dispose()
}

# Cada execução tem sua cópia: preserva o original e os replays anteriores.
$runName = [DateTime]::UtcNow.ToString('yyyyMMdd_HHmmss') + '_' + [Guid]::NewGuid().ToString('N').Substring(0, 8)
$runDirectory = Join-Path $projectRoot ('logs/replay/' + $runName)
New-Item -ItemType Directory -Path $runDirectory | Out-Null
$inputCopy = Join-Path $runDirectory ([IO.Path]::GetFileName($sourceLog))
Copy-Item -LiteralPath $sourceLog -Destination $inputCopy

if (-not $env:JAVA_HOME -and (Test-Path -LiteralPath 'C:/Users/Public/wpilib/2026/jdk/bin/java.exe')) {
    $env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
}
Push-Location -LiteralPath $projectRoot
try {
    & (Join-Path $projectRoot 'gradlew.bat') simulateJava "-PreplayLog=$inputCopy" --offline --console=plain
    if ($LASTEXITCODE -ne 0) { throw "Replay falhou (codigo $LASTEXITCODE). Consulte a mensagem acima." }
    $result = Join-Path $runDirectory ([IO.Path]::GetFileNameWithoutExtension($inputCopy) + '_replay.wpilog')
    if (-not (Test-Path -LiteralPath $result)) { throw 'O replay nao gerou o arquivo de saida esperado.' }
    Write-Output "Replay concluido: $result"
    Write-Output 'AdvantageScope > File > Import Layout > config/advantagescope/layouts/FRC9460-Replay.json'
} finally {
    Pop-Location
}
