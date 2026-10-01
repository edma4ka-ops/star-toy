# Выкладывание страниц игрушки на боевой сервер.
#   powershell -ExecutionPolicy Bypass -File deploy\deploy-site.ps1
#
# Что делает:
#   1. собирает архив из папки site (страницы, стили, архивы с программами);
#   2. копирует его и скрипт установки на сервер;
#   3. запускает установку и проверку страниц на сервере.
#
# Отдельно от развёртывания прибора: тот ставит код в /opt/pooldesk,
# а страницы игрушки живут в /opt/star-toy и переживают его развёртывания.

param(
    [string]$Server    = 'aks-server',
    [string]$RemoteTmp = '/tmp'
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$dist = Join-Path $repo 'dist'
$archive = Join-Path $dist 'star-toy-site.tar.gz'

# PATH в свежем терминале может не знать про git и ssh: перечитываем из реестра
function Update-PathFromRegistry {
    $machine = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $user = [Environment]::GetEnvironmentVariable('Path', 'User')
    $env:Path = (@($machine, $user) | Where-Object { $_ }) -join ';'
}
Update-PathFromRegistry

foreach ($tool in 'tar', 'ssh', 'scp') {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "Не найдена команда $tool."
    }
}

New-Item -ItemType Directory -Force -Path $dist | Out-Null
if (Test-Path $archive) { Remove-Item $archive -Force }

Write-Host "[toy] собираю архив страниц" -ForegroundColor Cyan
Push-Location (Join-Path $repo 'site')
try {
    # Точка входа: содержимое site кладём в корень архива, чтобы страница
    # открывалась по адресу /toy/, а не /toy/site/
    tar -czf $archive .
    if ($LASTEXITCODE -ne 0) { throw 'сборка архива не удалась' }
} finally {
    Pop-Location
}
$size = [math]::Round((Get-Item $archive).Length / 1KB, 1)
Write-Host "[toy] архив собран: $size КБ"

Write-Host "[toy] копирую на $Server" -ForegroundColor Cyan
scp $archive "${Server}:${RemoteTmp}/star-toy-site.tar.gz"
if ($LASTEXITCODE -ne 0) { throw 'scp архива не удался' }

# Скрипт установки чистим от CRLF: bash спотыкается о \r в последней строке
$setupLocal = Join-Path $PSScriptRoot 'setup-site.sh'
$setupClean = Join-Path $dist 'setup-site.sh'
$setupText = ([IO.File]::ReadAllText($setupLocal)) -replace "`r`n", "`n" -replace "`r", "`n"
[IO.File]::WriteAllText($setupClean, $setupText, (New-Object System.Text.UTF8Encoding($false)))

scp $setupClean "${Server}:${RemoteTmp}/setup-star-toy.sh"
if ($LASTEXITCODE -ne 0) { throw 'scp скрипта не удался' }

Write-Host "[toy] запускаю установку" -ForegroundColor Cyan
ssh $Server "bash ${RemoteTmp}/setup-star-toy.sh"
if ($LASTEXITCODE -ne 0) { throw 'установка на сервере завершилась с ошибкой' }

Write-Host "[toy] готово: http://194.87.200.210/toy/" -ForegroundColor Green
