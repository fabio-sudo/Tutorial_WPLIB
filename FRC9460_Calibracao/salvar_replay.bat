@echo off
rem Salva o replay de um log do AdvantageKit.
rem Duplo clique: usa o log mais recente de logs\simulacao.
rem Arrastar um .wpilog sobre este arquivo: usa esse log (robo ou simulacao).
rem O resultado fica em logs\replay\<data>, e o log original nao muda.
setlocal
cd /d "%~dp0"

set "LOG=%~1"
if defined LOG goto :replay
for /f "delims=" %%f in ('dir /b /o-d /a-d "logs\simulacao\*.wpilog" 2^>nul ^| findstr /v /i "_replay"') do (
    set "LOG=logs\simulacao\%%f"
    goto :replay
)
echo Nenhum log encontrado em logs\simulacao.
echo Rode a simulacao antes ou arraste um .wpilog sobre este arquivo.
pause
exit /b 1

:replay
echo Log: %LOG%
powershell -NoProfile -ExecutionPolicy Bypass -File "config\validacao\replay_robo.ps1" -Log "%LOG%"
if errorlevel 1 (
    echo.
    echo O replay nao foi salvo. Veja a mensagem acima.
    pause
    exit /b 1
)
echo.
echo Replay salvo em logs\replay.
pause
