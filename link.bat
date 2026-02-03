rem link.bat
rem === 权限提升部分 ===
FSUTIL DIRTY query %SystemDrive% >NUL 2>&1
if errorlevel 1 (
    echo 正在提升权限…
    powershell -Command "Start-Process -FilePath '%~f0' -ArgumentList 'elevated$
    timeout /t 10
    exit
)
if "%1" neq "elevated" (
    echo 正在提升权限…
    powershell -Command "Start-Process '%~f0' -ArgumentList 'elevated' -Verb Ru$
    timeout /t 10
    exit
)
:Elevated
echo 已获得管理员权限。

rem === 当前脚本所在目录 ===
set "SCRIPT_DIR=%~dp0"
echo script dir: %SCRIPT_DIR%


mklink /D %SCRIPT_DIR%/RoadRunner %SCRIPT_DIR%/../road-runner-ftc/RoadRunner
mklink /D %SCRIPT_DIR%/road-runner-actions %SCRIPT_DIR%/../road-runner/road-runner/actions
mklink /D %SCRIPT_DIR%/road-runner-core %SCRIPT_DIR%/../road-runner/road-runner/core
mklink /D %SCRIPT_DIR%/FtcDashboard %SCRIPT_DIR%/../ftc-dashboard/FtcDashboard
mklink /D %SCRIPT_DIR%/DashboardCore %SCRIPT_DIR%/../ftc-dashboard/DashboardCore