@echo off
setlocal EnableExtensions

rem ScanTracking dev helper (CMakePresets + README). Used by Cursor/VS Code tasks.

for %%I in ("%~dp0..") do set "ROOT=%%~fI"
set "CMAKE=C:\Program Files\CMake\bin\cmake.exe"
set "PRESET_DEBUG=win-msvc2019-qtcore-ninja-debug"
set "PRESET_RELEASE=win-msvc2019-qtcore-ninja-release"
set "APP_DEBUG=%ROOT%\build\%PRESET_DEBUG%"
set "APP_RELEASE=%ROOT%\build\%PRESET_RELEASE%"
set "DEPLOY_OUTPUT_DIR=D:\work\LY\deploy_debug"

if "%~1"=="" goto usage

if /I "%~1"=="configure-debug" goto configure_debug
if /I "%~1"=="configure-release" goto configure_release
if /I "%~1"=="build-debug" goto build_debug
if /I "%~1"=="build-release" goto build_release
if /I "%~1"=="run-debug" goto run_debug
if /I "%~1"=="run-release" goto run_release
if /I "%~1"=="stage-deploy" goto stage_deploy
if /I "%~1"=="deploy-debug" goto deploy_debug
if /I "%~1"=="package-ipc" goto package_ipc
goto usage

:configure_debug
"%CMAKE%" --preset %PRESET_DEBUG% -S "%ROOT%"
exit /b %ERRORLEVEL%

:configure_release
"%CMAKE%" --preset %PRESET_RELEASE% -S "%ROOT%"
exit /b %ERRORLEVEL%

:build_debug
"%CMAKE%" --build --preset %PRESET_DEBUG%
exit /b %ERRORLEVEL%

:build_release
"%CMAKE%" --build --preset %PRESET_RELEASE%
exit /b %ERRORLEVEL%

:run_debug
call :apply_runtime_path "%APP_DEBUG%"
cd /d "%APP_DEBUG%"
if not exist "%APP_DEBUG%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_DEBUG%\scan-tracking.exe - run build-debug first
  exit /b 1
)
"%APP_DEBUG%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:run_release
call :apply_runtime_path "%APP_RELEASE%"
cd /d "%APP_RELEASE%"
if not exist "%APP_RELEASE%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_RELEASE%\scan-tracking.exe - run build-release first
  exit /b 1
)
"%APP_RELEASE%\scan-tracking.exe" %*
exit /b %ERRORLEVEL%

:stage_deploy
echo [scan_tracking_dev] Staging runtime configs to %APP_DEBUG%
if not exist "%APP_DEBUG%" mkdir "%APP_DEBUG%"
robocopy "%ROOT%\config" "%APP_DEBUG%\config" /E /NFL /NDL /NJH /NJS /nc /ns /np /XD .git >nul
if errorlevel 8 exit /b 1
copy /Y "%ROOT%\scan_paths_config.json" "%APP_DEBUG%\scan_paths_config.json" >nul
if exist "%ROOT%\bevel" robocopy "%ROOT%\bevel" "%APP_DEBUG%\bevel" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\hole" robocopy "%ROOT%\hole" "%APP_DEBUG%\hole" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\thickness" robocopy "%ROOT%\thickness" "%APP_DEBUG%\thickness" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\internal_surface" robocopy "%ROOT%\internal_surface" "%APP_DEBUG%\internal_surface" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\third_party\LB" robocopy "%ROOT%\third_party\LB" "%APP_DEBUG%\third_party\LB" /E /NFL /NDL /NJH /NJS /nc /ns /np /XF *.obj *.pdb *.lib *.exp >nul
if exist "%ROOT%\third_party\LBN" robocopy "%ROOT%\third_party\LBN" "%APP_DEBUG%\third_party\LBN" /E /NFL /NDL /NJH /NJS /nc /ns /np /XF *.obj *.pdb *.lib *.exp >nul
if not exist "%APP_DEBUG%\data\LB" mkdir "%APP_DEBUG%\data\LB"
if not exist "%APP_DEBUG%\data\LBN" mkdir "%APP_DEBUG%\data\LBN"
if exist "%ROOT%\third_party\LB\data" robocopy "%ROOT%\third_party\LB\data" "%APP_DEBUG%\data\LB" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\third_party\LB\template_for_scanner_ori.txt" copy /Y "%ROOT%\third_party\LB\template_for_scanner_ori.txt" "%APP_DEBUG%\data\LB\template_for_scanner_ori.txt" >nul
if exist "%ROOT%\third_party\LBN\data" robocopy "%ROOT%\third_party\LBN\data" "%APP_DEBUG%\data\LBN" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if not exist "%APP_DEBUG%\config.ini" (
  copy /Y "%ROOT%\config.ini" "%APP_DEBUG%\config.ini" >nul
  echo [scan_tracking_dev] Created %APP_DEBUG%\config.ini from repo template
) else (
  echo [scan_tracking_dev] Kept existing %APP_DEBUG%\config.ini ^(field IPs/paths^)
)
powershell -NoProfile -Command "(Get-Content -LiteralPath '%APP_DEBUG%\config.ini' -Raw) -replace '(?m)^scanSegmentTotal=.*','scanSegmentTotal=144' | Set-Content -LiteralPath '%APP_DEBUG%\config.ini' -NoNewline"
echo [scan_tracking_dev] Updated scanSegmentTotal=144 in deploy config.ini
echo [scan_tracking_dev] Stage complete: %APP_DEBUG%
exit /b 0

:deploy_debug
call :build_debug
if errorlevel 1 exit /b 1
call :stage_deploy
exit /b 0

:package_ipc
set "PACKAGE_DIR=%DEPLOY_OUTPUT_DIR%"
echo [scan_tracking_dev] Packaging IPC runtime to %PACKAGE_DIR%
if exist "%PACKAGE_DIR%" rmdir /S /Q "%PACKAGE_DIR%"
mkdir "%PACKAGE_DIR%"
robocopy "%APP_DEBUG%" "%PACKAGE_DIR%" /E /NFL /NDL /NJH /NJS /nc /ns /np ^
  /XD CMakeFiles scan_tracking_autogen .git .vs .cache app common modules tests ^
  /XF *.obj *.pdb cmake_install.cmake CMakeCache.txt build.ninja .ninja_deps .ninja_log >nul
if errorlevel 8 exit /b 1
echo [scan_tracking_dev] Package ready: %PACKAGE_DIR%
echo [scan_tracking_dev] Zip this folder and copy to the IPC machine.
exit /b 0

:apply_runtime_path
set "APP_DIR=%~1"
set "PATH=%APP_DIR%;%APP_DIR%\mech_eye_api;%APP_DIR%\ThirdParty;%APP_DIR%\hik_mvs_runtime;%APP_DIR%\OpenNI2;%APP_DIR%\OpenNI2\Drivers;C:\Qt\5.15.2\msvc2019_64\bin;C:\Program Files\PCL 1.12.0\bin;C:\Program Files\PCL 1.12.0\3rdParty\VTK\bin;C:\Program Files\OpenNI2\Redist;%ROOT%\third_party\LB\opencv-3.4.3-vc14_vc15\opencv\build\x64\vc15\bin;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll_debug;%PATH%"
exit /b 0

:usage
echo.
echo Usage:
echo   scan_tracking_dev.cmd configure-debug ^| configure-release
echo   scan_tracking_dev.cmd build-debug     ^| build-release
echo   scan_tracking_dev.cmd run-debug       ^| run-release
echo   scan_tracking_dev.cmd stage-deploy    ^| deploy-debug ^(build + stage^)
echo   scan_tracking_dev.cmd package-ipc       ^(clean folder for IPC zip^)
echo.
echo Debug build/run directory: %APP_DEBUG%
echo.
exit /b 1
