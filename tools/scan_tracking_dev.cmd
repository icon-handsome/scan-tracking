@echo off
setlocal EnableExtensions

rem ScanTracking dev helper (CMakePresets + README).

for %%I in ("%~dp0..") do set "ROOT=%%~fI"
cd /d "%ROOT%"
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
if not exist "D:\HikCameraFTP" mkdir "D:\HikCameraFTP"
if exist "%ROOT%\third_party\LB\data" robocopy "%ROOT%\third_party\LB\data" "%APP_DEBUG%\data\LB" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\third_party\LB\template_for_scanner_ori.txt" copy /Y "%ROOT%\third_party\LB\template_for_scanner_ori.txt" "%APP_DEBUG%\data\LB\template_for_scanner_ori.txt" >nul
if exist "%ROOT%\third_party\LBN\data" robocopy "%ROOT%\third_party\LBN\data" "%APP_DEBUG%\data\LBN" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%ROOT%\third_party\Scanner_Self_Check\self_check.ini" (
  if not exist "%APP_DEBUG%\self_check" mkdir "%APP_DEBUG%\self_check"
  copy /Y "%ROOT%\third_party\Scanner_Self_Check\self_check.ini" "%APP_DEBUG%\self_check\self_check.ini" >nul
)
if exist "%APP_DEBUG%\internal_surface\tmp" rmdir /S /Q "%APP_DEBUG%\internal_surface\tmp"
if not exist "%APP_DEBUG%\config.ini" (
  copy /Y "%ROOT%\config.ini" "%APP_DEBUG%\config.ini" >nul
  echo [scan_tracking_dev] Created %APP_DEBUG%\config.ini from repo template
) else (
  echo [scan_tracking_dev] Kept existing %APP_DEBUG%\config.ini ^(field IPs/paths^)
)
powershell -NoProfile -Command "(Get-Content -LiteralPath '%APP_DEBUG%\config.ini' -Raw) -replace '(?m)^scanSegmentTotal=.*','scanSegmentTotal=144' | Set-Content -LiteralPath '%APP_DEBUG%\config.ini' -NoNewline"
echo [scan_tracking_dev] Updated scanSegmentTotal=144 in deploy config.ini
rem Debug 包在工控机无 VS 时缺 ucrtbased.dll，会导致 0xC0000135 静默退出（windeployqt --no-compiler-runtime）
if exist "%SystemRoot%\System32\ucrtbased.dll" (
  copy /Y "%SystemRoot%\System32\ucrtbased.dll" "%APP_DEBUG%\" >nul
  echo [scan_tracking_dev] Copied ucrtbased.dll for IPC debug deploy
)
if exist "%SystemRoot%\System32\vcruntime140_1d.dll" (
  copy /Y "%SystemRoot%\System32\vcruntime140_1d.dll" "%APP_DEBUG%\" >nul
  echo [scan_tracking_dev] Copied vcruntime140_1d.dll for IPC debug deploy
)
echo [scan_tracking_dev] Stage complete: %APP_DEBUG%
exit /b 0

:deploy_debug
call :build_debug
if errorlevel 1 exit /b 1
call :stage_deploy
exit /b 0

:package_ipc
set "PACKAGE_DIR=%DEPLOY_OUTPUT_DIR%"
set "DEPLOY_BACKUP=%TEMP%\scan_tracking_deploy_backup"
echo [scan_tracking_dev] Packaging IPC runtime to %PACKAGE_DIR%
if exist "%DEPLOY_BACKUP%" rmdir /S /Q "%DEPLOY_BACKUP%"
if exist "%PACKAGE_DIR%" (
  mkdir "%DEPLOY_BACKUP%"
  if exist "%PACKAGE_DIR%\config.ini" copy /Y "%PACKAGE_DIR%\config.ini" "%DEPLOY_BACKUP%\config.ini" >nul
  if exist "%PACKAGE_DIR%\config.ini.field_backup" copy /Y "%PACKAGE_DIR%\config.ini.field_backup" "%DEPLOY_BACKUP%\config.ini.field_backup" >nul
  if exist "%PACKAGE_DIR%\logs" robocopy "%PACKAGE_DIR%\logs" "%DEPLOY_BACKUP%\logs" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
  if exist "%PACKAGE_DIR%\output" robocopy "%PACKAGE_DIR%\output" "%DEPLOY_BACKUP%\output" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
  if exist "%PACKAGE_DIR%\testdata" robocopy "%PACKAGE_DIR%\testdata" "%DEPLOY_BACKUP%\testdata" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
)
call :build_debug
if errorlevel 1 exit /b 1
call :stage_deploy
if errorlevel 1 exit /b 1
if not exist "%APP_DEBUG%\scan-tracking.exe" (
  echo [scan_tracking_dev] Missing %APP_DEBUG%\scan-tracking.exe - run deploy-debug first
  exit /b 1
)
if exist "%PACKAGE_DIR%" rmdir /S /Q "%PACKAGE_DIR%"
mkdir "%PACKAGE_DIR%"
robocopy "%APP_DEBUG%" "%PACKAGE_DIR%" /E /NFL /NDL /NJH /NJS /nc /ns /np ^
  /XD CMakeFiles scan_tracking_autogen .git .vs .cache app common modules tests internal_surface\tmp ^
  /XF *.obj *.pdb cmake_install.cmake CMakeCache.txt build.ninja .ninja_deps .ninja_log >nul
if errorlevel 8 exit /b 1
if exist "%DEPLOY_BACKUP%\config.ini" (
  copy /Y "%DEPLOY_BACKUP%\config.ini" "%PACKAGE_DIR%\config.ini" >nul
  echo [scan_tracking_dev] Restored field config.ini from previous deploy
) else (
  copy /Y "%ROOT%\config.ini" "%PACKAGE_DIR%\config.ini" >nul
  echo [scan_tracking_dev] Seeded config.ini from repo template
)
if exist "%DEPLOY_BACKUP%\config.ini.field_backup" copy /Y "%DEPLOY_BACKUP%\config.ini.field_backup" "%PACKAGE_DIR%\config.ini.field_backup" >nul
if exist "%DEPLOY_BACKUP%\logs" robocopy "%DEPLOY_BACKUP%\logs" "%PACKAGE_DIR%\logs" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%DEPLOY_BACKUP%\output" robocopy "%DEPLOY_BACKUP%\output" "%PACKAGE_DIR%\output" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
if exist "%DEPLOY_BACKUP%\testdata" robocopy "%DEPLOY_BACKUP%\testdata" "%PACKAGE_DIR%\testdata" /E /NFL /NDL /NJH /NJS /nc /ns /np >nul
copy /Y "%ROOT%\tools\deploy\start.bat" "%PACKAGE_DIR%\start.bat" >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%ROOT%\tools\deploy\write_readme.ps1" -OutputDir "%PACKAGE_DIR%" -BuildDir "%APP_DEBUG%" -DeployDir "%DEPLOY_OUTPUT_DIR%"
if exist "%DEPLOY_BACKUP%" rmdir /S /Q "%DEPLOY_BACKUP%"
echo [scan_tracking_dev] Package ready: %PACKAGE_DIR%
echo [scan_tracking_dev] Zip this folder and copy to the IPC machine.
exit /b 0

:apply_runtime_path
set "APP_DIR=%~1"
set "PATH=%APP_DIR%;%APP_DIR%\mech_eye_api;%APP_DIR%\ThirdParty;%APP_DIR%\hik_mvs_runtime;%APP_DIR%\OpenNI2;%APP_DIR%\OpenNI2\Drivers;C:\Qt\5.15.2\msvc2019_64\bin;C:\Program Files\PCL 1.12.0\bin;C:\Program Files\PCL 1.12.0\3rdParty\VTK\bin;C:\Program Files\OpenNI2\Redist;%ROOT%\third_party\opencv-3.4.3-vc14_vc15\opencv\build\x64\vc15\bin;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll;%ROOT%\third_party\Mech-Eye SDK-2.5.4\API\dll_debug;%PATH%"
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
