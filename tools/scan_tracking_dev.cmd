@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "REPO_ROOT=%%~fI"

set "CMAKE_EXE=C:\Program Files\CMake\bin\cmake.exe"
set "BUILD_DIR=%REPO_ROOT%\build\win-msvc2019-qtcore-ninja-debug"
set "RELEASE_BUILD_DIR=%REPO_ROOT%\build\win-msvc2019-qtcore-ninja-release"
set "DEBUG_PRESET=win-msvc2019-qtcore-ninja-debug"
set "RELEASE_PRESET=win-msvc2019-qtcore-ninja-release"

if not exist "%CMAKE_EXE%" (
  echo cmake.exe not found: "%CMAKE_EXE%"
  exit /b 1
)

set "ACTION=%~1"
if "%ACTION%"=="" set "ACTION=configure-debug"

if /I "%ACTION%"=="configure-debug" goto :configure_debug
if /I "%ACTION%"=="build-debug" goto :build_debug
if /I "%ACTION%"=="run-debug" goto :run_debug
if /I "%ACTION%"=="configure-release" goto :configure_release
if /I "%ACTION%"=="build-release" goto :build_release
if /I "%ACTION%"=="run-release" goto :run_release

echo Unknown action: %ACTION%
echo Supported actions: configure-debug build-debug run-debug configure-release build-release run-release
exit /b 1

:configure_debug
"%CMAKE_EXE%" --preset "%DEBUG_PRESET%"
exit /b %errorlevel%

:build_debug
"%CMAKE_EXE%" --build --preset "%DEBUG_PRESET%"
exit /b %errorlevel%

:run_debug
if not exist "%BUILD_DIR%" (
  "%CMAKE_EXE%" --preset "%DEBUG_PRESET%"
  if errorlevel 1 exit /b 1
)
"%CMAKE_EXE%" --build --preset "%DEBUG_PRESET%"
if errorlevel 1 exit /b 1
start "" /D "%BUILD_DIR%\app" "%BUILD_DIR%\app\scan-tracking.exe"
exit /b %errorlevel%

:configure_release
"%CMAKE_EXE%" --preset "%RELEASE_PRESET%"
exit /b %errorlevel%

:build_release
"%CMAKE_EXE%" --build --preset "%RELEASE_PRESET%"
exit /b %errorlevel%

:run_release
if not exist "%RELEASE_BUILD_DIR%" (
  "%CMAKE_EXE%" --preset "%RELEASE_PRESET%"
  if errorlevel 1 exit /b 1
)
"%CMAKE_EXE%" --build --preset "%RELEASE_PRESET%"
if errorlevel 1 exit /b 1
start "" /D "%RELEASE_BUILD_DIR%\app" "%RELEASE_BUILD_DIR%\app\scan-tracking.exe"
exit /b %errorlevel%
