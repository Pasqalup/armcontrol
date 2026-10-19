@echo off
setlocal

set "PROJECT_DIR=%~dp0"
set "BUILD_DIR=%PROJECT_DIR%build"

cmake -S %PROJECT_DIR% -B %BUILD_DIR%
if errorlevel 1 exit /b %errorlevel%

cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b %errorlevel%

echo Build completed successfully.
endlocal