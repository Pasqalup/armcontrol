@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "PROJECT_DIR=%~dp0"
set "DIST_DIR=%PROJECT_DIR%dist\jniLibs"

if not "%~1"=="" set "ANDROID_NDK_HOME=%~1"

if not defined ANDROID_NDK_HOME (
    if defined ANDROID_SDK_ROOT set "SDK_DIR=%ANDROID_SDK_ROOT%"
    if not defined SDK_DIR if defined ANDROID_HOME set "SDK_DIR=%ANDROID_HOME%"
    if not defined SDK_DIR set "SDK_DIR=%LOCALAPPDATA%\Android\Sdk"

    if exist "!SDK_DIR!\ndk" (
        for /f "delims=" %%D in ('dir /b /ad /o-n "!SDK_DIR!\ndk" 2^>nul') do (
            if not defined ANDROID_NDK_HOME (
                set "ANDROID_NDK_HOME=!SDK_DIR!\ndk\%%D"
            )
        )
    )
)

if not defined ANDROID_NDK_HOME (
    echo ERROR: Android NDK not found. Set ANDROID_NDK_HOME or install an NDK.
    exit /b 1
)

if not exist "%ANDROID_NDK_HOME%\build\cmake\android.toolchain.cmake" (
    echo ERROR: Invalid Android NDK at "%ANDROID_NDK_HOME%".
    exit /b 1
)

where make >nul 2>nul
if errorlevel 1 (
    echo ERROR: GNU make was not found on PATH.
    exit /b 1
)

make --version 2>nul | findstr /i "GNU Make" >nul
if errorlevel 1 (
    echo ERROR: "make" on PATH is not GNU Make.
    exit /b 1
)

pushd "%PROJECT_DIR%"

call :build_abi android-armv7a armeabi-v7a
if errorlevel 1 (
    popd
    exit /b 1
)

call :build_abi android-arm64 arm64-v8a
if errorlevel 1 (
    popd
    exit /b 1
)

call :build_abi android-x86_64 x86_64
if errorlevel 1 (
    popd
    exit /b 1
)

popd
echo Android JNI build completed: "%DIST_DIR%"
endlocal
exit /b 0

:build_abi
set "PRESET=%~1"
set "ABI=%~2"

echo Configuring %PRESET%...
cmake --preset %PRESET%
if errorlevel 1 exit /b 1

cmake --build --preset %PRESET%
if errorlevel 1 exit /b 1

set "SOURCE_LIB=%PROJECT_DIR%build\android\%ABI%\libmodeled_jni.so"
set "DEST_DIR=%DIST_DIR%\%ABI%"
if not exist "%SOURCE_LIB%" (
    echo ERROR: Missing "%SOURCE_LIB%".
    exit /b 1
)
if not exist "%DEST_DIR%" mkdir "%DEST_DIR%"
copy /Y "%SOURCE_LIB%" "%DEST_DIR%\libmodeled_jni.so" >nul
if errorlevel 1 exit /b 1

exit /b 0
