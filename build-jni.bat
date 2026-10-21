@echo off
setlocal EnableExtensions

set "PROJECT_DIR=%~dp0"
set "BUILD_DIR=%PROJECT_DIR%build\host-jni"

if not defined JAVA_HOME (
    for /f "delims=" %%J in ('where javac 2^>nul') do (
        if not defined JAVA_HOME (
            for %%D in ("%%~dpJ..") do set "JAVA_HOME=%%~fD"
        )
    )
)

if not defined JAVA_HOME (
    echo ERROR: JAVA_HOME is not set and javac was not found on PATH.
    exit /b 1
)

if not exist "%JAVA_HOME%\bin\javac.exe" (
    echo ERROR: javac was not found under "%JAVA_HOME%\bin".
    exit /b 1
)

pushd "%PROJECT_DIR%"

cmake --preset host-jni
if errorlevel 1 (
    popd
    exit /b 1
)

cmake --build --preset host-jni
if errorlevel 1 (
    popd
    exit /b 1
)

set "JNI_DLL=%BUILD_DIR%\Release\modeled_jni.dll"
if not exist "%JNI_DLL%" set "JNI_DLL=%BUILD_DIR%\modeled_jni.dll"
if not exist "%JNI_DLL%" (
    echo ERROR: modeled_jni.dll was not produced under "%BUILD_DIR%".
    popd
    exit /b 1
)

for %%F in ("%JNI_DLL%") do set "JNI_DIR=%%~dpF"
if "%JNI_DIR:~-1%"=="\" set "JNI_DIR=%JNI_DIR:~0,-1%"

set "TEST_CLASSES=%BUILD_DIR%\jni-test\classes"
if exist "%TEST_CLASSES%" rmdir /s /q "%TEST_CLASSES%"
mkdir "%TEST_CLASSES%"
if errorlevel 1 (
    popd
    exit /b 1
)

"%JAVA_HOME%\bin\javac.exe" -d "%TEST_CLASSES%" ^
    "%PROJECT_DIR%jni\java\org\firstinspires\ftc\teamcode\modeled\NativePipeline.java" ^
    "%PROJECT_DIR%jni\test\SmokeTest.java"
if errorlevel 1 (
    popd
    exit /b 1
)

"%JAVA_HOME%\bin\java.exe" -Djava.library.path="%JNI_DIR%" -cp "%TEST_CLASSES%" SmokeTest
if errorlevel 1 (
    popd
    exit /b 1
)

popd
echo JNI build and smoke test completed successfully.
endlocal
