@echo off
setlocal EnableExtensions

set "PROJ=%~dp0"
set "PROJ=%PROJ:~0,-1%"
cd /d "%PROJ%"

set "ESP_PORT=COM3"
set "ESP_BAUD=460800"
if not "%~1"=="" set "ESP_PORT=%~1"

set "IDF_TOOLS=D:\Espressif"
set "IDF_PATH=%IDF_TOOLS%\frameworks\esp-idf-v5.1.2"
set "XTENSA=%IDF_TOOLS%\tools\xtensa-esp32s3-elf\esp-12.2.0_20230208\xtensa-esp32s3-elf\bin"
set "PYTHON_DIR=%IDF_TOOLS%\python_env\idf5.1_py3.11_env\Scripts"
set "PYTHON=%PYTHON_DIR%\python.exe"
set "ESPTOOL=%PYTHON% %IDF_PATH%\components\esptool_py\esptool\esptool.py"
set "NINJA=%IDF_TOOLS%\tools\ninja\1.10.2\ninja.exe"
set "CMAKE=%IDF_TOOLS%\tools\cmake\3.24.0\bin\cmake.exe"

rem ---- Toolchain + IDF Python first in PATH (for ninja sub-shells) ----
set "PATH=%XTENSA%;%PYTHON_DIR%;%PATH%"
set "PYTHONUTF8=1"

echo ============================================
echo   ESP32-S3 Build ^& Flash (Voice + Sensors)
echo   Port: %ESP_PORT%
echo ============================================
echo.

rem ---------- Step 0: CMake Configure (first time only) ----------
if not exist "%PROJ%\build\build.ninja" (
    echo [0/3] Configuring CMake...
    if not exist "%PROJ%\build" mkdir "%PROJ%\build"
    "%CMAKE%" -S "%PROJ%" -B "%PROJ%\build" -G Ninja -DCMAKE_BUILD_TYPE=MinSizeRel -DCMAKE_TOOLCHAIN_FILE="%IDF_PATH%\tools\cmake\toolchain-esp32s3.cmake" -DIDF_TARGET=esp32s3 -DSDKCONFIG="%PROJ%\sdkconfig" -DSDKCONFIG_DEFAULTS="%PROJ%\sdkconfig.defaults"
    if errorlevel 1 (
        echo CMAKE CONFIGURE FAILED
        pause
        exit /b 1
    )
    echo.
)

rem ---------- Step 1: Build ----------
echo [1/3] Building firmware...
"%NINJA%" -C "%PROJ%\build"
if errorlevel 1 (
    echo BUILD FAILED
    pause
    exit /b 1
)
echo.

rem ---------- Step 2: Pack wake model ----------
echo [2/3] Packing wake word model...
if not exist "%PROJ%\build\srmodels" mkdir "%PROJ%\build\srmodels"
"%PYTHON%" "%PROJ%\managed_components\espressif__esp-sr\model\movemodel.py" -d1 "%PROJ%\sdkconfig" -d2 "%PROJ%\managed_components\espressif__esp-sr" -d3 "%PROJ%\build"
if errorlevel 1 (
    echo MODEL PACK FAILED
    pause
    exit /b 1
)
echo.

rem ---------- Step 3: Flash ----------
echo [3/3] Flashing to %ESP_PORT% at %ESP_BAUD% baud...
"%PYTHON%" "%IDF_PATH%\components\esptool_py\esptool\esptool.py" -p %ESP_PORT% -b %ESP_BAUD% --before default_reset --after hard_reset --chip esp32s3 write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 "%PROJ%\build\bootloader\bootloader.bin" 0x8000 "%PROJ%\build\partition_table\partition-table.bin" 0x10000 "%PROJ%\build\pump_control.bin" 0x510000 "%PROJ%\build\srmodels\srmodels.bin"
if errorlevel 1 goto flash_fail

echo.
echo ============================================
echo   Done. ESP32 restarting...
echo ============================================
pause
exit /b 0

:flash_fail
echo.
echo FLASH FAILED - Is %ESP_PORT% busy?
echo Close serial monitor and retry.
pause
exit /b 1
