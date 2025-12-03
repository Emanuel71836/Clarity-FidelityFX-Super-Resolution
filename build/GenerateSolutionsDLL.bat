@echo off
setlocal enabledelayedexpansion

echo Checking pre-requisites... 

:: Check if CMake is installed
cmake --version > nul 2>&1
if %errorlevel% NEQ 0 (
    echo Cannot find path to cmake. Is CMake installed? Exiting...
    exit /b -1
) else (
    echo    CMake      - Ready.
) 

:: Cauldron check skipped for FSR 4.0 standalone build
:: if not exist ..\libs\cauldron\common.cmake (
::    ...
:: )

:: Check if VULKAN_SDK is installed but don't bail out
if "%VULKAN_SDK%"=="" (
    echo Vulkan SDK is not installed -Environment variable VULKAN_SDK is not defined- : Please install the latest Vulkan SDK from LunarG.
) else (
    echo    Vulkan SDK - Ready : %VULKAN_SDK%
)

:: Call CMake
mkdir DX12
cd DX12
cmake -A x64 ..\.. -DGFX_API=DX12 -DFSR2_BUILD_AS_DLL=1
cd ..

mkdir VK
cd VK
cmake -A x64 ..\.. -DGFX_API=VK -DFSR2_BUILD_AS_DLL=1
cd ..
