@echo off
setlocal

set PROJECT_ROOT=%CD%
set BUILD_ROOT=build
set VISUAL_STUDIO_GENERATOR="Visual Studio 16 2019"
set TARGET_ARCH=x64

taskkill /F /IM FSR2_Sample_DX12d.exe 2>nul & taskkill /F /IM FSR_Clarity_VKd.exe 2>nul & cmake --build build\DX12 --config Debug --target ALL_BUILD & cmake --build build\VK --config Debug --target ALL_BUILD

:: --- BUILD DX12 ---
echo.
echo =======================================================
echo == CONFIGURANDO E COMPILANDO DX12                    ==
echo =======================================================
mkdir %BUILD_ROOT%\DX12 2>nul
cd %BUILD_ROOT%\DX12

echo.
echo -- Configurando CMake para DX12...
cmake "%PROJECT_ROOT%" -G %VISUAL_STUDIO_GENERATOR% -A %TARGET_ARCH% -DGFX_API=DX12
if errorlevel 1 goto :error

echo.
echo -- Compilando DX12 (Debug)...
cmake --build . --config Debug --target ALL_BUILD
if errorlevel 1 goto :error

cd ..\..

:: --- BUILD VULKAN ---
echo.
echo =======================================================
echo == CONFIGURANDO E COMPILANDO VULKAN                  ==
echo =======================================================
mkdir %BUILD_ROOT%\VK 2>nul
cd %BUILD_ROOT%\VK

echo.
echo -- Configurando CMake para Vulkan...
cmake "%PROJECT_ROOT%" -G %VISUAL_STUDIO_GENERATOR% -A %TARGET_ARCH% -DGFX_API=VK
if errorlevel 1 goto :error

echo.
echo.
echo Qual sample executar?
echo.
echo 1 = DX12 Sample (como Admin)
echo 2 = Vulkan Sample (como Admin)
echo 3 = Ambos (como Admin)
echo 4 = Nenhum (apenas compilar)
echo.
set /p choice="Escolha (1-4): "

if "%choice%"=="1" goto :run_dx12
if "%choice%"=="2" goto :run_vulkan
if "%choice%"=="3" goto :run_both
if "%choice%"=="4" goto :eof
echo Opcao invalida!
goto :eof

:run_dx12
echo.
echo Executando DX12 como Administrador...
powershell -Command "Start-Process -FilePath '%CD%\build\DX12\bin\Debug\FSR2_Sample_DX12d.exe' -WorkingDirectory '%CD%\build\DX12\bin\Debug' -Verb RunAs"
goto :eof

:run_vulkan
echo.
echo Executando Vulkan como Administrador...
powershell -Command "Start-Process -FilePath '%CD%\build\VK\bin\Debug\FSR_Clarity_VKd.exe' -WorkingDirectory '%CD%\build\VK\bin\Debug' -Verb RunAs"
goto :eof

:run_both
echo.
echo Executando DX12 como Administrador...
powershell -Command "Start-Process -FilePath '%CD%\build\DX12\bin\Debug\FSR2_Sample_DX12d.exe' -WorkingDirectory '%CD%\build\DX12\bin\Debug' -Verb RunAs"
timeout /t 2 >nul
echo Executando Vulkan como Administrador...
powershell -Command "Start-Process -FilePath '%CD%\build\VK\bin\Debug\FSR_Clarity_VKd.exe' -WorkingDirectory '%CD%\build\VK\bin\Debug' -Verb RunAs"
goto :eof

:error
echo.
echo ERRO: Ocorreu um erro durante o processo de build.
pause
exit /b 1