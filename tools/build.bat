@echo off
pushd %~dp0\..\build

echo Cleaning old cache to prevent path errors...
if exist DX12\CMakeCache.txt del DX12\CMakeCache.txt
if exist VK\CMakeCache.txt del VK\CMakeCache.txt

echo Generating Solutions...
call GenerateSolutions.bat

echo Compiling FSR Clarity (DX12)...
cd DX12
cmake --build . --config Release

popd
