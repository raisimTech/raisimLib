Set-Location C:\Users\jemin\source\repos\raisim\build-release
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=ON -DCMAKE_PREFIX_PATH=C:\Users\jemin\devel -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32
cmake --build . --config Release
cmake --build . --target install --config Release

Set-Location C:\Users\jemin\source\repos\raisim\build-debug
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32
cmake --build . --config Debug
cmake --build . --target install --config Debug

conda activate python37
Set-Location C:\Users\jemin\source\repos\raisimLib
rm build -r
mkdir build
cd build
cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DRAISIM_MATLAB=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisim_install
cmake --build . --target install --config Release

conda activate python38
Set-Location C:\Users\jemin\source\repos\raisimLib
rm build -r
mkdir build
cd build
cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DRAISIM_MATLAB=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisim_install
cmake --build . --target install --config Release

conda activate python39
Set-Location C:\Users\jemin\source\repos\raisimLib
rm build -r
mkdir build
cd build
cmake .. -DRAISIM_EXAMPLE=OFF -DRAISIM_PY=ON -DRAISIM_MATLAB=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisim_install
cmake --build . --target install --config Release

conda activate python310
Set-Location C:\Users\jemin\source\repos\raisimLib
rm build -r
mkdir build
cd build
cmake .. -DRAISIM_EXAMPLE=OFF -DRAISIM_PY=ON -DRAISIM_MATLAB=ON -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisim_install
cmake --build . --target install --config Release


cd C:\Users\jemin
