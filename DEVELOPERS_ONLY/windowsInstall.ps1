Set-Location C:\Users\jemin\source\repos\raisim\build-release
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=ON -DCMAKE_PREFIX_PATH=C:\Users\jemin\devel -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32
cmake --build . --config Release
cmake --build . --target install --config Release

Set-Location C:\Users\jemin\source\repos\raisim\build-debug
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32
cmake --build . --config Debug
cmake --build . --target install --config Debug

conda activate python38
Set-Location C:\Users\jemin\source\repos\raisimLib\build
cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DRAISIM_MATLAB=OFF -DPYTHON_EXECUTABLE:FILEPATH=C:\Users\jemin\anaconda3\envs\python38\python
cmake --build . --config Release

conda activate python39
Set-Location C:\Users\jemin\source\repos\raisimLib\build
cmake .. -DRAISIM_EXAMPLE=OFF -DRAISIM_PY=ON -DRAISIM_MATLAB=ON -DPYTHON_EXECUTABLE:FILEPATH=C:\Users\jemin\anaconda3\envs\python39\python
cmake --build . --config Release



cd C:\Users\jemin
