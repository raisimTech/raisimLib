Set-Location C:\Users\jemin\source\repos\raisim\build-release
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=ON -DCMAKE_PREFIX_PATH=C:\Users\jemin\devel -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32\mt_release
cmake --build . --config Release
cmake --build . --target install --config Release

Set-Location C:\Users\jemin\source\repos\raisim\build-debug
cmake -G"Visual Studio 16 2019" -A x64 .. -DRAISIM_TEST=OFF -DCMAKE_INSTALL_PREFIX=C:\Users\jemin\source\repos\raisimLib\raisim\win32\mt_debug
cmake --build . --config Debug
cmake --build . --target install --config Debug

Set-Location C:\Users\jemin\source\repos\raisimLib\build
cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DRAISIM_MATLAB=ON -DPYTHON_EXECUTABLE=C:\Users\jemin\anaconda3\envs\python35\python
cmake --build . --config Release

cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=C:\Users\jemin\anaconda3\envs\python36\python
cmake --build . --config Release

cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=C:\Users\jemin\anaconda3\envs\python37\python
cmake --build . --config Release

cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=C:\Users\jemin\anaconda3\envs\python38\python
cmake --build . --config Release

cmake .. -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=C:\Users\jemin\anaconda3\envs\python39\python
cmake --build . --config Release

cd C:\Users\jemin
