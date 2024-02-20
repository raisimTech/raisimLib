cd ~/workspace/raisim/
rm -rf build build-arm build-debug
mkdir build
mkdir build-arm
mkdir build-debug
cd build
CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux -DRAISIM_TEST=OFF
make install -j

cd ../build-debug
CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux -DRAISIM_TEST=OFF
make install -j

# download the tool at https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/AT/JetsonLinuxToolchain.html#at-jetsonlinuxtoolchain
cd ../build-arm
CXX=~/workspace/aarch64--glibc--stable-final/bin/aarch64-buildroot-linux-gnu-g++ CC=~/workspace/aarch64--glibc--stable-final/bin/aarch64-buildroot-linux-gnu-gcc cmake .. -DRAISIM_ARM=ON -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux-arm -DRAISIM_TEST=OFF
make install -j

cd ~/workspace/raisimLib
rm -rf build
mkdir build
cd build

for PY_VERSION in 37 38 39 310 311
do
  CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_MATLAB=OFF -DRAISIM_PY=ON -DRAISIM_EXAMPLE=OFF -DCMAKE_INSTALL_PREFIX=~/workspace/raisim_install -DRAISIM_DOC=OFF -DPYTHON_EXECUTABLE=~/anaconda3/envs/py${PY_VERSION}/bin/python
  make install -j
done
