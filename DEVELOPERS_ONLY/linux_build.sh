cd ~/workspace/raisim/
rm -rf build build-arm build-debug
mkdir build
mkdir build-arm
mkdir build-debug
cd build
CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux
make install -j

cd ../build-debug
CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux
make install -j

cd ../build-arm
CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_ARM=ON -DCMAKE_INSTALL_PREFIX=~/workspace/raisimLib/raisim/linux-arm
make install -j

cd ~/workspace/raisimLib/build

for PY_VERSION in 37 38 39 310 311
do
  CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_MATLAB=OFF -DRAISIM_PY=ON -DRAISIM_EXAMPLE=OFF -DCMAKE_INSTALL_PREFIX=~/workspace/raisim_install -DRAISIM_DOC=OFF -DPYTHON_EXECUTABLE=/home/jemin/anaconda3/envs/py${PY_VERSION}/bin/python
  make install -j
done

CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_EXAMPLE=ON -DRAISIM_PY=OFF -DRAISIM_MATLAB=ON -DPYTHON_EXECUTABLE:FILEPATH=/home/jemin/anaconda3/envs/raisim/bin/python
make -j

make -j

