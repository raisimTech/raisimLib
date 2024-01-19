for PY_VERSION in 37 38 39 310 311
do
  CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_MATLAB=OFF -DRAISIM_PY=ON -DRAISIM_EXAMPLE=OFF -DCMAKE_INSTALL_PREFIX=~/workspace/raisim_install -DRAISIM_DOC=OFF -DPYTHON_EXECUTABLE=/home/jemin/anaconda3/envs/py${PY_VERSION}/bin/python
  make install -j
done

CXX=/usr/bin/clang++-15 CC=/usr/bin/clang-15 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_EXAMPLE=ON -DRAISIM_PY=OFF -DRAISIM_MATLAB=ON -DPYTHON_EXECUTABLE:FILEPATH=/home/jemin/anaconda3/envs/raisim/bin/python
make -j

make -j

