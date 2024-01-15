cd build

for PY_VERSION in 37 38 39 310 311
do
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_PY=ON -DRAISIM_EXAMPLE=ON -DPYTHON_EXECUTABLE=/opt/homebrew/anaconda3/envs/py${PY_VERSION}/bin/python
    make -j6
done
