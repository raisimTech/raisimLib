cd build

for PY_VERSION in 35 36 37 38 39
do
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_PY=ON -DRAISIM_EXAMPLE=ON -DPYTHON_EXECUTABLE:FILEPATH=/usr/local/anaconda3/envs/python${PY_VERSION}/bin/python
    make -j
done
