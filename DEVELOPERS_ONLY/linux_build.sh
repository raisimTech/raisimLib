cd build

for PY_VERSION in 3.5 3.6 3.7 3.8 3.9
do
	CXX=/usr/bin/clang++-11 CC=/usr/bin/clang-11 cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DRAISIM_MATLAB=ON -DRAISIM_PY=ON -DRAISIM_EXAMPLE=ON -DRAISIM_DOC=ON -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python${PY_VERSION}
    make -j
done
