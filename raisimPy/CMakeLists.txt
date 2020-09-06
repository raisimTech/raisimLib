# Run: cmake -DPYBIND11_PYTHON_VERSION=<PYTHON_VERSION> -DCMAKE_PREFIX_PATH=$LOCAL_BUILD ..
# where <PYTHON_VERSION>=2.7 or 3.*, LOCAL_BUILD is the build for raisimLib and raisimOgre
project (raisim_wrapper VERSION ${RAISIM_VERSION})
set(CMAKE_CXX_STANDARD 14)

# find the various packages
find_package(Eigen3 REQUIRED)
find_package(raisim CONFIG REQUIRED)

# source files
file(GLOB SOURCES "src/*.cpp")

pybind11_add_module(raisimpy ${SOURCES})
target_link_libraries(raisimpy PRIVATE raisim::raisim)

# header files
target_include_directories(raisimpy PUBLIC include ${EIGEN3_INCLUDE_DIRS})

if(UNIX)
    set_target_properties(raisimpy
            PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            )
else()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../win32/mt_release/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../win32/mt_debug/bin")
    target_link_libraries(raisimpy PRIVATE Ws2_32)
endif()
# Don't forget to export PYTHONPATH=$PYTHONPATH:$LOCAL_BUILD/lib
