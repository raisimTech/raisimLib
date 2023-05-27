cmake_minimum_required(VERSION 3.10)
project (raisim_wrapper VERSION ${RAISIM_VERSION})
set(CMAKE_CXX_STANDARD 14)

# find the various packages
find_package(Eigen3 REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(PythonInterp REQUIRED)

# source files
file(GLOB SOURCES "src/*.cpp")

pybind11_add_module(raisimpy ${SOURCES})
target_link_libraries(raisimpy PRIVATE raisim::raisim)

# header files
target_include_directories(raisimpy PUBLIC include ${EIGEN3_INCLUDE_DIRS})

# Install precompiled Python modules (raisimpy)
execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
    from distutils import sysconfig as sc
    print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

if(UNIX)
    install(TARGETS raisimpy DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE})
    set(INSTALL_SITE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib")
else()
    target_link_libraries(raisimpy PRIVATE Ws2_32)
    set(INSTALL_SITE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/bin")
endif()

install(
        TARGETS
        raisimpy
        DESTINATION
        "${INSTALL_SITE}"
)