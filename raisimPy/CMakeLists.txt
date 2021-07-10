cmake_minimum_required(VERSION 3.10)
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

execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
    from distutils import sysconfig as sc
    print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

if(UNIX)
    set_target_properties(raisimpy
            PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            )
else()
    set_target_properties(raisimpy
            PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/mt_release/bin"
            LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/mt_release/bin"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/mt_release/bin"
            )
    target_link_libraries(raisimpy PRIVATE Ws2_32)
endif()
