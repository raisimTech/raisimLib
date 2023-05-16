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

if(UNIX)
    set_target_properties(raisimpy
            PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib"
            )
else()
    target_link_libraries(raisimpy PRIVATE Ws2_32)
    set_target_properties(raisimpy
            PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/bin"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/bin"
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/bin"
            )
endif()

# Install precompiled Python modules (raisimpy)
execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_paths()[\"purelib\"])"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- Installing raisimpy in ${PYTHON_SITE}")
install(
        TARGETS
        raisimpy
        DESTINATION
        ${PYTHON_SITE}
)