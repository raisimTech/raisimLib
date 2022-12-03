cmake_minimum_required(VERSION 3.11)
set(RAISIM_VERSION 1.1.6)
project(raisim VERSION ${RAISIM_VERSION} LANGUAGES CXX)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Include CMake helper modules
include(GNUInstallDirs)

# Build options
option(RAISIM_EXAMPLE "Build example applications." TRUE)
option(RAISIM_MATLAB "Build a matlab wrapper." FALSE)
option(RAISIM_PY "Build a python wrapper." FALSE)
option(RAISIM_DOC "Build a raisim documentation. You can find the latest doc in raisim.com." FALSE)

if (UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL AND NOT WIN32)
    set(RAISIM_OS linux)  
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS})
elseif(APPLE)
    execute_process(COMMAND sysctl -q hw.optional.arm64
            OUTPUT_VARIABLE _sysctl_stdout
            ERROR_VARIABLE _sysctl_stderr
            RESULT_VARIABLE _sysctl_result
            )
    if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
        set(RAISIM_OS m1)
    else()
        set(RAISIM_OS mac)
    endif()
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS})
elseif(WIN32)
    set(RAISIM_OS win32)
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS})
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/Eigen3)
endif()

#==
# Package installation
#==
if(WIN32)
    set(RAISIM_BIN_DIR ${RAISIM_OS})
    install(
        DIRECTORY
        ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/bin
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
else()
    set(RAISIM_BIN_DIR ${RAISIM_OS})
endif()


install(
        DIRECTORY
        ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/include
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}
)

# Distinguish installation between shared libraries and Python modules
#  - NB: raisimpy may be rebuilt for the current environment Python if RAISIM_PY is set to be true. This does not affect the installation of precompiled libraries.
#  - GLOB_RECURSE is available from CMake version 3.11
file(GLOB_RECURSE RAISIM_LIBS "${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/*")
set(RAISIM_PY_LIBS ${RAISIM_LIBS})
list(FILTER RAISIM_PY_LIBS INCLUDE REGEX "^${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/raisimpy")
list(FILTER RAISIM_LIBS EXCLUDE REGEX "^${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/raisimpy")
list(FILTER RAISIM_LIBS EXCLUDE REGEX "^${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/cmake")

# Install regular libraries
install(
        FILES
        ${RAISIM_LIBS}
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib
)

# Install cmake files
install(
        DIRECTORY
        ${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib/cmake
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/lib
)

# Install precompiled Python modules (raisimpy)
find_package(PythonInterp REQUIRED)
execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
    from distutils import sysconfig as sc
    print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
install(
        FILES
        ${RAISIM_PY_LIBS}
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}
)


install(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}")

if(RAISIM_ALL)
    set(RAISIM_EXAMPLE TRUE)
    set(RAISIM_MATLAB TRUE)
    set(RAISIM_PY TRUE)
    set(RAISIM_DOC TRUE)
endif()

if(RAISIM_EXAMPLE)
    add_subdirectory(examples)
endif()

if(RAISIM_MATLAB)
    add_subdirectory(raisimMatlab)
endif()

if(RAISIM_PY)
    add_subdirectory(thirdParty/pybind11)
    add_subdirectory(raisimPy)
endif()

if(RAISIM_DOC)
    add_subdirectory(docs)
endif()


# EOF
