cmake_minimum_required(VERSION 3.10)
set(RAISIM_VERSION 1.1.0)
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
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS}/mt_release)
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/raisim/${RAISIM_OS}/mt_debug)   
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/Eigen3)
endif()

#==
# Package installation
#==
if(WIN32)
    set(RAISIM_BIN_DIR ${RAISIM_OS}/mt_release)
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

install(
        DIRECTORY
        "${CMAKE_CURRENT_LIST_DIR}/raisim/${RAISIM_BIN_DIR}/lib"
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}
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
