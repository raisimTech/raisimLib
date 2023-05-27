#//----------------------------//
#// This file is part of RaiSim//
#// Copyright 2020, RaiSim Tech//
#//----------------------------//

cmake_minimum_required(VERSION 3.10)
project(raisimMatlab VERSION ${RAISIM_VERSION})
add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)
add_definitions(/DKALCVF_MEX )

find_package(Matlab REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# the RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# don't add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

set(CMAKE_RUNPA)

if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/bin")
endif()

IF(Matlab_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")    
    add_library(raisim SHARED raisim_interface_mex.cpp)
    target_include_directories(raisim PUBLIC ${Matlab_INCLUDE_DIRS})

    if(WIN32)
        target_link_libraries(raisim ${Matlab_LIBRARIES} raisim::raisim Ws2_32)
    else()
        target_link_libraries(raisim ${Matlab_LIBRARIES} raisim::raisim pthread)
    endif()
    
    # 32-bit or 64-bit mex
    if(WIN32)
    if (CMAKE_CL_64)
        SET_TARGET_PROPERTIES(raisim PROPERTIES SUFFIX .mexw64)
    else(CMAKE_CL_64)
        SET_TARGET_PROPERTIES(raisim PROPERTIES SUFFIX .mexw32)
    endif(CMAKE_CL_64)
    else(WIN32)
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(raisim PROPERTIES SUFFIX .mexa64 PREFIX "")
    else(CMAKE_SIZEOF_VOID_P MATCHES "8")
        SET_TARGET_PROPERTIES(raisim PROPERTIES SUFFIX .mexglx PREFIX "")
    endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
    endif(WIN32)

    if(LINUX)
        SET_TARGET_PROPERTIES(raisim PROPERTIES SUFFIX .mexa64 PREFIX "")
    endif(LINUX)
        set_target_properties(raisim PROPERTIES LINK_FLAGS "-Wl,--no-as-needed,-rpath,${CMAKE_PREFIX_PATH}/lib")

    if (MSVC)
        set_target_properties(raisim
            PROPERTIES
            LINK_FLAGS "${_previous_link_flags} /EXPORT:mexFunction")
    endif()

    if(UNIX)
        set(INSTALL_SITE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/lib")
    else()
        set(INSTALL_SITE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/${RAISIM_OS}/bin")
    endif()

    install(
            TARGETS
            raisim
            DESTINATION
            "${INSTALL_SITE}"
    )

ELSE(Matlab_FOUND)
    MESSAGE("MATLAB not found. RaiSim Matlab will not be built.")
ENDIF(Matlab_FOUND)
