cmake_minimum_required(VERSION 3.5)
project(raisim_examples LANGUAGES CXX)

#==
# Dependencies
#==

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
include_directories ( ${EIGEN3_INCLUDE_DIRS} )

if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_release/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_debug/bin")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
else()
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/examples)
endif()


#==
# Build Targets
#==

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})
    if(WIN32)
        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32)
    else()
        target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)
    endif()
    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

# benchmark examples
create_executable(anymal_speed_test src/benchmark/anymal.cpp)

# server examples
create_executable(primitives src/server/primitives.cpp)
create_executable(meshes src/server/meshes.cpp)
create_executable(laikago src/server/laikago.cpp)
create_executable(balls src/server/balls.cpp)
create_executable(atlas src/server/atlas.cpp)
create_executable(heightmap src/server/heightmap.cpp)
create_executable(hibernate src/server/hibernate.cpp)
create_executable(robots src/server/robots.cpp)
create_executable(materials src/server/material.cpp)
create_executable(heightMapUsingPng src/server/heightMapUsingPng.cpp)

# xml reader
create_executable(xmlRader src/xml/xmlReader.cpp)