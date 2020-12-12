
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was raisimConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(FindPackageHandleStandardArgs)

# Unset found flag to ensure correct package configuration
unset(raisim_FOUND)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

if(NOT TARGET raisim::raisim)
  include(${CMAKE_CURRENT_LIST_DIR}/raisim-targets.cmake)
endif()

get_target_property(raisim_INCLUDE_DIRS raisim::raisim INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(raisim_LIBRARIES raisim::raisim INTERFACE_LINK_LIBRARIES)
list(APPEND raisim_LIBRARIES raisim::raisim)

#==
# Pacakge configuration check
#==

find_package_handle_standard_args(raisim
  REQUIRED_VARS
    raisim_INCLUDE_DIRS
    raisim_LIBRARIES
)

# compiling options
if (CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    add_definitions(-DRSDEBUG)
else()
    remove_definitions(-DRSDEBUG)
endif ()

if(raisim_FOUND)
  message(STATUS "raisim:")
  message(STATUS "  Version: ${raisim_VERSION}")
  message(STATUS "  Includes: ${raisim_INCLUDE_DIRS}")
  message(STATUS "  Libraries: ${raisim_LIBRARIES}")
endif()
