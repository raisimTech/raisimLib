#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimZ" for configuration "RELEASE"
set_property(TARGET raisim::raisimZ APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimZ PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimZ.so"
  IMPORTED_SONAME_RELEASE "libraisimZ.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimZ )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimZ "${_IMPORT_PREFIX}/lib/libraisimZ.so" )

# Import target "raisim::raisimPng" for configuration "RELEASE"
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimPng.so"
  IMPORTED_SONAME_RELEASE "libraisimPng.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimPng )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimPng "${_IMPORT_PREFIX}/lib/libraisimPng.so" )

# Import target "raisim::raisimMine" for configuration "RELEASE"
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimMine.so"
  IMPORTED_SONAME_RELEASE "libraisimMine.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimMine )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimMine "${_IMPORT_PREFIX}/lib/libraisimMine.so" )

# Import target "raisim::raisimODE" for configuration "RELEASE"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimODE.so.1.1.2"
  IMPORTED_SONAME_RELEASE "libraisimODE.so.1.1.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODE.so.1.1.2" )

# Import target "raisim::raisim" for configuration "RELEASE"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisim.so.1.1.2"
  IMPORTED_SONAME_RELEASE "libraisim.so.1.1.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisim.so.1.1.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
