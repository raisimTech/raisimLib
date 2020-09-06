#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimZ" for configuration ""
set_property(TARGET raisim::raisimZ APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(raisim::raisimZ PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libraisimZ.so"
  IMPORTED_SONAME_NOCONFIG "libraisimZ.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimZ )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimZ "${_IMPORT_PREFIX}/lib/libraisimZ.so" )

# Import target "raisim::raisimPng" for configuration ""
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libraisimPng.so"
  IMPORTED_SONAME_NOCONFIG "libraisimPng.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimPng )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimPng "${_IMPORT_PREFIX}/lib/libraisimPng.so" )

# Import target "raisim::raisimMine" for configuration ""
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libraisimMine.so"
  IMPORTED_SONAME_NOCONFIG "libraisimMine.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimMine )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimMine "${_IMPORT_PREFIX}/lib/libraisimMine.so" )

# Import target "raisim::raisimODE" for configuration ""
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libraisimODE.so"
  IMPORTED_SONAME_NOCONFIG "libraisimODE.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODE.so" )

# Import target "raisim::raisim" for configuration ""
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libraisim.so"
  IMPORTED_SONAME_NOCONFIG "libraisim.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisim.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
