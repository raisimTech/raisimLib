#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimZ" for configuration "Debug"
set_property(TARGET raisim::raisimZ APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimZ PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/raisimZd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/raisimZd.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimZ )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimZ "${_IMPORT_PREFIX}/lib/raisimZd.lib" "${_IMPORT_PREFIX}/bin/raisimZd.dll" )

# Import target "raisim::raisimPng" for configuration "Debug"
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/raisimPngd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/raisimPngd.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimPng )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimPng "${_IMPORT_PREFIX}/lib/raisimPngd.lib" "${_IMPORT_PREFIX}/bin/raisimPngd.dll" )

# Import target "raisim::raisimMine" for configuration "Debug"
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/raisimMined.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/raisimMined.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimMine )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimMine "${_IMPORT_PREFIX}/lib/raisimMined.lib" "${_IMPORT_PREFIX}/bin/raisimMined.dll" )

# Import target "raisim::raisimODE" for configuration "Debug"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/raisimODEd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/raisimODEd.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/raisimODEd.lib" "${_IMPORT_PREFIX}/bin/raisimODEd.dll" )

# Import target "raisim::raisim" for configuration "Debug"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/raisimd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/raisimd.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/raisimd.lib" "${_IMPORT_PREFIX}/bin/raisimd.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
