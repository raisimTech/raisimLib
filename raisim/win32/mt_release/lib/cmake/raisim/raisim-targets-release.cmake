#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimZ" for configuration "Release"
set_property(TARGET raisim::raisimZ APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimZ PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/raisimZ.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/raisimZ.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimZ )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimZ "${_IMPORT_PREFIX}/lib/raisimZ.lib" "${_IMPORT_PREFIX}/bin/raisimZ.dll" )

# Import target "raisim::raisimPng" for configuration "Release"
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/raisimPng.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/raisimPng.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimPng )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimPng "${_IMPORT_PREFIX}/lib/raisimPng.lib" "${_IMPORT_PREFIX}/bin/raisimPng.dll" )

# Import target "raisim::raisimMine" for configuration "Release"
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/raisimMine.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/raisimMine.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimMine )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimMine "${_IMPORT_PREFIX}/lib/raisimMine.lib" "${_IMPORT_PREFIX}/bin/raisimMine.dll" )

# Import target "raisim::raisimODE" for configuration "Release"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/raisimODE.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/raisimODE.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisimODE )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisimODE "${_IMPORT_PREFIX}/lib/raisimODE.lib" "${_IMPORT_PREFIX}/bin/raisimODE.dll" )

# Import target "raisim::raisim" for configuration "Release"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/raisim.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/raisim.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS raisim::raisim )
list(APPEND _IMPORT_CHECK_FILES_FOR_raisim::raisim "${_IMPORT_PREFIX}/lib/raisim.lib" "${_IMPORT_PREFIX}/bin/raisim.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
