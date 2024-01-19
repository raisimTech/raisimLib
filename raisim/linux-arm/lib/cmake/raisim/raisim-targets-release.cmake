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

list(APPEND _cmake_import_check_targets raisim::raisimZ )
list(APPEND _cmake_import_check_files_for_raisim::raisimZ "${_IMPORT_PREFIX}/lib/libraisimZ.so" )

# Import target "raisim::raisimPng" for configuration "RELEASE"
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimPng.so"
  IMPORTED_SONAME_RELEASE "libraisimPng.so"
  )

list(APPEND _cmake_import_check_targets raisim::raisimPng )
list(APPEND _cmake_import_check_files_for_raisim::raisimPng "${_IMPORT_PREFIX}/lib/libraisimPng.so" )

# Import target "raisim::raisimMine" for configuration "RELEASE"
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimMine.so"
  IMPORTED_SONAME_RELEASE "libraisimMine.so"
  )

list(APPEND _cmake_import_check_targets raisim::raisimMine )
list(APPEND _cmake_import_check_files_for_raisim::raisimMine "${_IMPORT_PREFIX}/lib/libraisimMine.so" )

# Import target "raisim::raisimODE" for configuration "RELEASE"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisimODE.so.1.1.7"
  IMPORTED_SONAME_RELEASE "libraisimODE.so.1.1.7"
  )

list(APPEND _cmake_import_check_targets raisim::raisimODE )
list(APPEND _cmake_import_check_files_for_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODE.so.1.1.7" )

# Import target "raisim::raisim" for configuration "RELEASE"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libraisim.so.1.1.7"
  IMPORTED_SONAME_RELEASE "libraisim.so.1.1.7"
  )

list(APPEND _cmake_import_check_targets raisim::raisim )
list(APPEND _cmake_import_check_files_for_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisim.so.1.1.7" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
