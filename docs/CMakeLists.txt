find_package(Doxygen REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(Sphinx REQUIRED)

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/docs/sphinx)

set(DOXYGEN_INPUT_DIR "${PROJECT_SOURCE_DIR}/raisim/linux/include/raisim")

list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/constraints/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/contact/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/math/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/object/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/object/ArticulatedSystem/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/object/singleBodies/*.hpp)
list(APPEND RAISIM_PUBLIC_HEADERS ${DOXYGEN_INPUT_DIR}/object/terrain/*.hpp)

set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/docs/doxygen)
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

#Replace variables inside @@ with the current values
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_target(doxygen_output ALL
        DEPENDS ${RAISIM_PUBLIC_HEADERS}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
        COMMENT "Generating docs")

add_custom_target(Sphinx ALL
        DEPENDS ${RAISIM_PUBLIC_HEADERS} doxygen_output
        COMMAND ${SPHINX_EXECUTABLE} -b html -E -a
        # Tell Breathe where to find the Doxygen output
        -Dbreathe_projects.raisim=${DOXYGEN_OUTPUT_DIR}/xml
        ${SPHINX_SOURCE} ${SPHINX_BUILD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating documentation with Sphinx")

add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})