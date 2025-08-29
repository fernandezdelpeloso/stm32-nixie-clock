cmake_minimum_required( VERSION 3.10 FATAL_ERROR )

project( kicad-packages3d NONE )


#================================================
# Locations for install targets.
#================================================
if( APPLE )
    # Like all variables, CMAKE_INSTALL_PREFIX can be over-ridden on the command line.
    set( CMAKE_INSTALL_PREFIX "/Library/Application Support/kicad/" CACHE PATH "" )
    # Everything without leading / is relative to CMAKE_INSTALL_PREFIX.
    set( KICAD_PACKAGES_3D 3dmodels )
else()
    # Everything without leading / is relative to CMAKE_INSTALL_PREFIX.
    set( KICAD_DATA share/kicad
        CACHE PATH "Location of KiCad data files." )
    set( KICAD_PACKAGES_3D ${KICAD_DATA}/3dmodels )
endif()

mark_as_advanced( KICAD_DATA KICAD_MODULES )


#================================================
# "make uninstall" rules
#
# The uninstaller does not remove folders only files.
#================================================
configure_file(
    "${PROJECT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY
)

add_custom_target( uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
)


#================================================================
# List of all models in the repository.
#
# This can be used by packagers to strip the models from footprints
# that do not have released models.
#================================================================

set( MODEL_NAMES_FILE ${CMAKE_CURRENT_BINARY_DIR}/model_names.txt )
set( METRICS_FILE ${CMAKE_CURRENT_BINARY_DIR}/metrics.txt )

# Command to generate the list of model names.
add_custom_command(
   OUTPUT ${MODEL_NAMES_FILE} ${MODEL_NAMES_FILE}.always_runs
   COMMENT "Listing all models in the repository"
   COMMAND find . -name "*.step" -print0 |
          xargs -0 realpath --relative-to=. |
          sed "s,\.step\$,," > ${MODEL_NAMES_FILE}
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   VERBATIM
)

# Generate some metrics about the models
add_custom_command(
    OUTPUT ${METRICS_FILE}
    DEPENDS ${MODEL_NAMES_FILE}
    COMMAND ${CMAKE_COMMAND} -E echo_append "TotalModelCount: " > ${METRICS_FILE}
    COMMAND wc -l < ${MODEL_NAMES_FILE} >> ${METRICS_FILE}
    COMMAND ${CMAKE_COMMAND} -E echo_append "LibraryCount: " >> ${METRICS_FILE}
    COMMAND find . -maxdepth 1 -type d -name "'*.3dshapes'" | wc -l >> ${METRICS_FILE}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

add_custom_target( list_models DEPENDS ${MODEL_NAMES_FILE} ${METRICS_FILE} )


#================================================
# Installed files.
#================================================

# Install all of the .3dshapes folders and their contents in to the modules path.
file ( GLOB SHAPE_DIRS "*.3dshapes" )

install( DIRECTORY ${SHAPE_DIRS}
    DESTINATION ${KICAD_PACKAGES_3D}
    COMPONENT resources
    FILES_MATCHING PATTERN "*.wrl"
    PATTERN ".git" EXCLUDE
)

install( DIRECTORY ${SHAPE_DIRS}
    DESTINATION ${KICAD_PACKAGES_3D}
    COMPONENT resources
    FILES_MATCHING PATTERN "*.step"
    PATTERN ".git" EXCLUDE
)
