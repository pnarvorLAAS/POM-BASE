cmake_minimum_required (VERSION 3.0)
project (pom-base VERSION 0.1)
set(LIBRARY_NAME ${PROJECT_NAME})

set(SOURCES 
    src/PositionManagerBase.cpp
    src/UrdfParser.cpp
    )

add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES})

SET(DEPENDENCIES "envire_core")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

# Installation module
set(HEADERS_INSTALL_DIR include/pom-base)
set(LIB_INSTALL_DIR lib)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

