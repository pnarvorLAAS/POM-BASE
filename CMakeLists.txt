cmake_minimum_required (VERSION 3.0)
project (pom-base VERSION 0.1)
set(LIBRARY_NAME ${PROJECT_NAME})

set(HEADERS_INSTALL_DIR include/pom-base)
set(LIB_INSTALL_DIR lib)

set(SOURCES src/PositionManagerBase.cpp)
set(HEADERS include/PositionManagerBase.hpp)

add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES})

SET(DEPENDENCIES "envire_core")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

## Generate and install a .pc file for installation (not the same as the one for compilation)
install(CODE "
    set(LIB_INSTALL_DIR ${LIB_INSTALL_DIR})
    set(HEADERS_INSTALL_DIR ${HEADERS_INSTALL_DIR})
    set(LIBRARY_NAME ${LIBRARY_NAME})
    set(PROJECT_VERSION ${PROJECT_VERSION})
    set(DEPENDENCIES ${DEPENDENCIES})
    foreach(header ${HEADERS})
        file(INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/\${header} DESTINATION ${CMAKE_INSTALL_PREFIX}/${HEADERS_INSTALL_DIR})
    endforeach()
    configure_file(${CMAKE_SOURCE_DIR}/template-install.pc.in ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)")
## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}/static
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
)

