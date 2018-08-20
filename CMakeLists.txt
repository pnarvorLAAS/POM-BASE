cmake_minimum_required(VERSION 3.0)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(infuse_pom_base VERSION 0.1)

set(CMAKE_CXX_STANDARD 11)

###
# Imports infuse_pom_base dependencies

find_package(PkgConfig REQUIRED)

find_package(urdfdom REQUIRED)
pkg_check_modules(ENVIRE REQUIRED infuse_envire)

include_directories(
    include/
    ${ENVIRE_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
)
link_directories(${ENVIRE_LIBRARY_DIRS})

####
# Generate target
set(SOURCES 
    src/PositionManagerBase.cpp
    src/UrdfParser.cpp
    )

add_library(${PROJECT_NAME} SHARED ${SOURCES})


####
# Set properties of target
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

set(HEADERS
    include/PositionManagerBase.hpp
    include/UrdfParser.hpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADERS}")


#####
# Generation of configuration files


# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)


# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

# This is exclusive for pkg-config
set(pkgconfig_install_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")


set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")


# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(pkgconfig_file "${generated_dir}/${PROJECT_NAME}.pc")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
#
#
# Include module with fuctions:
#   * 'write_basic_package_version_file'
#   * 'configure_package_config_file'
include(CMakePackageConfigHelpers)
#
#
# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Uses:
#   * PROJECT_VERSION
write_basic_package_version_file(
  "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Uses variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
#   * CMAKE_INSTALL_INCLUDEDIR
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    PATH_VARS
      CMAKE_INSTALL_INCLUDEDIR
)

# Configure '<PROJECT-NAME>.pc'
# Uses variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
#   * CMAKE_INSTALL_PREFIX
#   * CMAKE_INSTALL_LIBDIR
#   * CMAKE_INSTALL_INCLUDEDIR
configure_file(
    "cmake/pkgconfig.pc.cmakein"
    "${pkgconfig_file}" @ONLY
)


####
# Installation


# Targets:
#   * <prefix>/lib/lib${PROJECT_NAME}.so
#   * header location after install: <prefix>/include/${PROJECT_NAME}/*.h
#   * headers can be included by C++ code `#include <${PROJECT_NAME}/*.h>`
install(
    TARGETS ${PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)


# Config
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)


# Config
#   * <prefix>/lib/pkgconfig/${PROJECT_NAME}.pc
install(
    FILES "${pkgconfig_file}"
    DESTINATION "${pkgconfig_install_dir}")


# Config
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${config_install_dir}"
)
