cmake_minimum_required(VERSION 3.5)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(infuse_pom_base VERSION 0.1.2)

set(CMAKE_CXX_STANDARD 11)
set(PKG_DESCRIPTION "Base classe for Infuse Position Manager (PoM)")

option (USE_ENVIRE_MINIMAL "Compile with laas minimal envire version or the official one" FALSE)

configure_file(config.h.in include/config.h)

if(USE_ENVIRE_MINIMAL)


    ###
    # Imports infuse_pom_base dependencies
    
    find_package(urdfdom REQUIRED)
    find_package(infuse_envire REQUIRED)
    
    set(LIBTOCHECK infuse_envire)
    get_property(${LIBTOCHECK}_link TARGET ${LIBTOCHECK} PROPERTY INTERFACE_LINK_LIBRARY_RELEASE)
    get_property(${LIBTOCHECK}_include TARGET ${LIBTOCHECK} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    get_property(${LIBTOCHECK}_location TARGET ${LIBTOCHECK} PROPERTY IMPORTED_LOCATION_RELEASE)
    message(STATUS "${LIBTOCHECK}_link : ${${LIBTOCHECK}_link}")
    message(STATUS "${LIBTOCHECK}_include : ${${LIBTOCHECK}_include}")
    message(STATUS "${LIBTOCHECK}_location : ${${LIBTOCHECK}_location}")
    
    set(REQUIRED_COMPONENTS "urdfdom, infuse_envire")
    
    ####
    # Generate target
    set(SOURCES 
        src/PositionManagerBase.cpp
        src/UrdfParser.cpp
        )
    
    add_library(${PROJECT_NAME} SHARED ${SOURCES})
    target_link_libraries(${PROJECT_NAME} 
        infuse_envire
        ${urdfdom_LIBRARIES}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${urdfdom_INCLUDE_DIRS}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/>
    )

    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_ENVIRE_MINIMAL)

    set(USE_ENVIRE_MINIMAL_DEFINITION "-DUSE_ENVIRE_MINIMAL")
    set(ENVIRE_TO_USE "infuse_envire")

else()

    ###
    # Imports infuse_pom_base dependencies
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake") 
    find_package(urdfdom REQUIRED)
    find_package(envire_core REQUIRED)

    find_package(Eigen3 REQUIRED)
    add_definitions(${EIGEN3_DEFINITIONS})

    message(STATUS "Envire include dirs : ${ENVIRE_CORE_INCLUDE_DIRS}")
    message(STATUS "Envire library dirs : ${ENVIRE_CORE_LIBRARY_DIRS}")
    message(STATUS "Envire libraries    : ${ENVIRE_CORE_LIBRARIES}")
    
    # set(LIBTOCHECK ENVIRE_CORE)
    # get_property(${LIBTOCHECK}_link TARGET ${LIBTOCHECK} PROPERTY INTERFACE_LINK_LIBRARY_RELEASE)
    # get_property(${LIBTOCHECK}_include TARGET ${LIBTOCHECK} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    # get_property(${LIBTOCHECK}_location TARGET ${LIBTOCHECK} PROPERTY IMPORTED_LOCATION_RELEASE)
    # message(STATUS "${LIBTOCHECK}_link : ${${LIBTOCHECK}_link}")
    # message(STATUS "${LIBTOCHECK}_include : ${${LIBTOCHECK}_include}")
    # message(STATUS "${LIBTOCHECK}_location : ${${LIBTOCHECK}_location}")
    
    set(REQUIRED_COMPONENTS "urdfdom, envire_core, eigen3")
    
    ####
    # Generate target
    set(SOURCES 
        src/PositionManagerBase.cpp
        src/UrdfParser.cpp
        )
    
    add_library(${PROJECT_NAME} SHARED ${SOURCES})
    target_link_libraries(${PROJECT_NAME}
        ${envire_core_LIBRARIES}
        ${urdfdom_LIBRARIES}
        ${EIGEN3_LIBRARIES}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${envire_core_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include/>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/>
    )
    set(ENVIRE_TO_USE "envire_core")

endif()

####
# Set properties of target
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

set(HEADERS
    include/infuse_pom_base/PositionManagerBase.hpp
    include/infuse_pom_base/UrdfParser.hpp
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

# Config header
install(
    FILES "${CMAKE_BINARY_DIR}/include/config.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
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

