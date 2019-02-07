find_package(PkgConfig)

# find envire_core
pkg_check_modules(PC_ENVIRE_CORE REQUIRED envire_core)
set(ENVIRE_CORE_DEFINITIONS ${PC_ENVIRE_CORE_CFLAGS_OTHER})
find_path(ENVIRE_CORE_INCLUDE_DIR envire_core
    HINTS ${PC_ENVIRE_CORE_INCLUDEDIR} ${PC_ENVIRE_CORE_INCLUDE_DIRS} ${PC_ENVIRE_CORE_INCLUDEDIR}/../ ${PC_ENVIRE_CORE_INCLUDE_DIRS}/../
    PATH_SUFFIXES envire_core)
find_library(ENVIRE_CORE_LIBRARY NAME envire_core
    HINTS ${PC_ENVIRE_CORE_LIBDIR} ${PC_ENVIRE_CORE_LIBRARY_DIRS} )
set(ENVIRE_CORE_INCLUDE_DIRS ${ENVIRE_CORE_INCLUDE_DIR})
set(ENVIRE_CORE_LIBRARIES ${ENVIRE_CORE_LIBRARY})
##

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(envire_core DEFAULT_MSG ENVIRE_CORE_LIBRARY ENVIRE_CORE_INCLUDE_DIR)
