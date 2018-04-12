cmake_minimum_required (VERSION 3.0)
set(TARGET_name pom-base)
project (${TARGET_name} VERSION 0.1)

set(SOURCES src/PositionManagerBase.cpp)

add_library(${TARGET_name} ${SOURCES})
target_link_libraries(${TARGET_name} ${ENVIRE_LIBRARIES})




