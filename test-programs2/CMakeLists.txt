cmake_minimum_required(VERSION 3.5)
project(WelterTestPrograms)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(Simbody)
if (Simbody_FOUND)
    message(STATUS "Found Simbody installation.")
    message(STATUS " include directory: ${Simbody_INCLUDE_DIR}")
    message(STATUS " libraries: ${Simbody_LIBRARIES}")
    include_directories(${Simbody_INCLUDE_DIR})
    link_directories(${Simbody_LIB_DIR})
else()
    message(FATAL_ERROR "No Simbody installation found!")
endif()

find_package(OpenSim)
if (OPENSIM_FOUND)
    message(STATUS "Found OpenSim installation.")
    message(STATUS " include directory: ${OPENSIM_INCLUDE_DIRS}")
    message(STATUS " libraries: ${OPENSIM_LIBRARIES}")
    include_directories(${OPENSIM_INCLUDE_DIRS})
    link_directories(${OPENSIM_LIB_DIR})
else()
    message(FATAL_ERROR "No OpenSim installation found!")
endif()

find_package(Boost COMPONENTS program_options filesystem REQUIRED)

#include_directories(${CMAKE_SOURCE_DIR}/common)

add_executable(bouncing-sphere-model bouncing_sphere_model.cpp)

target_link_libraries(bouncing-sphere-model ${OPENSIM_LIBRARIES} ${Simbody_LIBRARIES}
                      ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
