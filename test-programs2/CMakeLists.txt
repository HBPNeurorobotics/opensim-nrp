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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_executable(spinning-sphere-model spinning_sphere_model.cpp)
add_executable(dynamic-topology dynamic_topology.cpp)
add_executable(driven-spinning-sphere driven_spinning_sphere.cpp MobilityDiscreteForce.cpp)
add_executable(tug-of-war-osim-load tug_of_war_osim_load.cpp)
add_executable(tug-of-war-modified tug_of_war_modified.cpp)

LIST (APPEND ALL_LIBS ${OPENSIM_LIBRARIES} ${Simbody_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}) # 
target_link_libraries(spinning-sphere-model ${ALL_LIBS})
target_link_libraries(dynamic-topology ${ALL_LIBS})
target_link_libraries(driven-spinning-sphere ${ALL_LIBS})
target_link_libraries(tug-of-war-osim-load ${ALL_LIBS})
target_link_libraries(tug-of-war-modified ${ALL_LIBS})
