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

add_executable(bouncing-sphere-model bouncing_sphere_model.cpp)
add_executable(spinning-sphere-model spinning_sphere_model.cpp)
add_executable(dynamic-topology dynamic_topology.cpp)

LIST (APPEND ALL_LIBS ${OPENSIM_LIBRARIES} ${Simbody_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}) # 
target_link_libraries(bouncing-sphere-model ${ALL_LIBS})
target_link_libraries(spinning-sphere-model ${ALL_LIBS})
target_link_libraries(dynamic-topology ${ALL_LIBS})
