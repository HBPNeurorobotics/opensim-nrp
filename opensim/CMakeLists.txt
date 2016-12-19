cmake_minimum_required(VERSION 3.5)
set(CMAKE_VERBOSE_MAKEFILE On)
project(NRP_OpenSim_Benchmark)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(BUILD_VERSIONED_LIBS True)
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

set(benchmark_SRCS  main.cpp
                    OpenSimSimulation.cpp)

set(benchmark_HEADERS OpenSimSimulation.h)

include_directories(${Boost_INCLUDE_DIR})
                    
add_executable(${PROJECT_NAME} ${benchmark_SRCS})
target_link_libraries(${PROJECT_NAME} ${OPENSIM_LIBRARIES} ${Simbody_LIBRARIES}
                      ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
