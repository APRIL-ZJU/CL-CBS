cmake_minimum_required(VERSION 3.5)

project(CL-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

## OPEN MOTION PLANNING LIBRARY
find_package(OMPL REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cl_cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/neighbor.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/planresult.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/environment.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sh_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cl_cbs.cpp
)

add_custom_target(clang-format
  COMMAND clang-format -style=Google -i ${ALL_SOURCE_FILES} 
)

# tests
# add_custom_target(run-test
#   COMMAND python3 -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test
# )

add_custom_target(everything
  DEPENDS clang-format docs 
  # run-test
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if (${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")
else(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif (${CMAKE_BUILD_TYPE} MATCHES "Release") 

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

# Examples


## Spatiotemporal Hybrid-State A_star
add_executable(SH_Astar
  src/sh_astar.cpp
)
target_link_libraries(SH_Astar
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

## CL-CBS
add_executable(CL-CBS
  src/cl_cbs.cpp
)
target_link_libraries(CL-CBS
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  yaml-cpp
)

## test CL-CBS
# add_executable(test
#   test/test_CLCBS.cpp
# )
# target_link_libraries(test
#   ${Boost_LIBRARIES}
# )