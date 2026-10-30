# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
cmake_minimum_required(VERSION 3.20)
project(IRBP LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(IRBP_BUILD_EXACT "Build irbp_exact (ARC LP, ARC MIP, DW LP); requires Boost headers" ON)
option(IRBP_BUILD_EXPERIMENTS "Build the experiment drivers used in the paper" ON)
set(IRBP_THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(IRBP_EXAMPLES "${CMAKE_CURRENT_SOURCE_DIR}/examples")

find_package(Threads REQUIRED)
find_path(GUROBI_INCLUDE_DIR gurobi_c.h
  HINTS "$ENV{GUROBI_HOME}/include" /Library/gurobi1300/macos_universal2/include
        /Library/gurobi1203/macos_universal2/include /opt/gurobi1300/linux64/include /opt/gurobi1203/linux64/include)
find_library(GUROBI_LIBRARY NAMES gurobi130 gurobi120
  HINTS "$ENV{GUROBI_HOME}/lib" /Library/gurobi1300/macos_universal2/lib /Library/gurobi1203/macos_universal2/lib
        /opt/gurobi1300/linux64/lib /opt/gurobi1203/linux64/lib)
if(NOT GUROBI_INCLUDE_DIR OR NOT GUROBI_LIBRARY)
  message(FATAL_ERROR "Gurobi 12 or 13 not found: set GUROBI_HOME, or GUROBI_INCLUDE_DIR and GUROBI_LIBRARY")
endif()

# RouteOpt solver interface: the only code that calls the Gurobi C API.
add_library(routeopt_solver STATIC third_party/routeopt/src/solver_grb.cpp)
target_include_directories(routeopt_solver PUBLIC third_party/routeopt/include ${GUROBI_INCLUDE_DIR})
target_compile_definitions(routeopt_solver PUBLIC SOLVER_TYPE=0)
target_link_libraries(routeopt_solver PUBLIC ${GUROBI_LIBRARY})

# miniz: ZIP reader for compressed and packed instance files.
add_library(irbp_zip STATIC third_party/miniz/miniz.c third_party/miniz/miniz_tinfl.c third_party/miniz/miniz_zip.c)
target_include_directories(irbp_zip PUBLIC third_party/miniz)
target_compile_definitions(irbp_zip PUBLIC MINIZ_NO_DEFLATE_APIS MINIZ_NO_ZLIB_COMPATIBLE_NAMES)

include(CTest)
add_subdirectory(heuristic)
if(IRBP_BUILD_EXACT)
  add_subdirectory(exact)
endif()
if(IRBP_BUILD_EXPERIMENTS)
  add_subdirectory(experiments)
endif()
