# Copyright (c) 2026 Zhengzhong (Ricky) You.
# All rights reserved.
# Software: RouteOpt
# Application: IRBP

file(GLOB_RECURSE sources "${SOURCE_ROOT}/src/*.cpp" "${SOURCE_ROOT}/src/*.hpp" "${SOURCE_ROOT}/include/*.hpp")
foreach(source IN LISTS sources)
  file(READ "${source}" contents)
  # The ARC incumbent trajectory labels its events with the callback name; this is not a backend call.
  string(REPLACE "\"GRB_CB_MIPSOL\"" "" contents "${contents}")
  if(contents MATCHES "GRB[a-zA-Z_]|COPT[a-zA-Z_]|gurobi_c[.]h|[.]solver[.](model|env)")
    message(FATAL_ERROR "Backend call outside Solver interface: ${source}")
  endif()
endforeach()
