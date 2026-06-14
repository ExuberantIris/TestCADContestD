cmake_minimum_required(VERSION 3.16)

project(ProblemD_SA LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(PD_SOURCES
    src/pd_parser.c
    src/pd_util.c
    src/pd_clock.c
    src/pd_timing.c
    src/pd_output.c
)

set(SA_SOURCES
    src/main.cpp
    src/lp_branch.cpp
    src/lp_score.cpp
    src/lp_buffer_dp.cpp
    src/lp_mo_init.cpp
    src/sa_eval.cpp
    src/sa_path_solve.cpp
    src/sa_apply.cpp
)

add_executable(sa_solver ${PD_SOURCES} ${SA_SOURCES})
target_include_directories(sa_solver PRIVATE include)

target_compile_options(sa_solver PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra>
)

if(UNIX)
    target_link_libraries(sa_solver PRIVATE m)
endif()

message(STATUS "ProblemD_SA: sa_solver")
