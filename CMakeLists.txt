cmake_minimum_required(VERSION 3.16)

project(ProblemD_lp LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(PROBLEMD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ProblemD")

set(PD_SOURCES
    ${PROBLEMD_DIR}/src/pd_parser.c
    ${PROBLEMD_DIR}/src/pd_util.c
    ${PROBLEMD_DIR}/src/pd_clock.c
    ${PROBLEMD_DIR}/src/pd_timing.c
    ${PROBLEMD_DIR}/src/pd_output.c
)

set(LP_SOURCES
    src/main.cpp
    src/lp_branch.cpp
    src/lp_score.cpp
    src/lp_solve.cpp
    src/lp_solve_pg.cpp
    src/lp_solve_glpk.cpp
    src/lp_apply.cpp
)

add_executable(lp_solver ${PD_SOURCES} ${LP_SOURCES})

target_include_directories(lp_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROBLEMD_DIR}/include
)

target_compile_options(lp_solver PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra>
)

if(UNIX)
    target_link_libraries(lp_solver PRIVATE m)
endif()

option(USE_VENDORED_GLPK "Link vendored GLPK from lib/install" OFF)
if(USE_VENDORED_GLPK)
    set(GLPK_INSTALL "${CMAKE_CURRENT_SOURCE_DIR}/lib/install")
    target_include_directories(lp_solver PRIVATE "${GLPK_INSTALL}/include")
    target_compile_definitions(lp_solver PRIVATE LP_HAVE_GLPK)
    target_link_libraries(lp_solver PRIVATE "${GLPK_INSTALL}/lib/libglpk.a")
endif()

message(STATUS "ProblemD_lp: lp_solver (C++17 + Phase0 C)")
