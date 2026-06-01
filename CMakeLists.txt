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

set(PD_SOURCES
    src/pd_parser.c
    src/pd_util.c
    src/pd_clock.c
    src/pd_timing.c
    src/pd_output.c
)

set(LP_CORE_SOURCES
    src/lp_branch.cpp
    src/lp_score.cpp
    src/lp_solve.cpp
    src/lp_solve_pg.cpp
    src/lp_solve_glpk.cpp
    src/lp_apply.cpp
    src/lp_buffer_dp.cpp
)

add_executable(lp_solver ${PD_SOURCES} src/main.cpp ${LP_CORE_SOURCES})

target_include_directories(lp_solver PRIVATE include)

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

add_executable(test_buffer_chain_dp
    ${PD_SOURCES}
    unittest/test_buffer_chain_dp.cpp
    src/lp_branch.cpp
    src/lp_buffer_dp.cpp
)
target_include_directories(test_buffer_chain_dp PRIVATE include)
target_compile_options(test_buffer_chain_dp PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra>
)
if(UNIX)
    target_link_libraries(test_buffer_chain_dp PRIVATE m)
endif()

enable_testing()
add_test(NAME buffer_chain_dp_testcase1 COMMAND test_buffer_chain_dp
    ${CMAKE_CURRENT_SOURCE_DIR}/testcase/testcase1)

message(STATUS "ProblemD: lp_solver, test_buffer_chain_dp (C++17 + Phase0 C)")
