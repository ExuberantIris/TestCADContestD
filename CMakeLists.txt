cmake_minimum_required(VERSION 3.16)

project(ProblemD LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Make graaf headers visible to all targets/editors.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/graaf/include)

# Common warnings
add_compile_options(
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
)

# Main C solver (Phase 0)
add_executable(cadd_problem_d
    src/main.c
    src/pd_util.c
    src/pd_parser.c
    src/pd_clock.c
    src/pd_timing.c
    src/pd_output.c
    src/pd_checker.c
)

target_include_directories(cadd_problem_d PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if (CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Optional C++ utility target for buffer parsing
add_library(problemd_utils
    utils/struct/buffer.cpp
    utils/record/buffer_record.cpp
    utils/record/ff_and_path_record.cpp
    utils/record/graph_record.cpp
)

target_include_directories(problemd_utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/struct
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/record
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/graaf/include
)

# Unit tests
add_executable(test_buffer_record unit_test/src/test_buffer_record.cpp)
add_executable(test_ff_and_path_record unit_test/src/test_ff_and_path_record.cpp)
add_executable(test_graph_record unit_test/src/test_graph_record.cpp)

target_link_libraries(test_buffer_record PRIVATE problemd_utils)
target_link_libraries(test_ff_and_path_record PRIVATE problemd_utils)
target_link_libraries(test_graph_record PRIVATE problemd_utils)

enable_testing()
add_test(NAME buffer_record_parse COMMAND test_buffer_record)
add_test(NAME ff_and_path_record_parse COMMAND test_ff_and_path_record)
add_test(NAME graph_record_parse COMMAND test_graph_record)

# Helpful status message
message(STATUS "Targets: cadd_problem_d, problemd_utils, test_buffer_record, test_ff_and_path_record, test_graph_record")
