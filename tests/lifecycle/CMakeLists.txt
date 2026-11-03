cmake_minimum_required(VERSION 3.20)
project(vjolt_lifecycle_tests LANGUAGES CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(VJOLT_BASELINE_REF "" CACHE STRING "Optional git ref for the original lifecycle implementation")
get_filename_component(VJOLT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${VJOLT_ROOT}/vphysics_jolt/vjolt_environment_lifecycle.cpp"
    "${VJOLT_ROOT}/vphysics_jolt/vjolt_environment.cpp"
    "${VJOLT_ROOT}/vphysics_jolt/vjolt_object.cpp"
    "${VJOLT_ROOT}/vphysics_jolt/vjolt_constraints.cpp"
    "${VJOLT_ROOT}/vphysics_jolt/vjolt_constraints.h"
    "${CMAKE_CURRENT_LIST_DIR}/extract_sources.py")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/extract_sources.py"
    --root "${VJOLT_ROOT}" --output "${CMAKE_CURRENT_BINARY_DIR}"
    --baseline-ref "${VJOLT_BASELINE_REF}"
    RESULT_VARIABLE extract_result ERROR_VARIABLE extract_error)
if(NOT extract_result EQUAL 0)
    message(FATAL_ERROR "Cannot extract production lifecycle methods: ${extract_error}")
endif()
add_executable(lifecycle_tests test_lifecycle.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/lifecycle.cpp"
    "${CMAKE_CURRENT_BINARY_DIR}/shared_methods.cpp")
target_include_directories(lifecycle_tests PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_compile_features(lifecycle_tests PRIVATE cxx_std_17)
enable_testing()
foreach(scenario simulation deferred_objects queue_growth nested_cleanup constraints
    listener_mutation collide_lifetime teardown teardown_snapshot cache_invalidation transfer)
    add_test(NAME "lifecycle_${scenario}" COMMAND lifecycle_tests "${scenario}")
endforeach()
