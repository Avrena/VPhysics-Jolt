cmake_minimum_required(VERSION 3.20)
project(vjolt_should_collide_cache_tests LANGUAGES CXX)

find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(VJOLT_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../vphysics_jolt")
set(VJOLT_LISTENER_SOURCE "${VJOLT_SOURCE}/vjolt_listener_contact.h" CACHE FILEPATH
    "Production listener source to extract; an older revision is a negative control")
option(VJOLT_CACHE_SANITIZERS "Enable address and undefined behaviour sanitizers" OFF)

add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/listener.inc"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
        "${CMAKE_CURRENT_BINARY_DIR}" --source "${VJOLT_LISTENER_SOURCE}"
    DEPENDS "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${VJOLT_LISTENER_SOURCE}"
    VERBATIM)
add_executable(should_collide_cache_tests test_cache.cpp)
add_executable(should_collide_listener_tests test_listener.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/listener.inc")
add_executable(should_collide_cache_benchmark benchmark.cpp)
foreach(target should_collide_cache_tests should_collide_listener_tests should_collide_cache_benchmark)
    target_include_directories(${target} PRIVATE "${VJOLT_SOURCE}" "${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_link_libraries(${target} PRIVATE Threads::Threads)
    if(VJOLT_CACHE_SANITIZERS)
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
endforeach()
enable_testing()
add_test(NAME should_collide_cache COMMAND should_collide_cache_tests)
add_test(NAME should_collide_listener COMMAND should_collide_listener_tests)
set_tests_properties(should_collide_cache should_collide_listener PROPERTIES TIMEOUT 30)
