cmake_minimum_required(VERSION 3.20)
project(vjolt_objectpairhash_tests LANGUAGES CXX)

# Compile the real implementation without pulling in the engine/Jolt PCH.
# Only the two engine include lines are replaced; no implementation is copied
# into the test or substituted with a model of the hash.
set(VJOLT_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../vphysics_jolt")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${VJOLT_SOURCE}/vjolt_objectpairhash.cpp")
file(READ "${VJOLT_SOURCE}/vjolt_objectpairhash.cpp" PAIR_HASH_SOURCE)
string(REPLACE "#include \"cbase.h\"" "#include \"test_support.h\"" PAIR_HASH_SOURCE "${PAIR_HASH_SOURCE}")
string(REPLACE "#include \"tier0/memdbgon.h\"" "" PAIR_HASH_SOURCE "${PAIR_HASH_SOURCE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/vjolt_objectpairhash.cpp" "${PAIR_HASH_SOURCE}")
add_executable(objectpairhash_tests test_objectpairhash.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/vjolt_objectpairhash.cpp")
target_include_directories(objectpairhash_tests PRIVATE "${VJOLT_SOURCE}" "${CMAKE_CURRENT_LIST_DIR}")
target_compile_features(objectpairhash_tests PRIVATE cxx_std_17)
enable_testing()
add_test(NAME objectpairhash COMMAND objectpairhash_tests)
