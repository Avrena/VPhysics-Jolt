cmake_minimum_required(VERSION 3.20)
project(vjolt_native_regressions LANGUAGES CXX)

# These targets compile the production bookkeeping and lifecycle code with
# narrow engine-interface shims. They do not require the Source SDK or Jolt.
option(VJOLT_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviourSanitizer" OFF)
if(VJOLT_SANITIZERS)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "VJOLT_SANITIZERS requires GCC or Clang")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

enable_testing()
add_subdirectory(pair_hash)
add_subdirectory(should_collide_cache)
add_subdirectory(lifecycle)
