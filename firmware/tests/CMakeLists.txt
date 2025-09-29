cmake_minimum_required(VERSION 3.14)
project(pico_host_tests CXX)

# This CMakeLists builds host-side tests (runs on the developer machine), not
# firmware for the RP2040. It's guarded by BUILD_HOST_TESTS in the top-level
# CMakeLists and is OFF by default.

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Prefer system-installed GoogleTest. If not found, optionally fetch it when
# the user explicitly enables network fetches using -DENABLE_FETCH_GOOGLETEST=ON.
find_package(GTest QUIET)
set(HAVE_GTEST OFF)
if(GTest_FOUND)
  set(HAVE_GTEST ON)
else()
  if(DEFINED ENABLE_FETCH_GOOGLETEST AND ENABLE_FETCH_GOOGLETEST)
    include(FetchContent)
    message(STATUS "GoogleTest not found: attempting to fetch googletest via FetchContent (ENABLE_FETCH_GOOGLETEST=ON)")
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/release-1.15.0.zip
    )
    # Try to fetch; if it fails the configure will error. This is opt-in.
    FetchContent_MakeAvailable(googletest)
    find_package(GTest REQUIRED)
    if(GTest_FOUND)
      set(HAVE_GTEST ON)
    endif()
  else()
    message(WARNING "GoogleTest not found. Tests that require GoogleTest will be skipped. Tests that provide their own main() will still be built and registered.")
  endif()
endif()

enable_testing()
set(CMAKE_CXX_STANDARD 17)

option(BUILD_PLACEHOLDER_TESTS "Build tests that are marked as NOT IMPLEMENTED" OFF)

# Determine repository root so this CMakeLists can be used either as a
# subdirectory of the firmware project or as a standalone top-level project
# (recommended for host tests so the host compiler is used instead of the
# RP2040 cross-toolchain).
if(NOT DEFINED REPO_ROOT)
  # tests/ is expected to live in the repo root: <repo>/tests
  get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
  message(STATUS "Inferred REPO_ROOT=${REPO_ROOT}")
else()
  message(STATUS "Using user-supplied REPO_ROOT=${REPO_ROOT}")
endif()

# Add host-only include dirs so tests that #include implementation files or
# project sources can find headers when built on the developer machine.
# This includes the repo 'src' tree and a small test-only stubs folder.
include_directories(
  ${REPO_ROOT}/tests/host_stubs
  ${REPO_ROOT}/src
)

# If this configure is using a cross toolchain (e.g. arm-none-eabi), warn the
# user because building host tests in the same configure will try to use the
# cross compiler and fail. Prefer configuring the tests directory as a
# separate CMake project (see README or instructions below).
if(CMAKE_C_COMPILER MATCHES "arm-none-eabi|arm-none-eabi-gcc|/opt/.*/arm-none-eabi/" OR CMAKE_CXX_COMPILER MATCHES "arm-none-eabi")
  message(WARNING "Detected cross-toolchain (CMAKE_C_COMPILER=${CMAKE_C_COMPILER}).\n  Host tests should be configured in a separate host build directory so the host compiler is used.\n  Example: \n    cmake -S tests -B build-host -DREPO_ROOT=.. -DCMAKE_BUILD_TYPE=Debug\n  or from repo root: \n    cmake -S . -B build-host -DBUILD_HOST_TESTS=OFF && cmake -S tests -B build-host -DREPO_ROOT=..")
endif()

# Collect test sources in this directory (top-level tests folder)
## Recursively collect test sources so files in subdirectories (contract/, integration/) are discovered
file(GLOB_RECURSE TEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

if(TEST_SOURCES)
  message(STATUS "Found ${TEST_SOURCES}" )
else()
  message(WARNING "No test source files found under ${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# For each .cpp file create an executable and register with CTest
foreach(test_relpath ${TEST_SOURCES})
  # full path to source
  set(test_src "${CMAKE_CURRENT_SOURCE_DIR}/${test_relpath}")
  # create a deterministic, unique target name from relative path: replace non-alnum with '_'
  string(REGEX REPLACE "[^A-Za-z0-9]" "_" test_name_sanitized "${test_relpath}")
  set(target_name "host_test_${test_name_sanitized}")

  # Read the source and detect whether it already defines main()
  file(READ ${test_src} test_src_content)
  string(FIND "${test_src_content}" "int main(" found_main_pos)
  string(FIND "${test_src_content}" "NOT IMPLEMENTED" found_notimpl_pos)

  # If the source is a placeholder (contains the marker "NOT IMPLEMENTED")
  # then skip building it by default. Developers can enable building these
  # placeholders with -DBUILD_PLACEHOLDER_TESTS=ON when desired.
  if(NOT found_notimpl_pos EQUAL -1 AND NOT BUILD_PLACEHOLDER_TESTS)
    message(STATUS "Skipping placeholder test ${test_relpath} (contains 'NOT IMPLEMENTED')")
    continue()
  endif()

  add_executable(${target_name} ${test_src})
  target_include_directories(${target_name} PRIVATE
    ${REPO_ROOT}/libmicroros/include
    ${REPO_ROOT}/include
  )

  # Define HOST_BUILD for host tests
  target_compile_definitions(${target_name} PRIVATE HOST_BUILD)

  if(found_main_pos EQUAL -1)
    # No main found in the source. If we have GTest, link to gtest_main; otherwise link to gtest if available.
    if(HAVE_GTEST)
      target_link_libraries(${target_name} PRIVATE GTest::gtest_main)
    else()
      message(WARNING "No main() in ${test_relpath} and GoogleTest is not available; skipping ${test_relpath}.")
      # remove the target we just added
      remove_executable(${target_name})
      continue()
    endif()
  else()
    # Source already contains a main(); build as a standalone test executable (no gtest_main)
    message(STATUS "Building standalone test executable for ${test_relpath} (provides its own main)")
    # If the source contains gtest usages and GTest is available, link the test
    # target to the GTest library (not gtest_main) so EXPECT_/ASSERT_ symbols are
    # resolved. We detect GTest usage by presence of <gtest/gtest.h> include.
    string(FIND "${test_src_content}" "<gtest/gtest.h>" found_gtest_include)
    if(found_gtest_include GREATER -1)
      if(HAVE_GTEST)
        target_link_libraries(${target_name} PRIVATE GTest::gtest)
      else()
        message(WARNING "Test ${test_relpath} uses GoogleTest but GTest was not found; skipping.")
        remove_executable(${target_name})
        continue()
      endif()
    endif()
  endif()

  # If a host_globals.cpp exists in tests/host_stubs, link it into each test so
  # firmware-global symbols (SetpointBuffer, data_lock, etc.) are satisfied for
  # host tests that include implementation files directly.
  if(EXISTS ${REPO_ROOT}/tests/host_stubs/host_globals.cpp)
    target_sources(${target_name} PRIVATE ${REPO_ROOT}/tests/host_stubs/host_globals.cpp)
  endif()

  add_test(NAME ${target_name} COMMAND ${target_name})

  # Label each test as 'host' for IDE/test explorers if gtest target exists
  if(HAVE_GTEST AND TARGET GTest::gtest)
    set_tests_properties(${target_name} PROPERTIES LABELS "host")
  endif()
endforeach()
