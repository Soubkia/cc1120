cmake_minimum_required (VERSION 3.8)
project("cc1120")
find_package(Git REQUIRED)
find_package(Threads REQUIRED)
enable_testing()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

include(Safeguards)
include(CompilerOptionsMacros)
include(DownloadProject)

csi_add_cpp17_support()
# csi_set_default_compiler_options()
download_project(
    PROJ                googletest
    GIT_REPOSITORY      https://github.com/google/googletest.git
    GIT_TAG             master
    UPDATE_DISCONNECTED 1
    QUIET
)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

add_executable(main src/main.cc src/bcm2835.c src/cc1120.cc)
#add_executable(test_main src/main.test.cc)
#target_link_libraries(test_main gtest gtest_main gmock gmock_main)
#add_test(NAME test_main COMMAND test_main)
#add_executable(test_cc1120 src/cc1120.test.cc src/cc1120.cc src/bcm2835.c)
#add_executable(blink src/blink.cc src/bcm2835.c)
