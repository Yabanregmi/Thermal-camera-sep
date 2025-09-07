cmake_minimum_required(VERSION 3.10)

project(myProjekt LANGUAGES CXX)

# Setze den Standard für C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.8.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)


set(MYTESTS_SOURCES
    src/add.cpp
    tests/test.cpp
)

set(MYTESTS_HEADERS
    inc/add.hpp
)

set(MYPROJECT_SOURCES
    src/main.cpp
)

set(MYPROJECT_HEADERS
)

add_executable(myTests ${MYTESTS_SOURCES} ${MYTESTS_HEADERS})
add_executable(myProjekt ${MYPROJECT_SOURCES} ${MYPROJECT_HEADERS})

target_link_libraries(myTests PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(myTests)
