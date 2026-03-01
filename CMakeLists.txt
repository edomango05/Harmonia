cmake_minimum_required(VERSION 3.14)
project(Harmonia C)

set(CMAKE_C_STANDARD 11)

# Include FetchContent to download raylib automatically
include(FetchContent)

message(STATUS "Downloading and building raylib...")
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.0
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(raylib)

include_directories(include)

file(GLOB SOURCES "src/*.c")

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE raylib)
