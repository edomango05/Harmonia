cmake_minimum_required(VERSION 3.14)
project(Harmonia C)

set(CMAKE_C_STANDARD 11)

# Include FetchContent to download raylib automatically
include(FetchContent)

message(STATUS "Downloading and building raylib...")
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.0  # You can use "master" for the latest, but 5.0 is stable
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(raylib)

# Include the headers from our project
include_directories(include)

# Find all source files
file(GLOB SOURCES "src/*.c")

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link raylib to our executable
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)