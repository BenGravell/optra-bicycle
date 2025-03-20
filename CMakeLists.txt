cmake_minimum_required(VERSION 3.23)
project(optra_bicycle)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Find packages
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(indicators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(raylib REQUIRED)

# Add all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")
message(STATUS "Source files: ${SOURCES}")

# Manually include main.cpp from the repo root
set(MAIN_FILE ${CMAKE_SOURCE_DIR}/src/main.cpp)

# Add the executable, combining main.cpp and the other source files
add_executable(${PROJECT_NAME} ${MAIN_FILE} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Link external libraries to the target
target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        glog::glog 
        Eigen3::Eigen 
        indicators::indicators
        nlohmann_json::nlohmann_json
        raylib
)