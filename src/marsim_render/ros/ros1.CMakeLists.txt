cmake_minimum_required(VERSION 3.10)
project(marsim_render)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)  # Options: Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -Wall")
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS  "-O -Wall -g")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Add compiler flags to treat all warnings as errors
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Enable all common warnings and treat them as errors
    message(STATUS "GCC or Clang detected, adding -Wall -Wextra -Werror")
    add_compile_options(-Wall -Wextra -Werror -Wunused)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Enable warning level 4 and treat all warnings as errors for MSVC
    message(STATUS "MSVC detected, adding /W4 /WX")
    add_compile_options(/W4 /WX)
endif()



# Add preprocessor definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${glfw3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        DEPENDS OpenCV
)


add_library(GLAD include/glad/glad.c)


add_library(${PROJECT_NAME}
        src/marsim_render.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${glfw3_LIBRARIES}
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
        GLAD
        glfw
        dl
        yaml-cpp
)





add_executable(test_render
    test/main.cpp
)

target_compile_features(test_render PUBLIC cxx_std_17)

target_link_libraries(test_render PUBLIC
        ${PROJECT_NAME}
)
