cmake_minimum_required(VERSION 3.5)

project(marsim_render)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif ()


set(CMAKE_BUILD_TYPE Release)  # Options: Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -Wall")
# FIXME! This is a temporary fix to suppress warnings from glad
add_compile_options(-Wno-pedantic)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS  "-O -Wall -g")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add compiler flags to treat all warnings as errors
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Enable all common warnings and treat them as errors
    message(STATUS "GCC or Clang detected, adding -Wall -Wextra -Werror")
    add_compile_options(-Wall -Wextra -Werror -Wunused)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Enable warning level 4 and treat all warnings as errors for MSVC
    message(STATUS "MSVC detected, adding /W4 /WX")
    add_compile_options(/W4 /WX)
endif ()


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


set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIRS}
        ${glfw3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        ${OpenCV_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${glfw3_LIBRARIES}
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
        glfw
        dl
        yaml-cpp
)

set(ros_libs
        rclcpp
        std_msgs
)


include_directories(
        ${includes}
)

add_library(${PROJECT_NAME} SHARED
        include/glad/glad.c
        src/marsim_render.cpp
)


ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_libs}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${third_party_libs}
)

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}
        ${third_party_libs}
)
ament_export_include_directories(${includes})
install(DIRECTORY include/
        DESTINATION include/
)

ament_package()