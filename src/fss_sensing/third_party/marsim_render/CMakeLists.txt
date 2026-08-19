cmake_minimum_required(VERSION 3.8)
project(marsim_render LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(marsim_render STATIC
  include/glad/glad.c
  src/marsim_render.cpp
)

target_compile_features(marsim_render PUBLIC cxx_std_17)
target_include_directories(
  marsim_render
  SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  marsim_render
  PUBLIC
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    glfw
    dl
    yaml-cpp
)

option(MARSIM_RENDER_ENABLE_INSTALL "Install marsim_render runtime resources" OFF)

if(MARSIM_RENDER_ENABLE_INSTALL)
  if(NOT DEFINED MARSIM_RENDER_INSTALL_RESOURCE_DESTINATION)
    message(FATAL_ERROR "MARSIM_RENDER_INSTALL_RESOURCE_DESTINATION must be set when install is enabled")
  endif()
  install(FILES
    README.md
    DESTINATION "${MARSIM_RENDER_INSTALL_RESOURCE_DESTINATION}"
  )
  install(DIRECTORY
    config
    pcd
    src/map_generator
    DESTINATION "${MARSIM_RENDER_INSTALL_RESOURCE_DESTINATION}"
  )
endif()
