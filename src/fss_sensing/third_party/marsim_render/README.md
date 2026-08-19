# marsim_render

`marsim_render` is an OpenGL/PCL-based local sensing library. It loads a static
map point cloud, renders synthetic LiDAR-like observations from a pose, and
returns point clouds in body frame (local point cloud) and in world frame (global point cloud).

This is inherited and improved from https://github.com/hku-mars/SUPER/tree/master/mars_uav_sim/marsim_render 

This library is ROS-independent and is a pure c++ project.

The OpenGL auto selects the graphic card for point cloud rendering, and a **Dedicated Graphics Card** achieves higher rendering FPS.

## What It Provides

- `marsim::MarsimRender`
  - loads render configuration from YAML
  - loads a map PCD file
  - renders local point clouds from a given pose
  - exposes the global map
- runtime resources
  - `config/`: render configuration and shader files
  - `pcd/`: example map point clouds
  - `src/map_generator/`: helper scripts for generating maps

## Build

Add the project as a subdirectory and link against `marsim_render`:

```cmake
add_subdirectory(path/to/marsim_render)
target_link_libraries(your_target marsim_render)
```

The library depends on:

- Eigen3
- OpenCV
- GLEW
- OpenGL
- glfw3
- PCL
- OpenMP
- yaml-cpp

## Runtime Inputs

Construct `marsim::MarsimRender` with:

- a YAML config path
- a `marsim::ResourcePaths` containing:
  - `pcd_directory`
  - `shader_directory`

The YAML should usually keep `pcd_name` as a file name relative to the PCD
directory, for example:

```yaml
pcd_name: random_map_150.pcd
```

## Installation

This project provides an optional install switch:

- `MARSIM_RENDER_ENABLE_INSTALL=OFF` by default
- when enabled, it installs:
  - `README.md`
  - `config/`
  - `pcd/`
  - `src/map_generator/`
