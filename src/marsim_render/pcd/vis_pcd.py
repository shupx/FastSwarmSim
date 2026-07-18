#!/usr/bin/env python3
"""
PCD File Visualizer
Simple tool to visualize point cloud files in PCD format

@author: Peixuan Shu
@date: Jan 2026

Usage Examples:
--------------------
# 交互式模式（自动列出文件让你选择）
python3 vis_pcd.py

# 查看单个文件
python3 vis_pcd.py random_map.pcd

# 查看多个文件（用不同颜色）
python3 vis_pcd.py map1.pcd map2.pcd map3.pcd

# 列出所有PCD文件
python3 vis_pcd.py --list

# 自定义点大小
python3 vis_pcd.py random_map.pcd --point-size 2.0

# 隐藏坐标轴
python3 vis_pcd.py random_map.pcd --no-axes

# 自定义背景色（白色背景）
python3 vis_pcd.py random_map.pcd --bg-color 1.0 1.0 1.0
"""

import open3d as o3d
import numpy as np
import argparse
import os
import glob


def visualize_pcd(pcd_file, show_axes=True, point_size=1.0, background_color=[0.1, 0.1, 0.1]):
    """
    Visualize a single PCD file
    
    Args:
        pcd_file: Path to the PCD file
        show_axes: Whether to show coordinate axes
        point_size: Size of points in visualization
        background_color: RGB background color [0-1, 0-1, 0-1]
    """
    if not os.path.exists(pcd_file):
        print(f"Error: File '{pcd_file}' not found!")
        return False
    
    print(f"Loading point cloud: {pcd_file}")
    
    # Load point cloud
    pcd = o3d.io.read_point_cloud(pcd_file)
    
    if not pcd.has_points():
        print("Error: Point cloud is empty!")
        return False
    
    # Print point cloud information
    print(f"Number of points: {len(pcd.points)}")
    print(f"Has normals: {pcd.has_normals()}")
    print(f"Has colors: {pcd.has_colors()}")
    
    # Calculate bounding box
    aabb = pcd.get_axis_aligned_bounding_box()
    print(f"Bounding box: min={aabb.min_bound}, max={aabb.max_bound}")
    
    # Add color if point cloud doesn't have colors
    if not pcd.has_colors():
        # Default color: light gray
        pcd.paint_uniform_color([0.7, 0.7, 0.7])
    
    # Create visualization list
    geometries = [pcd]
    
    # Add coordinate frame if requested
    if show_axes:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0]
        )
        geometries.append(coordinate_frame)
    
    # Visualize
    print("\nVisualization Controls:")
    print("  - Mouse left button: Rotate")
    print("  - Mouse right button: Pan")
    print("  - Mouse wheel: Zoom")
    print("  - Q/ESC: Quit")
    print("  - H: Print help")
    print("\nOpening visualization window...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"PCD Viewer - {os.path.basename(pcd_file)}", 
                      width=1280, height=720)
    
    for geom in geometries:
        vis.add_geometry(geom)
    
    # Set render options
    render_option = vis.get_render_option()
    render_option.point_size = point_size
    render_option.background_color = np.array(background_color)
    
    # Run visualizer
    vis.run()
    vis.destroy_window()
    
    return True


def visualize_multiple_pcd(pcd_files, show_axes=True, point_size=1.0, 
                          background_color=[0.1, 0.1, 0.1], random_colors=True):
    """
    Visualize multiple PCD files together
    
    Args:
        pcd_files: List of PCD file paths
        show_axes: Whether to show coordinate axes
        point_size: Size of points in visualization
        background_color: RGB background color [0-1, 0-1, 0-1]
        random_colors: Assign random colors to different point clouds
    """
    if not pcd_files:
        print("Error: No PCD files provided!")
        return False
    
    print(f"Loading {len(pcd_files)} point clouds...")
    
    geometries = []
    total_points = 0
    
    # Define color palette for different point clouds
    colors = [
        [1.0, 0.0, 0.0],  # Red
        [0.0, 1.0, 0.0],  # Green
        [0.0, 0.0, 1.0],  # Blue
        [1.0, 1.0, 0.0],  # Yellow
        [1.0, 0.0, 1.0],  # Magenta
        [0.0, 1.0, 1.0],  # Cyan
        [1.0, 0.5, 0.0],  # Orange
        [0.5, 0.0, 1.0],  # Purple
    ]
    
    for idx, pcd_file in enumerate(pcd_files):
        if not os.path.exists(pcd_file):
            print(f"Warning: File '{pcd_file}' not found! Skipping...")
            continue
        
        # Load point cloud
        pcd = o3d.io.read_point_cloud(pcd_file)
        
        if not pcd.has_points():
            print(f"Warning: Point cloud '{pcd_file}' is empty! Skipping...")
            continue
        
        num_points = len(pcd.points)
        total_points += num_points
        print(f"  [{idx+1}] {os.path.basename(pcd_file)}: {num_points} points")
        
        # Assign color if requested
        if random_colors:
            color = colors[idx % len(colors)]
            pcd.paint_uniform_color(color)
        elif not pcd.has_colors():
            pcd.paint_uniform_color([0.7, 0.7, 0.7])
        
        geometries.append(pcd)
    
    if not geometries:
        print("Error: No valid point clouds loaded!")
        return False
    
    print(f"\nTotal points: {total_points}")
    
    # Add coordinate frame if requested
    if show_axes:
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0]
        )
        geometries.append(coordinate_frame)
    
    # Visualize
    print("\nVisualization Controls:")
    print("  - Mouse left button: Rotate")
    print("  - Mouse right button: Pan")
    print("  - Mouse wheel: Zoom")
    print("  - Q/ESC: Quit")
    print("  - H: Print help")
    print("\nOpening visualization window...")
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"PCD Viewer - {len(geometries)-1 if show_axes else len(geometries)} files", 
                      width=1280, height=720)
    
    for geom in geometries:
        vis.add_geometry(geom)
    
    # Set render options
    render_option = vis.get_render_option()
    render_option.point_size = point_size
    render_option.background_color = np.array(background_color)
    
    # Run visualizer
    vis.run()
    vis.destroy_window()
    
    return True


def list_pcd_files(directory="."):
    """List all PCD files in a directory"""
    pcd_pattern = os.path.join(directory, "*.pcd")
    pcd_files = glob.glob(pcd_pattern)
    
    if not pcd_files:
        print(f"No PCD files found in '{directory}'")
        return []
    
    print(f"Found {len(pcd_files)} PCD file(s) in '{directory}':")
    for idx, pcd_file in enumerate(sorted(pcd_files)):
        print(f"  [{idx+1}] {os.path.basename(pcd_file)}")
    
    return sorted(pcd_files)


def main():
    parser = argparse.ArgumentParser(
        description='Visualize PCD (Point Cloud Data) files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Visualize a single PCD file
  python vis_pcd.py map.pcd
  
  # Visualize multiple PCD files
  python vis_pcd.py map1.pcd map2.pcd map3.pcd
  
  # List all PCD files in current directory
  python vis_pcd.py --list
  
  # Visualize with custom point size
  python vis_pcd.py map.pcd --point-size 2.0
  
  # Visualize without coordinate axes
  python vis_pcd.py map.pcd --no-axes
        """
    )
    
    parser.add_argument('files', nargs='*', help='PCD file(s) to visualize')
    parser.add_argument('--list', '-l', action='store_true', 
                       help='List all PCD files in current directory')
    parser.add_argument('--dir', '-d', type=str, default='.',
                       help='Directory to search for PCD files (default: current directory)')
    parser.add_argument('--point-size', '-s', type=float, default=1.0,
                       help='Point size for visualization (default: 1.0)')
    parser.add_argument('--no-axes', action='store_true',
                       help='Hide coordinate axes')
    parser.add_argument('--bg-color', nargs=3, type=float, default=[0.1, 0.1, 0.1],
                       metavar=('R', 'G', 'B'),
                       help='Background color RGB values 0-1 (default: 0.1 0.1 0.1)')
    parser.add_argument('--no-random-colors', action='store_true',
                       help='Disable random colors for multiple files')
    
    args = parser.parse_args()
    
    # List mode
    if args.list:
        list_pcd_files(args.dir)
        return
    
    # Check if files are provided
    if not args.files:
        # If no files specified, try to find PCD files in current directory
        pcd_files = list_pcd_files(args.dir)
        if not pcd_files:
            parser.print_help()
            return
        
        # Ask user to select a file
        print("\nEnter file number to visualize (or 'all' for all files): ", end='')
        user_input = input().strip()
        
        if user_input.lower() == 'all':
            args.files = pcd_files
        else:
            try:
                idx = int(user_input) - 1
                if 0 <= idx < len(pcd_files):
                    args.files = [pcd_files[idx]]
                else:
                    print("Invalid selection!")
                    return
            except ValueError:
                print("Invalid input!")
                return
    
    show_axes = not args.no_axes
    random_colors = not args.no_random_colors
    
    # Visualize single or multiple files
    if len(args.files) == 1:
        visualize_pcd(args.files[0], 
                     show_axes=show_axes,
                     point_size=args.point_size,
                     background_color=args.bg_color)
    else:
        visualize_multiple_pcd(args.files,
                              show_axes=show_axes,
                              point_size=args.point_size,
                              background_color=args.bg_color,
                              random_colors=random_colors)


if __name__ == '__main__':
    main()
