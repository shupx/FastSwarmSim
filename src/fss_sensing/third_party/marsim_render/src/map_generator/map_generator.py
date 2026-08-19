#!/usr/bin/env python3
"""
Random Map Generator with Cylinders and Torus Obstacles
Generates random 3D point cloud maps with configurable field dimensions
@author: Peixuan Shu
@date: Jan 2026
pip3 install open3d numpy pyyaml
"""

import open3d as o3d
import numpy as np
import argparse
import yaml
import os


class MapGenerator:
    def __init__(self, config):
        """
        Initialize the map generator with configuration
        
        Args:
            config: Dictionary containing configuration parameters
        """
        self.config = config
        self.seed = config.get('seed', 42)
        np.random.seed(self.seed)
        
        # Field dimensions
        self.field_x = config.get('field_x', 20.0)
        self.field_y = config.get('field_y', 20.0)
        self.field_z = config.get('field_z', 5.0)
        
        # Obstacle parameters
        self.num_cylinders = config.get('num_cylinders', 10)
        self.num_torus = config.get('num_torus', 5)
        
        # Cylinder parameters
        self.cyl_radius_range = config.get('cyl_radius_range', [0.2, 0.5])
        self.cyl_height_range = config.get('cyl_height_range', [1.0, 3.0])
        self.cyl_points = config.get('cyl_points', 3000)
        
        # Torus parameters
        self.torus_major_radius_range = config.get('torus_major_radius_range', [0.8, 1.5])
        self.torus_minor_radius_range = config.get('torus_minor_radius_range', [0.15, 0.3])
        self.torus_points = config.get('torus_points', 3000)
        
        # Output
        self.output_file = config.get('output_file', 'random_map.pcd')
        self.obstacle_config_file = config.get('obstacle_config_file', None)
        self.save_obstacle_config_file = config.get('save_obstacle_config_file', None)
        
        # Optional: Add boundary walls
        self.add_boundaries = config.get('add_boundaries', False)
        self.boundary_points = config.get('boundary_points', 10000)
        
        # Optional: Add ground plane
        self.add_ground = config.get('add_ground', True)
        self.ground_points = config.get('ground_points', 5000)
        
        # Minimum distance between obstacles
        self.min_obstacle_distance = config.get('min_obstacle_distance', 1.0)
        self.max_placement_attempts = config.get('max_placement_attempts', 100)
        
        self.scene = o3d.geometry.PointCloud()
        self.obstacles = []  # List of (center, radius) tuples for collision detection
        self.cylinders = []
        self.torus = []
    
    def check_collision(self, center, radius):
        """Check if a new obstacle collides with existing obstacles"""
        for obs_center, obs_radius in self.obstacles:
            # Calculate distance between centers (only in x-y plane for better distribution)
            distance = np.sqrt((center[0] - obs_center[0])**2 + (center[1] - obs_center[1])**2)
            # Check if distance is less than sum of radii plus minimum distance
            min_required_distance = radius + obs_radius + self.min_obstacle_distance
            if distance < min_required_distance:
                return True  # Collision detected
        return False  # No collision
    
    def generate_cylinder(self):
        """Generate a random cylinder obstacle with collision avoidance"""
        radius = np.random.uniform(*self.cyl_radius_range)
        height = np.random.uniform(*self.cyl_height_range)
        
        # Try to find a valid position
        for attempt in range(self.max_placement_attempts):
            # Random position within field
            x = np.random.uniform(-self.field_x/2, self.field_x/2)
            y = np.random.uniform(-self.field_y/2, self.field_y/2)
            z = height / 2  # Place cylinder on ground (z=0), center at height/2
            
            center = np.array([x, y, z])
            
            # Check collision with existing obstacles
            if not self.check_collision(center, radius):
                # Create cylinder
                cyl = o3d.geometry.TriangleMesh.create_cylinder(radius, height)
                cyl.translate([x, y, z])
                
                # Sample points from the mesh
                pcd_cyl = cyl.sample_points_uniformly(self.cyl_points)
                
                # Add to obstacles list for future collision checks
                self.obstacles.append((center, radius))
                self.cylinders.append({
                    'center': center.tolist(),
                    'radius': float(radius),
                    'height': float(height),
                })
                
                return pcd_cyl
        
        # If failed to place after max attempts, return None
        print(f"Warning: Failed to place cylinder after {self.max_placement_attempts} attempts")
        return None
    
    def generate_torus(self):
        """Generate a random torus (ring) obstacle with collision avoidance"""
        major_radius = np.random.uniform(*self.torus_major_radius_range)
        minor_radius = np.random.uniform(*self.torus_minor_radius_range)
        
        # Use major_radius as equivalent radius for collision detection
        equivalent_radius = major_radius
        
        # Try to find a valid position
        for attempt in range(self.max_placement_attempts):
            # Random position within field
            x = np.random.uniform(-self.field_x/2 + major_radius, self.field_x/2 - major_radius)
            y = np.random.uniform(-self.field_y/2 + major_radius, self.field_y/2 - major_radius)
            z = np.random.uniform(major_radius, self.field_z - major_radius)
            
            center = np.array([x, y, z])
            
            # Check collision with existing obstacles
            if not self.check_collision(center, equivalent_radius):
                # Create torus
                torus = o3d.geometry.TriangleMesh.create_torus(major_radius, minor_radius)
                
                # Random rotation
                rotation_xyz = [
                    np.random.uniform(0, 2*np.pi),
                    np.random.uniform(0, 2*np.pi),
                    np.random.uniform(0, 2*np.pi)
                ]
                R = torus.get_rotation_matrix_from_xyz(rotation_xyz)
                torus.rotate(R, center=(0, 0, 0))
                torus.translate([x, y, z])
                
                # Sample points from the mesh
                pcd_torus = torus.sample_points_uniformly(self.torus_points)
                
                # Add to obstacles list for future collision checks
                self.obstacles.append((center, equivalent_radius))
                self.torus.append({
                    'center': center.tolist(),
                    'major_radius': float(major_radius),
                    'minor_radius': float(minor_radius),
                    'rotation_xyz': [float(v) for v in rotation_xyz],
                })
                
                return pcd_torus
        
        # If failed to place after max attempts, return None
        print(f"Warning: Failed to place torus after {self.max_placement_attempts} attempts")
        return None

    def create_cylinder_from_config(self, cylinder_config):
        """Create a cylinder obstacle from saved configuration."""
        center = np.array(cylinder_config['center'], dtype=float)
        radius = float(cylinder_config['radius'])
        height = float(cylinder_config['height'])

        cyl = o3d.geometry.TriangleMesh.create_cylinder(radius, height)
        cyl.translate(center.tolist())
        self.obstacles.append((center, radius))
        self.cylinders.append({
            'center': center.tolist(),
            'radius': radius,
            'height': height,
        })
        return cyl.sample_points_uniformly(self.cyl_points)

    def create_torus_from_config(self, torus_config):
        """Create a torus obstacle from saved configuration."""
        center = np.array(torus_config['center'], dtype=float)
        major_radius = float(torus_config['major_radius'])
        minor_radius = float(torus_config['minor_radius'])
        rotation_xyz = torus_config.get('rotation_xyz', [0.0, 0.0, 0.0])

        torus = o3d.geometry.TriangleMesh.create_torus(major_radius, minor_radius)
        R = torus.get_rotation_matrix_from_xyz(rotation_xyz)
        torus.rotate(R, center=(0, 0, 0))
        torus.translate(center.tolist())
        self.obstacles.append((center, major_radius))
        self.torus.append({
            'center': center.tolist(),
            'major_radius': major_radius,
            'minor_radius': minor_radius,
            'rotation_xyz': [float(v) for v in rotation_xyz],
        })
        return torus.sample_points_uniformly(self.torus_points)

    def load_obstacle_config(self, obstacle_config_file):
        """Load cylinder and torus layout from a YAML file."""
        with open(obstacle_config_file, 'r') as f:
            obstacle_config = yaml.safe_load(f) or {}
        cylinders = obstacle_config.get('cylinders', [])
        torus_obstacles = obstacle_config.get('torus', obstacle_config.get('rings', []))
        return cylinders, torus_obstacles

    def save_obstacle_config(self, obstacle_config_file):
        """Save the generated cylinder and torus layout to a YAML file."""
        output_dir = os.path.dirname(obstacle_config_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        obstacle_config = {
            'cylinders': self.cylinders,
            'torus': self.torus,
        }
        with open(obstacle_config_file, 'w') as f:
            yaml.safe_dump(obstacle_config, f, default_flow_style=False, sort_keys=False)

        print(f"Obstacle layout saved to: {os.path.abspath(obstacle_config_file)}")
    
    def generate_ground(self):
        """Generate ground plane"""
        ground = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=self.field_y,
            depth=0.1
        )
        ground.translate([-self.field_x/2, -self.field_y/2, -0.1])
        pcd_ground = ground.sample_points_uniformly(self.ground_points)
        return pcd_ground
    
    def generate_boundaries(self):
        """Generate boundary walls around the field"""
        boundaries = o3d.geometry.PointCloud()
        
        # Create 4 walls
        wall_thickness = 0.1
        
        # Wall 1: x = -field_x/2
        wall1 = o3d.geometry.TriangleMesh.create_box(
            width=wall_thickness,
            height=self.field_y,
            depth=self.field_z
        )
        wall1.translate([-self.field_x/2, -self.field_y/2, 0])
        boundaries += wall1.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 2: x = field_x/2
        wall2 = o3d.geometry.TriangleMesh.create_box(
            width=wall_thickness,
            height=self.field_y,
            depth=self.field_z
        )
        wall2.translate([self.field_x/2 - wall_thickness, -self.field_y/2, 0])
        boundaries += wall2.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 3: y = -field_y/2
        wall3 = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=wall_thickness,
            depth=self.field_z
        )
        wall3.translate([-self.field_x/2, -self.field_y/2, 0])
        boundaries += wall3.sample_points_uniformly(self.boundary_points // 4)
        
        # Wall 4: y = field_y/2
        wall4 = o3d.geometry.TriangleMesh.create_box(
            width=self.field_x,
            height=wall_thickness,
            depth=self.field_z
        )
        wall4.translate([-self.field_x/2, self.field_y/2 - wall_thickness, 0])
        boundaries += wall4.sample_points_uniformly(self.boundary_points // 4)
        
        return boundaries
    
    def generate_map(self):
        """Generate the complete map with all obstacles"""
        print(f"Generating map with seed: {self.seed}")
        print(f"Field dimensions: {self.field_x} x {self.field_y} x {self.field_z}")
        print(f"Minimum obstacle distance: {self.min_obstacle_distance} m")
        
        # Clear generated geometry and obstacle metadata
        self.scene = o3d.geometry.PointCloud()
        self.obstacles = []
        self.cylinders = []
        self.torus = []

        if self.obstacle_config_file:
            cylinders, torus_obstacles = self.load_obstacle_config(self.obstacle_config_file)
            print(f"Loading obstacle layout from: {self.obstacle_config_file}")

            for cylinder_config in cylinders:
                self.scene += self.create_cylinder_from_config(cylinder_config)
            print(f"Loaded {len(cylinders)} cylinders")

            for torus_config in torus_obstacles:
                self.scene += self.create_torus_from_config(torus_config)
            print(f"Loaded {len(torus_obstacles)} torus obstacles")
        else:
            # Generate cylinders
            print(f"Generating {self.num_cylinders} cylinders...")
            cylinders_placed = 0
            for i in range(self.num_cylinders):
                pcd_cyl = self.generate_cylinder()
                if pcd_cyl is not None:
                    self.scene += pcd_cyl
                    cylinders_placed += 1
            print(f"Successfully placed {cylinders_placed}/{self.num_cylinders} cylinders")
            
            # Generate torus obstacles
            print(f"Generating {self.num_torus} torus obstacles...")
            torus_placed = 0
            for i in range(self.num_torus):
                pcd_torus = self.generate_torus()
                if pcd_torus is not None:
                    self.scene += pcd_torus
                    torus_placed += 1
            print(f"Successfully placed {torus_placed}/{self.num_torus} torus obstacles")

            if self.save_obstacle_config_file:
                self.save_obstacle_config(self.save_obstacle_config_file)
        
        # Add ground if enabled
        if self.add_ground:
            print("Adding ground plane...")
            pcd_ground = self.generate_ground()
            self.scene += pcd_ground
        
        # Add boundaries if enabled
        if self.add_boundaries:
            print("Adding boundary walls...")
            boundaries = self.generate_boundaries()
            self.scene += boundaries
        
        print(f"Total points in scene: {len(self.scene.points)}")
        
        return self.scene
    
    def save_map(self):
        """Save the generated map to a PCD file"""
        output_path = self.output_file
        
        # Create directory if it doesn't exist
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # Handle empty point cloud case
        if not self.scene.has_points():
            print("Warning: Point cloud is empty. Creating minimal point cloud...")
            # Add a single point at origin to create a valid (nearly empty) PCD file
            self.scene.points = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, -2.0]]))
        
        # Save point cloud
        success = o3d.io.write_point_cloud(output_path, self.scene)
        
        if success:
            abs_path = os.path.abspath(output_path)
            print(f"Map saved successfully to: {abs_path}")
        else:
            print(f"Failed to save map to: {output_path}")
        
        return success
    
    def visualize(self):
        """Visualize the generated map"""
        print("Visualizing map...")
        o3d.visualization.draw_geometries([self.scene],
                                          window_name="Generated Map",
                                          width=1024,
                                          height=768)


def load_config(config_file):
    """Load configuration from YAML file"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config


def main():
    parser = argparse.ArgumentParser(description='Generate random 3D maps with obstacles')
    parser.add_argument('--config', type=str, help='Path to configuration YAML file')
    parser.add_argument('--seed', type=int, help='Random seed')
    parser.add_argument('--output', type=str, help='Output PCD file path')
    parser.add_argument('--visualize', action='store_true', help='Visualize the generated map')
    parser.add_argument('--field-x', type=float, help='Field width (x dimension)')
    parser.add_argument('--field-y', type=float, help='Field length (y dimension)')
    parser.add_argument('--field-z', type=float, help='Field height (z dimension)')
    parser.add_argument('--num-cylinders', type=int, help='Number of cylinders')
    parser.add_argument('--num-torus', type=int, help='Number of torus obstacles')
    parser.add_argument(
        '--obstacle-config',
        type=str,
        help='Path to a YAML file with saved cylinder and torus positions. If set, random obstacle generation is skipped.'
    )
    parser.add_argument(
        '--save-obstacle-config',
        type=str,
        help='Path to save the generated cylinder and torus positions as YAML'
    )
    
    args = parser.parse_args()
    
    # Load configuration
    config_file = None
    
    if args.config:
        # Use user-specified config file
        config_file = args.config
    else:
        # Try to load default config file from script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_config_file = os.path.join(script_dir, 'map_generator_config.yaml')
        if os.path.exists(default_config_file):
            config_file = default_config_file
            print(f"Using default config file: {config_file}")
    
    if config_file and os.path.exists(config_file):
        config = load_config(config_file)
    else:
        # Default configuration (fallback if no config file found)
        print("No config file found, using default configuration")
        config = {
            'seed': 42,
            'field_x': 20.0,
            'field_y': 20.0,
            'field_z': 5.0,
            'num_cylinders': 10,
            'num_torus': 5,
            'cyl_radius_range': [0.2, 0.5],
            'cyl_height_range': [1.0, 3.0],
            'cyl_points': 3000,
            'torus_major_radius_range': [0.8, 1.5],
            'torus_minor_radius_range': [0.15, 0.3],
            'torus_points': 3000,
            'add_boundaries': False,
            'boundary_points': 10000,
            'add_ground': True,
            'ground_points': 5000,
            'min_obstacle_distance': 1.0,
            'max_placement_attempts': 100,
            'output_file': 'random_map.pcd',
            'obstacle_config_file': None,
            'save_obstacle_config_file': None
        }
    
    # Override with command line arguments
    if args.seed is not None:
        config['seed'] = args.seed
    if args.output is not None:
        config['output_file'] = args.output
    if args.field_x is not None:
        config['field_x'] = args.field_x
    if args.field_y is not None:
        config['field_y'] = args.field_y
    if args.field_z is not None:
        config['field_z'] = args.field_z
    if args.num_cylinders is not None:
        config['num_cylinders'] = args.num_cylinders
    if args.num_torus is not None:
        config['num_torus'] = args.num_torus
    if args.obstacle_config is not None:
        config['obstacle_config_file'] = args.obstacle_config
    if args.save_obstacle_config is not None:
        config['save_obstacle_config_file'] = args.save_obstacle_config
    
    # Create map generator
    generator = MapGenerator(config)
    
    # Generate map
    generator.generate_map()
    
    # Save map
    generator.save_map()

    print("Using built-in visualization...")
    generator.visualize()
    
    # Visualize pcd if requested
    if args.visualize:
        import subprocess
        output_file = generator.output_file
        output_dir = os.path.dirname(os.path.abspath(output_file))
        output_filename = os.path.basename(output_file)
        vis_script = os.path.join(output_dir, 'vis_pcd.py')
        
        if os.path.exists(vis_script):
            print(f"\nLaunching visualization with {vis_script}...")
            subprocess.run(['python3', vis_script, output_filename], cwd=output_dir)
        else:
            print(f"\nWarning: vis_pcd.py not found at {vis_script}")


if __name__ == '__main__':
    main()
