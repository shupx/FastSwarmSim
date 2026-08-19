# Map Generator 使用说明

这个工具可以生成随机的3D点云地图，包含圆柱体和圆环障碍物，用于机器人仿真。

## 功能特点

- ✅ 生成随机圆柱体障碍物
- ✅ 生成随机圆环(torus)障碍物
- ✅ 可配置场地长宽高
- ✅ 使用随机种子确保可重复性
- ✅ 导出为PCD点云文件
- ✅ 可导出/导入圆柱体和圆环布局
- ✅ 可选添加地面和边界墙
- ✅ 支持配置文件和命令行参数

## 安装依赖

```bash
pip3 install open3d numpy pyyaml
```

## 使用方法

### 方法1：使用配置文件

```bash
cd src/local_sensing_sim/src/map_generator
python3 map_generator.py --config ./map_generator_config.yaml
```

默认使用`map_generator_config.yaml`文件：

```bash
python3 map_generator.py
```

### 方法2：使用命令行参数

```bash
python3 map_generator.py \
  --seed 123 \
  --field-x 30.0 \
  --field-y 30.0 \
  --field-z 6.0 \
  --num-cylinders 15 \
  --num-torus 8 \
  --output ../pcd/my_map.pcd
```

### 方法3：生成并可视化

```bash
python3 map_generator.py --config ./map_generator_config.yaml --visualize
```

### 方法4：保存和复用障碍物布局

随机生成地图，并把圆柱体/圆环的位置和尺寸保存为 YAML：

```bash
python3 map_generator.py \
  --config ./map_generator_config.yaml \
  --save-obstacle-config ./obstacle_layout.yaml
```

从保存的布局生成地图，此时不会随机生成障碍物：

```bash
python3 map_generator.py \
  --config ./map_generator_config.yaml \
  --obstacle-config ./obstacle_layout.yaml \
  --output ../../pcd/reused_map.pcd
```

## 配置参数说明

### 基本参数

- `seed`: 随机种子，用于生成可重复的地图
- `field_x`: 场地宽度 (米)
- `field_y`: 场地长度 (米)
- `field_z`: 场地高度 (米)

### 障碍物数量

- `num_cylinders`: 圆柱体数量
- `num_torus`: 圆环数量

### 圆柱体参数

- `cyl_radius_range`: 圆柱体半径范围 [最小值, 最大值]
- `cyl_height_range`: 圆柱体高度范围 [最小值, 最大值]
- `cyl_points`: 每个圆柱体采样的点数

### 圆环参数

- `torus_major_radius_range`: 圆环大半径范围 [最小值, 最大值]
- `torus_minor_radius_range`: 圆环小半径范围 [最小值, 最大值]
- `torus_points`: 每个圆环采样的点数

### 可选功能

- `add_boundaries`: 是否添加边界墙 (true/false)
- `add_ground`: 是否添加地面 (true/false)
- `output_file`: 输出PCD文件路径
- `obstacle_config_file`: 障碍物布局YAML路径；为空时默认随机生成
- `save_obstacle_config_file`: 随机生成后保存障碍物布局YAML的路径

障碍物布局文件示例：

```yaml
cylinders:
  - center: [1.0, 2.0, 1.0]
    radius: 0.4
    height: 2.0
torus:
  - center: [4.0, 2.0, 2.5]
    major_radius: 1.0
    minor_radius: 0.15
    rotation_xyz: [0.0, 1.57, 0.0]
```

## 示例

### 生成小型测试场地

```bash
python3 map_generator.py \
  --seed 42 \
  --field-x 10.0 \
  --field-y 10.0 \
  --field-z 3.0 \
  --num-cylinders 5 \
  --num-torus 3 \
  --output ../pcd/test_map.pcd
```

### 生成大型复杂场地

```bash
python3 map_generator.py \
  --seed 100 \
  --field-x 50.0 \
  --field-y 50.0 \
  --field-z 10.0 \
  --num-cylinders 30 \
  --num-torus 15 \
  --output ../pcd/large_map.pcd
```

### 生成多个不同的地图

```bash
# 地图1
python3 map_generator.py --seed 1 --output ../pcd/map_001.pcd

# 地图2
python3 map_generator.py --seed 2 --output ../pcd/map_002.pcd

# 地图3
python3 map_generator.py --seed 3 --output ../pcd/map_003.pcd
```

## 输出格式

生成的文件为标准PCD格式点云，可以用以下工具查看：
- Open3D
- PCL Viewer
- CloudCompare
- RViz (ROS)

## Python API使用

你也可以在Python代码中直接使用：

```python
from map_generator import MapGenerator

config = {
    'seed': 42,
    'field_x': 20.0,
    'field_y': 20.0,
    'field_z': 5.0,
    'num_cylinders': 10,
    'num_torus': 5,
    'output_file': 'my_map.pcd'
}

generator = MapGenerator(config)
generator.generate_map()
generator.save_map()
generator.visualize()  # 可选：可视化
```

## 注意事项

1. 确保场地尺寸足够大，能容纳所有障碍物
2. 圆柱体和圆环会被自动放置在场地范围内
3. 使用相同的随机种子可以生成完全相同的地图
4. 点数越多，点云越密集，但文件也越大
