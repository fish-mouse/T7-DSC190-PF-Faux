# particle_filter/create_test_map.py

import numpy as np
import pickle

def create_test_map():
    """
    Create map sized for your dataset:
    - Robot moves in X: 0.2 to 1.12m
    - Robot moves in Y: 0.056 to 1.013m
    - Map covers: -0.5 to 3.5m in both X and Y (4m × 4m)
    """
    
    # Map parameters
    resolution = 0.05  # 5cm per cell
    width = 80   # 80 cells × 0.05m = 4.0m
    height = 80  # 80 cells × 0.05m = 4.0m
    
    # Origin (bottom-left corner of map in world coordinates)
    origin_x = -0.5
    origin_y = -0.5
    
    print(f"Creating map:")
    print(f"  Size: {width}×{height} cells")
    print(f"  Resolution: {resolution}m/cell")
    print(f"  World bounds: X=[{origin_x}, {origin_x + width*resolution}], Y=[{origin_y}, {origin_y + height*resolution}]")
    print(f"  Robot bounds: X=[0.2, 1.12], Y=[0.056, 1.013]")
    
    # Create empty map (0 = free space, 100 = occupied)
    map_data = np.zeros((height, width), dtype=np.int8)
    
    # Add boundary walls
    map_data[0, :] = 100      # Bottom wall
    map_data[-1, :] = 100     # Top wall
    map_data[:, 0] = 100      # Left wall
    map_data[:, -1] = 100     # Right wall
    
    # Add interior wall at x=1.2m (just past robot's path)
    # x=1.2m → cell = (1.2 - (-0.5)) / 0.05 = 34
    wall_x = int((1.2 - origin_x) / resolution)
    if wall_x < width:
        map_data[10:70, wall_x] = 100  # Vertical wall
        print(f"  Added wall at x=1.2m (cell {wall_x})")
    
    # Flatten to 1D list (row-major order)
    map_1d = map_data.flatten().tolist()
    
    # Save map info
    map_info = {
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'origin_theta': 0.0,
        'data': map_1d
    }
    
    with open('test_map.pkl', 'wb') as f:
        pickle.dump(map_info, f)
    
    print("✓ Saved to test_map.pkl")
    
    # Verify robot trajectory fits in map
    robot_x_min, robot_x_max = 0.2, 1.12
    robot_y_min, robot_y_max = 0.056, 1.013
    
    cell_x_min = int((robot_x_min - origin_x) / resolution)
    cell_x_max = int((robot_x_max - origin_x) / resolution)
    cell_y_min = int((robot_y_min - origin_y) / resolution)
    cell_y_max = int((robot_y_max - origin_y) / resolution)
    
    print(f"\nRobot trajectory occupies cells:")
    print(f"  X: {cell_x_min} to {cell_x_max} (out of {width})")
    print(f"  Y: {cell_y_min} to {cell_y_max} (out of {height})")
    
    if 0 < cell_x_min < width and 0 < cell_x_max < width and \
       0 < cell_y_min < height and 0 < cell_y_max < height:
        print("✓ Robot trajectory fits within map bounds")
    else:
        print("✗ WARNING: Robot trajectory outside map!")
    
    return map_info

if __name__ == "__main__":
    create_test_map()