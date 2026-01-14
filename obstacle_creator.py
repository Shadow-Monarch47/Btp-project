import numpy as np
import json
import os
import math
import random

class ObstacleMapManager:
    """
    Manages obstacle maps for multi-robot path planning.
    Provides methods to create, load, save, and retrieve obstacle configurations.
    """
    
    def __init__(self, grid_size=55, save_file=r"yielding_robot_obstacle_maps.json"):
        """
        Initialize the obstacle map manager.
        
        Args:
            grid_size (int): Size of the grid (assumes square grid)
            save_file (str): Path to JSON file for saving/loading maps
        """
        self.GRID_SIZE = grid_size
        self.obstacles = set()
        self.protected_positions = set()  # Positions that cannot have obstacles
        self.save_file = save_file
        
        # Store different map configurations
        self.saved_maps = {
            'simple': set(),
            'complex': set(),
            'cross': set(),
            'diamond': set(),
            'zigzag': set(),
            'current': 'simple'  # Track current map type
        }
        
        # Load saved maps from file
        self._load_maps_from_file()
    
    def add_protected_position(self, x, y):
        """Add a position that cannot have obstacles (e.g., robot start/goal)"""
        self.protected_positions.add((x, y))
    
    def remove_protected_position(self, x, y):
        """Remove a protected position"""
        self.protected_positions.discard((x, y))
    
    def add_obstacle(self, x, y):
        """
        Add a single obstacle at position (x, y).
        
        Returns:
            bool: True if obstacle was added, False otherwise
        """
        if (x, y) not in self.protected_positions and 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
            self.obstacles.add((x, y))
            self._update_current_saved_map()
            return True
        return False
    
    def remove_obstacle(self, x, y):
        """
        Remove obstacle at position (x, y).
        
        Returns:
            bool: True if obstacle was removed, False otherwise
        """
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))
            self._update_current_saved_map()
            return True
        return False
    
    def toggle_obstacle(self, x, y):
        """Toggle obstacle at position (x, y)"""
        if (x, y) in self.obstacles:
            return self.remove_obstacle(x, y)
        else:
            return self.add_obstacle(x, y)
    
    def clear_all_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()
        self._update_current_saved_map()
    
    def get_obstacles(self):
        """
        Get current obstacles set.
        
        Returns:
            set: Copy of current obstacles as (x, y) tuples
        """
        return self.obstacles.copy()
    
    def get_obstacle_map(self):
        """
        Get obstacle map as a binary numpy array.
        
        Returns:
            np.ndarray: Binary grid where 1 = obstacle, 0 = free space
        """
        obstacle_map = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        for x, y in self.obstacles:
            if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                obstacle_map[y, x] = 1  # Note: row=y, col=x
        return obstacle_map
    
    def get_grid_size(self):
        """Get the grid size"""
        return self.GRID_SIZE
    
    def get_protected_positions(self):
        """Get set of protected positions"""
        return self.protected_positions.copy()
    
    def _update_current_saved_map(self):
        """Update the saved map for the current mode"""
        current_map = self.saved_maps['current']
        if current_map in self.saved_maps:
            self.saved_maps[current_map] = self.obstacles.copy()
            self._save_maps_to_file()
    
    def _load_saved_map(self, map_type):
        """Load obstacles from saved map type"""
        if map_type in self.saved_maps:
            self.obstacles = self.saved_maps[map_type].copy()
            self.saved_maps['current'] = map_type
            print(f"Loaded saved {map_type} map with {len(self.obstacles)} obstacles")
        else:
            print(f"Map type '{map_type}' not found")
    
    def _save_current_map_as(self, map_type):
        """Save current obstacles as a specific map type"""
        if map_type != 'current':
            self.saved_maps[map_type] = self.obstacles.copy()
            self.saved_maps['current'] = map_type
            self._save_maps_to_file()
            print(f"Saved current map as '{map_type}' with {len(self.obstacles)} obstacles")
    
    def _save_maps_to_file(self):
        """Save all maps to JSON file"""
        try:
            # Convert sets to lists for JSON serialization
            data_to_save = {}
            for key, value in self.saved_maps.items():
                if key == 'current':
                    data_to_save[key] = value
                else:
                    data_to_save[key] = list(value) if isinstance(value, set) else value
            
            with open(self.save_file, 'w') as f:
                json.dump(data_to_save, f, indent=2)
        except Exception as e:
            print(f"Error saving maps to file: {e}")
    
    def _load_maps_from_file(self):
        """Load all maps from JSON file"""
        try:
            if os.path.exists(self.save_file):
                with open(self.save_file, 'r') as f:
                    data = json.load(f)
                
                # Convert lists back to sets
                for key, value in data.items():
                    if key == 'current':
                        self.saved_maps[key] = value
                    else:
                        self.saved_maps[key] = set(tuple(pos) for pos in value) if isinstance(value, list) else set()
                
                # Load the current map
                current_map = self.saved_maps.get('current', 'simple')
                if current_map in self.saved_maps and isinstance(self.saved_maps[current_map], set):
                    self.obstacles = self.saved_maps[current_map].copy()
                
                print(f"Maps loaded from {self.save_file}")
                print(f"Current mode: {current_map} with {len(self.obstacles)} obstacles")
            else:
                print(f"No save file found. Starting with empty maps.")
        except Exception as e:
            print(f"Error loading maps from file: {e}")
            print("Starting with empty maps.")
    
    def load_obstacles(self, obstacle_list):
        """
        Load obstacles from a list of (x, y) tuples.
        
        Args:
            obstacle_list (list): List of (x, y) coordinate tuples
        """
        self.obstacles.clear()
        for x, y in obstacle_list:
            self.add_obstacle(x, y)
    
    def create_simple_maze(self):
        """Create a simple horizontal barrier maze or load if exists"""
        if len(self.saved_maps['simple']) > 0:
            self._load_saved_map('simple')
        else:
            self.obstacles.clear()
            for x in range(self.GRID_SIZE):
                if 25 <= x <= 29:  # Leave gap in middle
                    continue
                for y in range(25, 30):
                    self.obstacles.add((x, y))
            self._save_current_map_as('simple')
    
    def create_complex_maze(self):
        """Create a complex maze with multiple patterns or load if exists"""
        if len(self.saved_maps['complex']) > 0:
            self._load_saved_map('complex')
        else:
            self.obstacles = set()
            
            # Create outer walls (with gaps)
            for x in range(self.GRID_SIZE):
                if not (x >= 25 and x <= 29):  # Leave gap in middle
                    if (x, 0) not in self.protected_positions:
                        self.obstacles.add((x, 0))
                    if (x, self.GRID_SIZE-1) not in self.protected_positions:
                        self.obstacles.add((x, self.GRID_SIZE-1))
            
            # Left and right walls
            for y in range(self.GRID_SIZE):
                if not (y >= 25 and y <= 29):  # Leave gap in middle
                    if (0, y) not in self.protected_positions:
                        self.obstacles.add((0, y))
                    if (self.GRID_SIZE-1, y) not in self.protected_positions:
                        self.obstacles.add((self.GRID_SIZE-1, y))
            
            # Create spiral pattern in center
            center_x, center_y = self.GRID_SIZE // 2, self.GRID_SIZE // 2
            for radius in range(5, 20, 3):
                for angle in range(0, 360, 10):
                    x = int(center_x + radius * math.cos(math.radians(angle)))
                    y = int(center_y + radius * math.sin(math.radians(angle)))
                    if 5 <= x < self.GRID_SIZE-5 and 5 <= y < self.GRID_SIZE-5:
                        self.obstacles.add((x, y))
            
            # Create maze walls with pathways
            for x in [10, 20, 35, 45]:
                for y in range(5, self.GRID_SIZE-5):
                    if y % 8 != 3 and y % 8 != 4:  # Leave gaps every 8 cells
                        self.obstacles.add((x, y))
            
            for y in [10, 20, 35, 45]:
                for x in range(5, self.GRID_SIZE-5):
                    if x % 8 != 3 and x % 8 != 4:  # Leave gaps every 8 cells
                        self.obstacles.add((x, y))
            
            # Add some random scattered obstacles
            random.seed(42)
            for _ in range(50):
                x = random.randint(3, self.GRID_SIZE-4)
                y = random.randint(3, self.GRID_SIZE-4)
                if (x, y) not in self.protected_positions:
                    self.obstacles.add((x, y))
            
            # Ensure protected positions are clear
            for pos in self.protected_positions:
                self.obstacles.discard(pos)
            
            self._save_current_map_as('complex')
    
    def create_custom_pattern(self, pattern_type="cross"):
        """
        Create custom obstacle patterns or load if exists.
        
        Args:
            pattern_type (str): Type of pattern ('cross', 'diamond', 'zigzag')
        """
        if len(self.saved_maps[pattern_type]) > 0:
            self._load_saved_map(pattern_type)
        else:
            self.obstacles.clear()
            
            if pattern_type == "cross":
                # Create cross pattern
                center_x, center_y = self.GRID_SIZE // 2, self.GRID_SIZE // 2
                for i in range(-10, 11):
                    if abs(i) > 2:  # Leave center gap
                        self.obstacles.add((center_x + i, center_y))
                        self.obstacles.add((center_x, center_y + i))
            
            elif pattern_type == "diamond":
                # Create diamond patterns
                center_x, center_y = self.GRID_SIZE // 2, self.GRID_SIZE // 2
                for size in [5, 10, 15]:
                    for i in range(-size, size + 1):
                        for j in range(-size, size + 1):
                            if abs(i) + abs(j) == size:
                                x, y = center_x + i, center_y + j
                                if 0 <= x < self.GRID_SIZE and 0 <= y < self.GRID_SIZE:
                                    self.obstacles.add((x, y))
            
            elif pattern_type == "zigzag":
                # Create zigzag pattern
                for x in range(5, self.GRID_SIZE - 5, 2):
                    y_offset = (x % 8) - 4
                    y_center = self.GRID_SIZE // 2
                    if 0 <= y_center + y_offset < self.GRID_SIZE:
                        self.obstacles.add((x, y_center + y_offset))
            
            # Remove obstacles from protected positions
            for pos in self.protected_positions:
                self.obstacles.discard(pos)
            
            self._save_current_map_as(pattern_type)
    
    def export_obstacles(self, filename=None):
        """
        Export current obstacles to a list or file.
        
        Args:
            filename (str, optional): If provided, save to file
            
        Returns:
            list: List of (x, y) obstacle coordinates
        """
        obstacle_list = list(self.obstacles)
        if filename:
            try:
                with open(filename, 'w') as f:
                    for x, y in obstacle_list:
                        f.write(f"{x},{y}\n")
                print(f"Current obstacles exported to {filename}")
            except Exception as e:
                print(f"Error exporting obstacles: {e}")
        return obstacle_list
    
    def import_obstacles(self, filename):
        """
        Import obstacles from a file.
        
        Args:
            filename (str): Path to file containing obstacle coordinates
        """
        try:
            self.obstacles.clear()
            with open(filename, 'r') as f:
                for line in f:
                    x, y = map(int, line.strip().split(','))
                    self.add_obstacle(x, y)
            print(f"Obstacles imported from {filename}")
        except Exception as e:
            print(f"Error importing obstacles: {e}")
    
    def get_save_file_info(self):
        """
        Get information about the save file.
        
        Returns:
            dict: File information including existence, size, and modification time
        """
        if os.path.exists(self.save_file):
            file_size = os.path.getsize(self.save_file)
            mod_time = os.path.getmtime(self.save_file)
            from datetime import datetime
            mod_time_str = datetime.fromtimestamp(mod_time).strftime('%Y-%m-%d %H:%M:%S')
            return {
                'exists': True,
                'size': file_size,
                'modified': mod_time_str,
                'path': os.path.abspath(self.save_file)
            }
        else:
            return {'exists': False, 'path': os.path.abspath(self.save_file)}
    
    def get_current_map_type(self):
        """Get the current map type name"""
        return self.saved_maps['current']


# Convenience function for external modules
def get_obstacle_map(manager=None):
    """
    Get obstacle map from a manager instance or create a new one.
    
    Args:
        manager (ObstacleMapManager, optional): Existing manager instance
        
    Returns:
        tuple: (obstacles_set, grid_size, obstacle_map_array)
    """
    if manager is None:
        manager = ObstacleMapManager()
    
    return manager.get_obstacles(), manager.get_grid_size(), manager.get_obstacle_map()


# Example usage
if __name__ == "__main__":
    # Create obstacle manager
    manager = ObstacleMapManager(grid_size=55)
    
    # Add some protected positions (e.g., robot start/goal positions)
    manager.add_protected_position(0, 0)
    manager.add_protected_position(54, 54)
    manager.add_protected_position(0, 54)
    manager.add_protected_position(54, 0)
    
    # Create a complex maze
    manager.create_complex_maze()
    
    # Display information
    print(f"Grid size: {manager.get_grid_size()}")
    print(f"Number of obstacles: {len(manager.get_obstacles())}")
    print(f"Current map type: {manager.get_current_map_type()}")
    
    # Get obstacle map as array
    obstacle_array = manager.get_obstacle_map()
    print(f"Obstacle map shape: {obstacle_array.shape}")
    
    # Display save file info
    file_info = manager.get_save_file_info()
    if file_info['exists']:
        print(f"Save file: {file_info['modified']}")
    else:
        print("Save file: Not found")