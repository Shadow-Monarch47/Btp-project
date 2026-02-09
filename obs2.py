import json
import os


class ObstacleMapManager:
    def __init__(self, grid_size=None, save_file="maps5.json"):
        self.save_file = save_file
        self.grid_size = grid_size
        self.obstacles = []

        self._load_map()

    def _load_map(self):
        if not os.path.exists(self.save_file):
            raise FileNotFoundError(f"Map file not found: {self.save_file}")

        with open(self.save_file, "r") as f:
            data = json.load(f)

        # Basic validation
        if "obstacles" not in data:
            raise ValueError("Invalid map file: 'obstacles' key missing")

        if self.grid_size is None:
            self.grid_size = data.get("grid_size")

        self.obstacles = [
            tuple(pos) for pos in data["obstacles"]
        ]

    def get_obstacles(self):
        """
        Returns list of (x, y) obstacle coordinates
        """
        return self.obstacles

    def get_grid_size(self):
        return self.grid_size
