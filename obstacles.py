import numpy as np

class ObstacleManager:
    def __init__(self, num_obstacles, r_min, r_max, start, end):
        self.num_obstacles = num_obstacles
        self.r_min = r_min
        self.r_max = r_max
        self.start = start
        self.end = end
        self.centers = []
        self.radii = []
    
    def generate_obstacles(self):
        direction = self.end - self.start
        length = np.linalg.norm(direction)
        unit_dir = direction / length

        for i in range(self.num_obstacles):
            t = np.random.uniform(0.3, 0.7)
            point_on_line = self.start + t * direction
            perp_dir = np.array([-unit_dir[1], unit_dir[0]])
            offset = np.random.uniform(-10, 10)
            candidate = point_on_line + offset * perp_dir

            radius = np.random.uniform(self.r_min, self.r_max)
            if all(np.linalg.norm(candidate - c) > (radius + r + 2) for c, r in zip(self.centers, self.radii)):
                self.centers.append(candidate)
                self.radii.append(radius)

        return self.centers, self.radii
    
    def check_collision(self, x, y, frame):
        for idx, (cx, cy) in enumerate(self.centers):
            cy += 15 * np.sin(0.05 * frame + idx)
            if np.linalg.norm([x - cx, y - cy]) < self.radii[idx] + 1.5:
                return True
        return False