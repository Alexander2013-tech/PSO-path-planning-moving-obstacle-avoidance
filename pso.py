import numpy as np
from scipy.interpolate import splprep, splev
from obstacles import ObstacleManager

class PSOPathPlanner:
    def __init__(self, start, end, n_obstacles, max_iterations, bounds_x, bounds_y):
        self.start = start
        self.end = end
        self.n_obstacles = n_obstacles
        self.max_iterations = max_iterations
        self.space_bounds_x = bounds_x
        self.space_bounds_y = bounds_y
        
        # PSO parameters
        self.r_min, self.r_max = 5, 15
        self.n_waypoints = 4
        self.n_particles = 100
        self.w, self.c1, self.c2 = 0.5, 1.5, 1.5
        
        # Initialize obstacles
        self.obstacle_manager = ObstacleManager(n_obstacles, self.r_min, self.r_max, start, end)
        self.obstacles, self.radii = self.obstacle_manager.generate_obstacles()
    
    def run(self):
        dim = self.n_waypoints * 2
        positions = np.zeros((self.n_particles, dim))
        for i in range(self.n_particles):
            for j in range(self.n_waypoints):
                positions[i, j * 2] = np.random.uniform(self.space_bounds_x[0], self.space_bounds_x[1])
                positions[i, j * 2 + 1] = np.random.uniform(self.space_bounds_y[0], self.space_bounds_y[1])

        velocities = np.random.uniform(-1, 1, (self.n_particles, dim))
        p_best_pos = positions.copy()
        p_best_cost = np.full(self.n_particles, np.inf)
        g_best_pos = None
        g_best_cost = np.inf
        g_best_iter = -1
        convergence = []

        for iteration in range(self.max_iterations):
            for i in range(self.n_particles):
                waypoints = positions[i].reshape(self.n_waypoints, 2)
                x_spline, y_spline = self._spline_path(waypoints)
                if x_spline is None:
                    continue
                cost = self._path_cost(x_spline, y_spline)
                if cost < p_best_cost[i]:
                    p_best_cost[i] = cost
                    p_best_pos[i] = positions[i].copy()
                if cost < g_best_cost:
                    g_best_cost = cost
                    g_best_pos = positions[i].copy()
                    g_best_iter = iteration

            convergence.append(g_best_cost)
            print(f"Iteration {iteration+1}/{self.max_iterations}, Best Cost: {g_best_cost:.4f}")

            for i in range(self.n_particles):
                r1, r2 = np.random.rand(2)
                velocities[i] = (self.w * velocities[i] +
                                 self.c1 * r1 * (p_best_pos[i] - positions[i]) +
                                 self.c2 * r2 * (g_best_pos - positions[i]))
                positions[i] += velocities[i]
                for j in range(self.n_waypoints):
                    positions[i, j * 2] = np.clip(positions[i, j * 2], self.space_bounds_x[0], self.space_bounds_x[1])
                    positions[i, j * 2 + 1] = np.clip(positions[i, j * 2 + 1], self.space_bounds_y[0], self.space_bounds_y[1])

        if g_best_pos is not None:
            best_waypoints = g_best_pos.reshape(self.n_waypoints, 2)
            return self._spline_path(best_waypoints), g_best_cost, g_best_iter, convergence
        return None, np.inf, -1, convergence

    def _spline_path(self, waypoints):
        points = np.vstack([self.start, waypoints, self.end]).T
        try:
            tck, _ = splprep(points, s=2.0)
            u = np.linspace(0, 1, 100)
            return splev(u, tck)
        except:
            return None, None

    def _path_cost(self, x_spline, y_spline):
        if not self._is_valid_path(x_spline, y_spline):
            return 1e6
        dx, dy = np.diff(x_spline), np.diff(y_spline)
        return np.sum(np.sqrt(dx**2 + dy**2))

    def _is_valid_path(self, x_spline, y_spline, dynamic=True):
        for i, (x, y) in enumerate(zip(x_spline, y_spline)):
            if self.obstacle_manager.check_collision(x, y, i):
                return False
        return True