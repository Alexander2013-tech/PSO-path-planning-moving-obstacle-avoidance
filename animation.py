import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import numpy as np
import tkinter as tk
from tkinter import messagebox
from pso import PSOPathPlanner

class PathAnimator:
    def __init__(self, canvas_frame, start, end, x_spline, y_spline, obstacles, radii, best_cost, best_iteration):
        self.canvas_frame = canvas_frame
        self.start = start
        self.end = end
        self.x_spline = x_spline
        self.y_spline = y_spline
        self.obstacles = obstacles
        self.radii = radii
        self.best_cost = best_cost
        self.best_iteration = best_iteration
        
        # Clear previous widgets
        for widget in self.canvas_frame.winfo_children():
            widget.destroy()
    
    def animate(self):
        self.fig, self.ax = plt.subplots(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.robot, = self.ax.plot([], [], 'b-', lw=2)
        self.ax.plot(self.start[0], self.start[1], 'go', label='Start')
        self.ax.plot(self.end[0], self.end[1], 'ro', label='End')

        self.obstacle_circles = []
        for center, radius in zip(self.obstacles, self.radii):
            circle = plt.Circle(center, radius, color='gray', alpha=0.6)
            self.ax.add_patch(circle)
            self.obstacle_circles.append(circle)

        self.ax.set_xlim(min(self.x_spline)-5, max(self.x_spline)+5)
        self.ax.set_ylim(min(self.y_spline)-5, max(self.y_spline)+5)
        self.ax.set_title(f"Path Cost: {self.best_cost:.2f} @ Iter {self.best_iteration}")
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3)
        self.fig.subplots_adjust(bottom=0.2)

        self.ani = animation.FuncAnimation(
            self.fig, self._update, frames=len(self.x_spline),
            init_func=self._init, blit=True, interval=60, repeat=False
        )
        self.canvas.draw()
    
    def _init(self):
        self.robot.set_data([], [])
        return (self.robot, *self.obstacle_circles)
    
    def _update(self, i):
        if i >= len(self.x_spline):
            return (self.robot, *self.obstacle_circles)

        x, y = self.x_spline[i], self.y_spline[i]
        self.robot.set_data(self.x_spline[:i + 1], self.y_spline[:i + 1])

        for idx, circle in enumerate(self.obstacle_circles):
            cx, cy = self.obstacles[idx]
            new_y = cy + 15 * np.sin(0.05 * i + idx)
            circle.center = (cx, new_y)

        if self._check_collision(x, y, i):
            current_pos = np.array([x, y])
            planner = PSOPathPlanner(current_pos, self.end, len(self.obstacles), 
                                   100, (min(x, self.end[0]), max(x, self.end[0])), 
                                   (min(y, self.end[1]), max(y, self.end[1])))
            result, cost, iter_found, _ = planner.run()
            if result[0] is not None:
                self.x_spline, self.y_spline = result
                self.ani.event_source.stop()
                self.ani.frame_seq = self.ani.new_frame_seq()
                self.ani.event_source.start()
            else:
                self.ani.event_source.stop()
                messagebox.showerror("Collision", "Robot collided and could not replan.")

        return (self.robot, *self.obstacle_circles)
    
    def _check_collision(self, x, y, frame):
        for idx, (cx, cy) in enumerate(self.obstacles):
            cy += 15 * np.sin(0.05 * frame + idx)
            if np.linalg.norm([x - cx, y - cy]) < self.radii[idx] + 1.5:
                return True
        return False