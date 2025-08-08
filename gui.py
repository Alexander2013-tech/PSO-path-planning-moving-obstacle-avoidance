import tkinter as tk
from tkinter import ttk, messagebox
import threading
import numpy as np
from pso import PSOPathPlanner
from animation import PathAnimator
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

class PathPlannerGUI:
    def __init__(self):
        self.window = None
        self.convergence_data = []
        self.space_bounds_x = (0, 100)
        self.space_bounds_y = (0, 100)
        
    def run(self):
        self.window = tk.Tk()
        self.window.title("PSO Path Planning")
        self.window.geometry("960x540")

        self._setup_styles()
        self._create_widgets()
        self.window.mainloop()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#f0f2f5")
        style.configure("TLabel", background="#f0f2f5", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=6)
        style.configure("TEntry", font=("Segoe UI", 10), padding=4)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.window, padding=10, style="TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.control_panel = ttk.Frame(main_frame, padding=20, relief="raised", borderwidth=1, style="TFrame")
        self.control_panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        self.canvas_frame = ttk.Frame(main_frame, style="TFrame")
        self.canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self._create_control_panel()

    def _create_control_panel(self):
        ttk.Label(self.control_panel, text="PSO Path Planner with\nMoving obstacles", 
                 font=("Segoe UI", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 15))

        self.obs_entry = self._labeled_entry("Number of Obstacles (1 or 2 only):", 2, 1)
        self.iter_entry = self._labeled_entry("Max Iterations:", 300, 2)
        self.sx_entry = self._labeled_entry("Start X:", 10, 3)
        self.sy_entry = self._labeled_entry("Start Y:", 10, 4)
        self.ex_entry = self._labeled_entry("End X:", 90, 5)
        self.ey_entry = self._labeled_entry("End Y:", 90, 6)

        self.loading_label = ttk.Label(self.control_panel, text="", foreground="blue", 
                                     font=("Segoe UI", 10, "italic"))
        self.loading_label.grid(row=7, column=0, columnspan=2, pady=(5, 0))

        self.submit_btn = ttk.Button(self.control_panel, text="Start Planning", command=self._on_submit)
        self.submit_btn.grid(row=8, column=0, columnspan=2, pady=10, sticky="ew")

        self.convergence_btn = ttk.Button(self.control_panel, text="Show Convergence", 
                                        state="disabled", command=self._show_convergence)
        self.convergence_btn.grid(row=9, column=0, columnspan=2, pady=(0, 10), sticky="ew")

    def _labeled_entry(self, label, default, row):
        ttk.Label(self.control_panel, text=label).grid(row=row, column=0, sticky="w", pady=2, padx=5)
        entry = ttk.Entry(self.control_panel)
        entry.insert(0, str(default))
        entry.grid(row=row, column=1, sticky="ew", pady=2, padx=5)
        return entry

    def _on_submit(self):
        def run_planner():
            try:
                n_obs = int(self.obs_entry.get().strip())
                if n_obs not in [1, 2]:
                    raise ValueError("Only 1 or 2 obstacles are supported.")

                max_iterations = int(self.iter_entry.get().strip())
                sx, sy = float(self.sx_entry.get().strip()), float(self.sy_entry.get().strip())
                ex, ey = float(self.ex_entry.get().strip()), float(self.ey_entry.get().strip())
                start = np.array([sx, sy])
                end = np.array([ex, ey])

                self.space_bounds_x = (min(sx, ex) - 10, max(sx, ex) + 10)
                self.space_bounds_y = (min(sy, ey) - 10, max(sy, ey) + 10)

                self.submit_btn.config(state="disabled")
                self.convergence_btn.config(state="disabled")
                self.loading_label.config(text="\u23F3 Planning path, please wait...")

                # Create path planner and run PSO
                planner = PSOPathPlanner(start, end, n_obs, max_iterations, 
                                       self.space_bounds_x, self.space_bounds_y)
                result, cost, iter_found, conv = planner.run()

                def update_ui():
                    self.submit_btn.config(state="normal")
                    self.loading_label.config(text="")
                    self.convergence_data.clear()
                    self.convergence_data.extend(conv)
                    if result[0] is not None:
                        self.convergence_btn.config(state="normal")
                        animator = PathAnimator(self.canvas_frame, start, end, *result, 
                                              planner.obstacles, planner.radii, cost, iter_found)
                        animator.animate()
                    else:
                        messagebox.showwarning("No Path", "No valid path found.")

                self.window.after(10, update_ui)

            except ValueError as e:
                self.submit_btn.config(state="normal")
                self.loading_label.config(text="")
                messagebox.showerror("Invalid Input", f"Error: {e}")

        threading.Thread(target=run_planner).start()

    def _show_convergence(self):
        if not self.convergence_data:
            messagebox.showwarning("No Data", "Run the planner first.")
            return
        
        conv_window = tk.Toplevel(self.window)
        conv_window.title("Convergence Plot")
        
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(self.convergence_data, 'b-', linewidth=2)
        ax.set_title("PSO Convergence")
        ax.set_xlabel("Iteration")
        ax.set_ylabel("Best Cost")
        ax.grid(True)
        
        canvas = FigureCanvasTkAgg(fig, master=conv_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        close_btn = ttk.Button(conv_window, text="Close", command=conv_window.destroy)
        close_btn.pack(pady=10)