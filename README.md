# PSO Path Planner with Dynamic Obstacles

## Description
A Particle Swarm Optimization (PSO) path planning algorithm with moving obstacle visualization.  
The application finds optimal paths while avoiding dynamically moving obstacles.

---

**Requirements:**
- Python 3.7+
- Packages:
  - `numpy`
  - `matplotlib`
  - `scipy`

---

## Usage

```bash
python main.py
```

**GUI Parameters:**

| Parameter            | Default Value | Description                                |
|----------------------|--------------:|--------------------------------------------|
| Number of Obstacles  | 2             | Only 1 or 2 supported                      |
| Max Iterations       | 300           | PSO optimization iterations                |
| Start X / Y          | 10, 10        | Path starting coordinates                  |
| End X / Y            | 90, 90        | Path destination coordinates               |

---

## Project Files

```
main.py         - Main application entry point
gui.py          - User interface implementation
pso.py          - PSO algorithm core
obstacles.py    - Obstacle generation and management
animation.py    - Path visualization
```

---

## Troubleshooting

**Missing Tkinter:**
```bash
sudo apt install python3-tk  # Linux
pip install tk               # windows
```

**Slow Performance:**
- Reduce `n_particles` in `pso.py`
- Decrease max iterations

**No Path Found:**
- Adjust start/end positions
- Increase iterations

---

Any corrections will be much appreciated