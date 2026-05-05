#  3D UAV Path Planning Benchmark

### A Comparative Study of A*, RRT*, and PSO in Urban Environments

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-orange)
![Status](https://img.shields.io/badge/status-research-green)
![License](https://img.shields.io/badge/license-MIT-blue)
![Field](https://img.shields.io/badge/field-UAV%20Navigation-blueviolet)

---

##  Author

**H. CHERIET**
🎓 USTO-MB — Université des Sciences et de la Technologie d'Oran

---

## 📌 Overview

This repository presents a **3D UAV path planning benchmark framework** comparing:

*  **A*** — graph-based optimal search
*  **RRT*** — sampling-based exploration
*  **PSO** — optimization-based trajectory generation

All algorithms are evaluated in **realistic 3D urban environments** with obstacles.

---

## 🖼️ Demo (Add your figure here)

> 📸 Replace this with your plot or animation

```markdown
![Demo](results/demo.png)
```

---

## ⚡ Quick Start

```matlab
run('experiments/Comparison.m')
```

✔ Runs all algorithms
✔ Displays 3D paths
✔ Outputs metrics

---

## 🧪 Experiments

The framework reproduces **four benchmark scenarios**:

*  Sparse environment
*  Small map
*  Large map
*  Normal Map

Each scenario evaluates:

*  Path Length
*  Turning Angles
*  Computation Time

---

## 📊 Results Summary

| Scenario   | Best Path | Smoothest | Fastest |
| ---------- | --------- | --------- | ------- |
| Most cases | 🥇 A*     | 🥇 PSO    | 🥇 A*   |

📌 Key findings:

* ✅ **A*** → shortest and fastest paths
* 🔁 **RRT*** → robust but suboptimal
* 🎯 **PSO** → smoothest trajectories

These results match the findings in the published paper.

---

##  Project Structure

```text
Comparison-Repo/
│
├── experiments/
│   └── Comparison.m
│
├── src/
│   ├── Astar/
│   ├── RRTstar/
│   ├── PSO/
│   ├── metrics/
│   └── maps/
│
├── results/
│   └── *.mat
│
└── README.md
```

---

## ⚙️ Requirements

* MATLAB R2020+
* Navigation Toolbox

---

##  Algorithms Overview

###  A*

* Optimal shortest path
* Fast execution
* Grid-based search

###  RRT*

* Random tree exploration
* Works in high-dimensional spaces
* Near-optimal

###  PSO

* Bio-inspired optimization
* Smooth trajectories
* Computationally expensive

---

## 📈 Metrics

### Path Length

[
L = \sum ||x_{i+1} - x_i||
]

### Turning Angle

Measures trajectory smoothness for UAV dynamics

### Computation Time

Measured using `tic / toc`

---

##  Reproducibility

✔ Deterministic A*
✔ Seeded RRT* (`rng(1)`)
✔ Configurable PSO

---

## 🚀 Future Work

*  TIG / APPATT* integration
*  Deep Learning-guided planning
*  UAV dynamic constraints (MPC)
*  ROS / Gazebo simulation

---

## 📜 Citation

```bibtex
@inproceedings{cheriet2024uav,
  author    = {Cheriet, Hichem and Badra, Khellat Kihel and Samira, Chouraqui},
  title     = {Comparative Analysis of UAV Path Planning Algorithms for Efficient Navigation in Urban 3D Environments},
  booktitle = {2024 International Conference of the African Federation of Operational Research Societies (AFROS)},
  year      = {2024},
  pages     = {1--8},
  address   = {Tlemcen, Algeria},
  doi       = {10.1109/AFROS62115.2024.11037069},
  keywords  = {UAV, Path Planning, A*, RRT*, PSO, 3D Navigation}
}
```

---

## 🤝 Contributing

Pull requests are welcome!
Feel free to add new planners or improvements.

---

## ⭐ Support

If you find this useful:

👉 Star ⭐ the repo
👉 Cite 📖 the paper

---

## 🏫 Acknowledgment

Supported by:

**USTO-MB — Algeria**
Research project on UAV path planning and control

---

## 📬 Contact

**H. CHERIET**
Research in UAV Navigation & AI
hichem.cheriet@univ-usto.dz

---

⭐ *This repository is part of ongoing PhD research in UAV path planning.*
