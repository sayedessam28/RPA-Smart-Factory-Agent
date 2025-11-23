# Integrated Route Planning Algorithm (RPA) for Smart Factories

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python&logoColor=white)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

## Project Overview

This repository hosts the source code for the research project: **"An Integrated Route Planning Algorithm (RPA) for Autonomous Mobile Robots in Smart Manufacturing"**.

The system simulates a centralized controller for **Autonomous Mobile Robots (AMRs)** operating in a dynamic smart factory (20x20 grid). 
It solves three main challenges:

1. **Task Assignment** – Assign the most suitable robot using a rule-based agent. 
2. **Path Planning** – Efficient navigation using an **A\*** search algorithm. 
3. **Power Management** – Smart charging scheduling using **Fuzzy Logic**.

> **Key Result:** 
> Improved operational quality by **40%** and achieved **100% system reliability** (Zero Unplanned Downtime).

---

## ⚙️ Key Modules & Algorithms

### **1. Robot Finding Module (RFM) **
- **Goal:** Select the best robot for each task. 
- **Algorithm:** Rule-Based Agent 
- **Priority Logic:** 
  1. Skill Match 
  2. Proximity (Manhattan Distance) 
  3. Battery Level 

---

### **2. Route Selection Module (RSM) **
- **Goal:** Compute the shortest path avoiding obstacles 
- **Algorithm:** **A\*** 
- **Heuristic:** Manhattan Distance 
---

### **3. Robot Charging Module (RCM) **
- **Goal:** Manage battery levels and charging queues 
- **Algorithm:** Tuned Fuzzy Logic Controller 
- **Inputs:** Charge %, Velocity, Congestion 

---

##  Installation & Usage

### **Prerequisites**
```bash
pip install numpy scipy scikit-fuzzy matplotlib networkx
```

### **Setup**
```bash
git clone https://github.com/sayedessam28/RPA-Smart-Factory.git
cd RPA-Smart-Factory
```

### **Run Simulation**
```bash
python factory_agent_project.py
```

---

## Simulation Results

### **1. Quality & Speed (RFM + RSM)**

| Metric | RPA (Proposed) | Baseline | Improvement |
|--------|----------------|----------|-------------|
| **Optimal Match Rate** | **94.12%** | 53.49% | **+40.6%** |
| **Avg Response Time** | **10.24 steps** | 14.74 steps | Faster |
| **A\* Avg Explored Nodes** | **71.3 nodes** | N/A | Efficient |

---

### **2. Reliability (RCM)**

| Metric | RPA (Fuzzy Logic) | Baseline | Result |
|--------|--------------------|----------|--------|
| **Unplanned Downtime** | **0** | 0 | **100% Reliable** |
| **Avg Availability** | 3.88 robots | 4.37 robots | Safe Operation |

---

## Project Structure
```text
RPA-Smart-Factory/
├── factory_agent_project.py
├── requirements.txt
├── README.md
└── images/
    ├── rfm_chart.png
    ├── rcm_chart.png
    └── rsm_chart.png
```

---

## Future Work
- Implementing Reinforcement Learning (Q-Learning) for the Robot Finding Module to adapt to changing factory layouts dynamically. 
- Integrating dynamic obstacle avoidance for real-time navigation.

---


