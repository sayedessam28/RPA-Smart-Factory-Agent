# Integrated Route Planning Algorithm (RPA) for Smart Factories

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python&logoColor=white)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

## Project Overview

This repository hosts the source code for the research project: **"An
Integrated Route Planning Algorithm (RPA) for Autonomous Mobile Robots
in Smart Manufacturing"**.

The project simulates a centralized controller for a fleet of
**Autonomous Mobile Robots (AMRs)** operating in a dynamic smart factory
environment (20x20 Grid). It addresses three critical challenges in
multi-agent systems:

1.  **Task Assignment:** Selecting the most suitable robot for a
    specific task based on skill and proximity.
2.  **Path Planning:** Navigating efficiently through obstacles using
    heuristic search.
3.  **Power Management:** Intelligent charging scheduling to prevent
    system downtime.

> **Key Achievement:** The proposed RPA framework enhances operational
> quality by **40%** and ensures **100% system reliability** (Zero
> Unplanned Downtime).

------------------------------------------------------------------------

## Key Modules & Algorithms

The RPA framework is built upon three integrated intelligent modules:

### 1. Robot Finding Module (RFM) 

-   **Goal:** Optimal Task Assignment.
-   **Algorithm:** **Rule-Based Selection Agent**.
-   **Logic:** Prioritizes robots based on a strict hierarchy:
    1.  **Skill Match**
    2.  **Proximity** (Manhattan Distance)
    3.  **Battery Level**

### 2. Route Selection Module (RSM) 

-   **Goal:** Efficient Pathfinding & Obstacle Avoidance.
-   **Algorithm:** **A\*** Search Algorithm.
-   **Logic:** Finds the shortest path using Manhattan Distance
    heuristic while avoiding obstacles.

### 3. Robot Charging Module (RCM) 

-   **Goal:** Intelligent Battery Management.
-   **Algorithm:** **Tuned Fuzzy Logic Controller**.
-   **Logic:** Inputs: **Charge %**, **Velocity**, **Congestion** to
    determine charging priority.

------------------------------------------------------------------------

## Installation & Usage

### Prerequisites

``` bash
pip install numpy scipy scikit-fuzzy matplotlib networkx
```

### Setup

``` bash
git clone https://github.com/sayedessam28/RPA-Smart-Factory.git
cd RPA-Smart-Factory
```

### Run Simulation

``` bash
python factory_agent_project.py
```

------------------------------------------------------------------------

## Simulation Results

### Quality & Speed

  Metric               RPA               Baseline   Improvement
  -------------------- ----------------- ---------- -------------
  Optimal Match Rate   **94.12%**        53.49%     **+40.6%**
  Avg Response Time    **10.24 steps**   14.74      Faster

### Reliability

  Metric               RPA     Baseline   Result
  -------------------- ------- ---------- -------------------
  Unplanned Downtime   **0**   0          **100% Reliable**

------------------------------------------------------------------------

## Project Structure

``` text
RPA-Smart-Factory/
├── factory_agent_project.py
├── requirements.txt
├── README.md
└── images/
    ├── rfm_chart.peng
    └── rcm_chart.peng
    └── rsm_chart.peng
```

## Future Work

- Implementing Reinforcement Learning (Q-Learning) for the Robot Finding Module to adapt to changing factory layouts dynamically.

- Integrating dynamic obstacle avoidance for real-time navigation.


