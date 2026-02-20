# RRT-Connect Motion Planning Implementation

Bidirectional sampling-based motion planner for high-dimensional configuration spaces, focusing on implementing RRT-Connect with greedy tree extension, and accelerated convergence in constrained environments. 

This was built for research on exploration design for robotic path planning tasks with complex obstacle geometry and passages. 


# Overview

This project develops a full sampling-based motion planning pipeline for robotic manipulators operating in obstacle rich simulation environments. 

RRT-Connect grows two trees simultaneously from a start and end goal configuration. Trees expand through randomized sampling and incremental steering. A greedy connection phase attempts to fuse both trees into a continuous collision free trajectory.

# Dependencies
- numpy
- scipy
- tqdm
- pybullet (for robotic simulation of the Kinova Gen3 Lite arm)

Three environments are modelled: 
1. Free - no obstacles; used for validating the core planner.
2. Easiest - one ring of balloon obstacles surrounding the goal.
3. Hardest - Two concentric rings of obstacles blocing both the start and goal regions.

A successful plan is visualized below: 


<img width="502" height="496" alt="Screenshot 2026-02-08 at 7 33 58 PM" src="https://github.com/user-attachments/assets/9a7418c8-6bec-4caf-8c34-72c3b1f9b22f" />


Figure 1. Bidirectional RRT-Connect exploration in joint space. High node density indicates constrained feasible regions caused by obstacle tiers. The highlighted bridge marks successful greedy tree fusion and path reconstruction in the hardest environment.

# Running the Planner

The default environment is free: 

python arm_rrt.py 

However, can select a specific environment with the --environment flag: 
python arm_rrt.py --environment=Free
python arm_rrt.py --environment=Easiest
python arm_rrt.py --environment=Hardest

# Algorithm: RRT-Connect
The implementation of this algorithm follows the RRT-Connect formulation from Kuffner & LaValle (2000) and the broader treatment of sampling-based planning in LaValle's Planning Algorithms (2006). 

# Notes

All implementation changes are confined to arm_rrt.py — robots.py and gen3lite_urdf/ are unchanged.
The Node and Tree classes in arm_rrt.py provide the scaffolding for tree construction; the plan() function is where the RRT-Connect logic runs.
The robot model is a Kinova Gen3 Lite 6-DOF arm. Its joint limits and collision geometry are loaded automatically via the URDF files in gen3lite_urdf/.



