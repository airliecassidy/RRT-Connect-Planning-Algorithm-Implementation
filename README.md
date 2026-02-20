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
- pybullet (for robotic simulation)

# Usage



