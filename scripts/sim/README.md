# Sonar-based Odometry System

## Overview
The sonar-based odometry mainly includes three components:
1. Initialization
2. AnP pose tracking  
3. 3D reconstruction

At the initialization stage, the noise-corrupted poses at the first two frames are provided so that some initial points (simultaneously observed from these two poses) can be triangulated via the ANRS algorithm. After the initialization stage, AnP and ANRS algorithms are alternately executed to form a full odometry system. Specifically, the AnP algorithm is used to track new sonar frames with already reconstructed 3D points, while the ANRS algorithm triangulates new points to the map with latest two sonar frames.

## Algorithm
Input: T_t0, T_t1, SI_q_t0, SI_q_t1, ..., SI_q_tn

W_p_set ← 0
ΔT ← T_t1^(-1) · T_t0
s_p_t0 ← triangulation(ΔT, SI_q_t0, SI_q_t1)
W_p ← T_t0 · s_p_t0
W_p_set ← W_p_set ∪ W_p
While i = 2 to n-1:
T_ti ← ANP(SI_q_ti, W_p_set)
ΔT ← T_ti^(-1) · T_t(i-1)
s_p ← triangulation(ΔT, SI_q_t(i-1), SI_q_ti)
p^W ← T_t(i-1) · W_p
W_p_set ← W_p_set ∪ {p^W}

apache

Copy

## Simulator
![Simulator](figures/simulator.jpg)

The simulator shows:
- Red dots: 3D feature points in the environment
- Green space: Sonar's field of view (FOV)
- Blue dots: Feature points within the FOV
- Red curve: Robot's trajectory

## Experimental Setup
- Sonar's measurable distance and angle ranges match previous subsection
- Feature point density: ~40-50 visible points at any pose
- Noise intensities: σd=10^(-5) m and σθ=10^(-5)
- Initial poses provided with Gaussian noise:
  - Rotation matrix: std dev = 10^(-5)
  - Translation vector: std dev = 10^(-4)
- Test trajectories: 8-shaped, circle, and square