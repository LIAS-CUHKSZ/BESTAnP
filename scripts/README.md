# Sonar-based Odometry

## System Components
The sonar-based odometry mainly includes three components:
1. Initialization
2. AnP pose tracking
3. 3D reconstruction

At the initialization stage, the noise-corrupted poses at the first two frames are provided so that some initial points (simultaneously observed from these two poses) can be triangulated via the ANRS algorithm [21]. After the initialization stage, AnP and ANRS algorithms are alternately executed to form a full odometry system. Specifically, the AnP algorithm is used to track new sonar frames with already reconstructed 3D points, while the ANRS algorithm triangulates new points to the map with latest two sonar frames.

## Algorithm 2: Sonar-based Odometry

### Input
T_t0, T_t1, ^SI^q_t0, ^SI^q_t1, ..., ^SI^q_tn

### Algorithm Steps
1. ^W^P_set ← 0
2. ΔT ← T_t1^(-1) · T_t0
3. ^s^p_t0 ← triangulation(ΔT, ^SI^q_t0, ^SI^q_t1)
4. ^W^P ← T_t0 · ^s^p_t0
5. ^W^P_set ← ^W^P_set ∪ ^W^p

### Main Loop
while i = 2, i < n, i++ do:
1. T_ti ← ANP(^SI^q_ti, ^W^P_set)
2. ΔT ← T_ti^(-1) · T_t(i-1)
3. ^s^p ← triangulation(ΔT, ^SI^q_t(i-1), ^SI^q_ti)
4. P^W ← T_t(i-1) · ^W^p
5. ^W^P_set ← ^W^P_set ∪ {P^W}

## Experimental Setup
We then introduce the experimental setup. A simulator shown in Fig. 6 is designed. The sonar's measurable distance and angle ranges are the same as those in the previous subsection. The density of the feature points is set to a proper amount so that the number of feature points visible in the sonar is around 40 to 50 points at any pose. The noise intensities are set as σ_d = 10^(-5) m and σ_θ = 10^(-5). For the initial two poses, we provide them by adding Gaussian noises to the elements of the rotation matrix and translation vector. Specifically, the standard deviations of the noises are set as 10^(-5) and 10^(-4) for the rotation matrix and translation vector, respectively. We test the odometry system for three trajectories: 8-shaped, circle, and square.