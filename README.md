# MATLAB_Simulations

This repository contains implementations of drone swarm simulation that explores various stages of development, from single-particle navigation to complex swarm behaviors and challenges in obstacle avoidance.

## GIF
Simulations have been attached as GIFs

## Stages Overview

### Stage 0 
Initial Setup - Establishes the basic 3D environment for the simulation, setting the foundation for all subsequent stages.

### Stage 1 
Single Particle Navigation - Tests basic navigation capabilities of a single particle moving towards predefined goals within the 3D environment.
![](https://github.com/Aya2Lati/MATLAB_Simulations/blob/main/singleParticle_simulation.gif)

### Stage 2 
Formation Initialisation - Implements and verifies the initial positioning of a swarm in a V formation, ensuring cohesion and correct particle placement.
![](https://github.com/Aya2Lati/MATLAB_Simulations/blob/main/V_Formation.gif)

### Stage 3
Formation Transition - Demonstrates the swarm's ability to transition between different formations (V, Line, Circle) while maintaining cohesion.
![](https://github.com/Aya2Lati/MATLAB_Simulations/blob/main/dynamic_particleFormation.gif)

### Stage 4 
Obstacle Avoidance for Single Particle - Develops and tests obstacle detection and avoidance for a single particle, identifying potential challenges for extending this capability to a swarm. This however faced issues due to failure to implement successfuly the obstacle avoidance

### Stage 5 
Swarm-Wide Obstacle Avoidance - Intended to apply the obstacle avoidance strategies from Stage 4 to an entire swarm. However, this stage was not successfully implemented due to complexities in coordinating swarm movements and obstacle detection.

# Future Work
Further development will focus on overcoming the challenges faced in Stage 4 and 5, enhancing the algorithms for swarm-wide obstacle avoidance, and improving the overall robustness and visualisation of the simulation.


