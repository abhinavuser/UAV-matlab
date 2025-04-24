# AI for Controlling a Simulated Swarm of UAVs with Pollutant-Cloud Tracking

This project implements an AI system to control a swarm of unmanned aerial vehicles (UAVs) for tracking and monitoring pollutant clouds. The simulation includes realistic UAV dynamics, UAV-to-UAV communication, battery life limitations, and cloud dispersion models.

## Project Overview

The system simulates a swarm of UAVs that work together to track and monitor the concentration of pollutants in a moving cloud. Each UAV is equipped with:
- Sensors to detect pollutant concentration
- GPS for navigation
- Communication capabilities to coordinate with other UAVs
- A limited battery life

The UAVs use a combination of swarm intelligence algorithms to efficiently track the cloud while avoiding collisions with each other.

## Features

- Realistic UAV dynamics with velocity and turn rate constraints
- Simulated sensor noise and GPS error
- Inter-UAV communications with message delay
- Collision avoidance algorithms
- Pheromone-based coordination strategy
- Battery life simulation with random failures
- Two different cloud dispersion models for testing

## Files and Components

- `sim_start.m`: Main simulation script that initializes and runs the simulation
- `UavBrain.m`: Implements the AI decision-making logic for each UAV
- `UavBody.m`: Physical model of the UAV with movement constraints
- `Pheremone.m` and `PheremoneType.m`: Implements the pheromone-based coordination system
- `Message.m`: Defines the communication protocol between UAVs
- `cloudplot.m`: Visualization for the pollution cloud
- `cloudsamp.m`: Sampling function for pollution concentration
- `cloud1.mat` and `cloud2.mat`: Pre-computed cloud dispersion data
- Support functions:
  - `moveRungeKutta.m`: Advanced movement calculation
  - `forwardEuler.m`: Simple movement calculation
  - `sqrLen.m`: Utility function for distance calculations

## Requirements

- MATLAB R2014b or later
- No additional toolboxes required

## How to Run

1. Open MATLAB
2. Navigate to the project directory
3. Run the main simulation script:
   ```matlab
   sim_start
   ```

## Simulation Parameters

The following parameters can be adjusted in `sim_start.m`:
- `uavCount`: Number of UAVs in the swarm (default: 24)
- `dt`: Time step for simulation (default: 1.0 second)
- `maxTime`: Maximum simulation time (default: 1800 seconds/30 minutes)
- `uavBatteryFailProb`: Probability of a UAV having reduced battery life (default: 0.1)
- `drawCloud`, `drawUavs`, `drawAI`, `drawAIExtra`: Visualization options

## AI Strategy

The UAV swarm uses a distributed AI approach where each UAV:
1. Takes sensor readings to detect pollutant concentration
2. Communicates its position and findings to nearby UAVs
3. Leaves virtual "pheromones" to mark searched areas
4. Makes decisions based on local information and pheromone trails
5. Avoids collisions with other UAVs
6. Returns home when battery is low

## Performance Metrics

After running the simulation, the system reports:
- Number of UAVs that failed to return to their launch spot
- Number of UAVs that crashed into other UAVs
- Number of UAVs that left the bounds of the map