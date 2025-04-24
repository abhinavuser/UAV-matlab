# Getting Started with UAV Swarm Simulation

This guide will help you get started with the UAV swarm simulation for pollutant-cloud tracking.

## Prerequisites

1. **MATLAB**: This simulation requires MATLAB R2014b or later.
   - No additional toolboxes are required.
   - The simulation has been tested on Windows, macOS, and Linux.

## Installation

1. **Clone or download** the repository to your local machine.
2. **Open MATLAB** and navigate to the project directory.

## Quick Start

### Running the Simulation

#### Method 1: Using the GUI (Windows Only)
1. Double-click on `run_simulation.bat` in Windows Explorer.
2. The script will attempt to locate your MATLAB installation and run the simulation.

#### Method 2: Using the Command Line
1. Open a terminal or command prompt.
2. Navigate to the project directory.
3. On Unix/Linux/Mac:
   ```bash
   # Make the script executable (only needed once)
   chmod +x run_simulation.sh
   
   # Run the simulation
   ./run_simulation.sh
   ```
4. On Windows:
   ```cmd
   run_simulation.bat
   ```

#### Method 3: Direct MATLAB Execution
1. Open MATLAB.
2. Navigate to the project directory.
3. Run:
   ```matlab
   run_simulation
   ```

### Configuring the Simulation

1. Open `run_simulation.m` in MATLAB.
2. Modify the parameters according to your needs:
   ```matlab
   % Number of UAVs in the swarm
   uavCount = 24;
   
   % Time step for simulation (seconds)
   dt = 1.0;
   
   % Maximum simulation time (seconds)
   maxTime = 1800.0;
   
   % ... and more parameters
   ```
3. Save the file and run the simulation.

### Analyzing Results

After running a simulation, you can analyze the results:

1. Make sure the simulation has completed (don't clear the MATLAB workspace).
2. Run:
   ```matlab
   visualize_results
   ```
3. The script will generate several plots and print statistics in the command window.

## Experiment Ideas

Here are some experiments you can try:

1. **Change the number of UAVs**:
   - Try with fewer UAVs (e.g., 10) and see if they can still track the cloud effectively.
   - Try with more UAVs (e.g., 40) and see if they avoid collisions successfully.

2. **Adjust battery failure probability**:
   - Set `uavBatteryFailProb` to 0.5 for a more challenging scenario.
   - Set it to 0 to see performance with fully charged batteries.

3. **Try different cloud models**:
   - Change `cloudSelection` between 1 and 2 to test different cloud dispersion patterns.

4. **Modify UAV spacing**:
   - Decrease `initialUavSpacing` to test the collision avoidance algorithms.
   - Increase it to see how it affects the initial search phase.

## Troubleshooting

### Common Issues

1. **MATLAB not found**:
   - Make sure MATLAB is installed and in your system PATH.
   - Edit the batch/shell scripts to point to your MATLAB installation.

2. **Simulation runs slowly**:
   - Reduce the number of UAVs.
   - Set `drawCloud` to `false` for better performance.
   - Set `drawAIExtra` to `false` (default) as it's performance-intensive.

3. **All UAVs crash**:
   - This might indicate a problem with the parameters.
   - Try increasing `uavCrashRadius` or `initialUavSpacing`.

### Getting Help

If you encounter issues not covered here, please:
1. Check the comments in the code files for more details.
2. Read the documentation in the README.md file.
3. Experiment with different parameter settings.

## Next Steps

Once you're comfortable with running the simulation, you can:

1. **Modify the AI**: Look at `UavBrain.m` to change the UAV decision-making process.
2. **Improve collision avoidance**: Experiment with different algorithms in `UavBrain.m`.
3. **Create your own cloud models**: Generate new .mat files with different dispersal patterns.
4. **Extend the visualization**: Add more analytics to `visualize_results.m`. 