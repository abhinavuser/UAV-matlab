#!/bin/bash
# Run UAV Swarm Simulation
# This script runs the MATLAB simulation for UAV swarm

# Check if MATLAB is installed
if command -v matlab >/dev/null 2>&1; then
    # MATLAB is available
    echo "Starting UAV Swarm Simulation..."
    
    # Run MATLAB in non-GUI mode
    matlab -nodisplay -nosplash -nodesktop -r "try, run('run_simulation.m'); catch e, disp(getReport(e)); end; exit;" | tail -n +11
    
    echo "Simulation complete."
else
    echo "Error: MATLAB is not installed or not in your PATH."
    echo "Please make sure MATLAB is properly installed and accessible from your terminal."
    exit 1
fi 