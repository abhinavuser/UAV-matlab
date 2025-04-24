@echo off
REM Run UAV Swarm Simulation
REM This batch file runs the MATLAB simulation for UAV swarm

echo Checking for MATLAB installation...

REM Check if MATLAB is installed in common locations
if exist "C:\Program Files\MATLAB" (
    echo MATLAB found.
    echo Starting UAV Swarm Simulation...
    
    REM Try to find the latest MATLAB version
    for /f "delims=" %%i in ('dir /b /ad "C:\Program Files\MATLAB\R*" 2^>nul') do (
        set MATLAB_PATH=C:\Program Files\MATLAB\%%i\bin\matlab.exe
    )
    
    REM Run MATLAB with the simulation
    "%MATLAB_PATH%" -nosplash -r "try, run('run_simulation.m'); catch e, disp(getReport(e)); end;"
    
    echo Simulation complete.
) else (
    echo Error: MATLAB installation not found.
    echo Please make sure MATLAB is properly installed.
    pause
    exit /b 1
)

pause 