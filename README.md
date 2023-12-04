# Requirements
Matlab / Simulink 2020a
#### Toolbox
Communication Toolbox

DSP System Toolbox

Robotics Toolbox

# AINDI

This is a repository containing the Matlab / Simulink file, to simulate a quadrotor using the adaptive-incremental-nonlinear-dynamic-inversion controller proposed in :

**"Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles"**, Ewoud J. J. Smeur,∗ Qiping Chu,† and Guido C. H. E. de Croon, Delft University of Technology, 2629 HS Delft, The Netherlands - DOI: 10.2514/1.G001490

This work is based on the existing adatation of the following paper :

*S. Sun, X. Wang, Q. Chu and C. d. Visser, **"Incremental Nonlinear Fault-Tolerant Control of a Quadrotor With Complete Loss of Two Opposing Rotors, in IEEE Transactions on Robotics, doi: 10.1109/TRO.2020.3010626.*

## File structure
*run.m* is the main script to run the simulation, after loading necessary simulation and control parameters defined in *simParams.m* & *controlParams.m* respectively. The high-level simulation archetecture is given in *frame.slx* file, where one can design the position + yaw command. The flight controller is given in *controls.slx*, and the 6Dof simulator is given in *droneSim.slx* file. visualize.slx contains the visualization block. 

## How to test and compare with the INDI approach?
The innovation of this research, is to solve quadrotor fault-tolerant control problem using the Incremental-Nonlinear-Dynamic-Inversion (INDI) method. One can simply test it by directly running the simulation (run run.m file). 

### Change failuire type
In controlParams.m file, one can select swtching off which one or two propellers by changing par.fail_id. If par.DRF_enable is set as 1, then both the rotor of the given index and the rotor on its opposite side are switched off. 

### Change initial condition
Initial condition can be changed in the simParams.m file.

### Change input command
Input command can be designed in the frame.slx file.

### Test your own controller
To test your own controller with the given simulation, simply replace the content in controls.slx.


