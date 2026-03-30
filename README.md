# CableDrivenRobot-TFG
Kinematic and dynamic simulation of a Cable Driven Parallel Robot with 3DOF using CoppeliaSim.

## Requirements

CoppeliaSim

[Download CoppeliaSim](https://www.coppeliarobotics.com/)

## Simulation

1. git clone the proyect.
   ```bash
   git clone https://github.com/albalopezd/CableDrivenRobot-TFG.git
2. Move to the scene folder of the proyect.
   ```
   cd route/to/the/scene
3. Launch CoppeliaSim with the scene file. (Make sure CoppeliaSim is configurated in the PATH or use an alias).
4. Push PLAY buttom to start the simulation.
5. Simulation`s data is in the csv folder.

## Plots and scripts

  ### plot_generic.py — general purpose                                                                                                                             
  ```bash
  python plot_generic.py <csv_file> <x_col> <y_col>
  ```
  - e.g.:                                                                                                                                                       
  python plot_generic.py ../csv/direct_kinematic_validation_4seg.csv time d_real                                                                                
  - list available columns:                                                                                                                                     
  python plot_generic.py ../csv/inverse_kinematic_validation_4seg.csv --list                                                                                    
                                                                                                                                                                
                                                        
  ### plot_direct_kinematics.py — 4 subplots:                                                                                                                       
                                                                                                                                                                
  1. Real vs theoretical position (x, y, z) — validates the forward model
  2. Position error d_real over time — accuracy metric                                                                                                          
  3. Total rotation theta_total over time — orientation tracking                                                                                                
  4. Segment bending (b1x, b2x, b3x) — deformation per segment                                                                                                  
                                                                                                                                                                
                                                         
  ### plot_inverse_kinematics.py — 4 subplots:                                                                                                                      
                                                            
  1. Target vs real position (x, y, z) — validates IK tracking
  2. error_dist over time — IK convergence curve (key result)                                                                                                   
  3. Joint angles theta, phi — configuration space trajectory                                                                                                   
  4. Cable lengths c1, c2, c3 — actuation signals over time                                                                                                     
                                                           
  Dependencies: pandas and matplotlib. Install with pip install pandas matplotlib if not already available.   

