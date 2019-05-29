# adept-fc

This repository has the code that will run onboard the ADEPT-FC aircraft, to test flight control with distributed electric propulsion.

# Running the autopilot
0. install dependencies (zcm, Navio2)-instructions in `notes` directory 
1. build the software `make all` 
2. type `./run.sh` 

This will print outputs to the terminal and to the file `outlog.dat` (usefull for flights when the terminal isn't attached). 
