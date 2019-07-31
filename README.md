# adept-fc

This repository has the code that will run onboard the ADEPT-FC aircraft, to test flight control with distributed electric propulsion.

# Building the software
1. install dependencies (zcm, Navio2) - instructions in `notes` directory.
2. if this is the first build, create an empty bin directory by typing 'mkdir bin'.
2. build the software `make all`.
3. copy `config_files/rc.local` into `etc/rc.local` 

# Starting the software
1. Type `sudo ./run.sh`.
2. This will print outputs to the terminal and to the file `outlog.dat` (usefull for flights when the terminal isn't attached).

# Using the monitor app
1. Once all the modules are booted, type `help` to see a list of options in the monitor app.
2. Type 'monitor check' to perform pre-flight checks.
3. Must type `pwm arm` for outputs to be armed, and for logging to begin.
4. Once satisfied, disconnect from the software by typing `monitor exit`.
5. Fly!
6. Regain access to the software by running the monitor app: `sudo ./bin/monitor`.
7. Disarm pwm outputs by typing 'pwm disarm'.
8. Type `all exit` to shutdown all modules.

# Running in HITL mode
1. change `hitl false` to `hitl true` in `config_files/adept_fc.config`.
2. Run the autopilot as usual.

note: the 'hitl' module expects udp messages upon startup.

# License

This repository has a mixed license. The header of each source file indicates whether the file falls under the MIT License (see https://github.com/tbretl/adept-fc/blob/master/LICENSE_MIT) or the GPLv3 License (see https://github.com/tbretl/adept-fc/blob/master/LICENSE_GPL).
