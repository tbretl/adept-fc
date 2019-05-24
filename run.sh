#!/bin/bash

pipe=config_files/log_pipe
if [[ -p $pipe ]]
then
    echo "Logging terminal output." 
else 
    mkfifo $pipe 
fi

#This will save all terminal outputs to a log file:
tee outlog.dat < $pipe & 
sudo ./bin/adept_fc > $pipe
