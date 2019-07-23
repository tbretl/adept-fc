#!/bin/bash

pipe=config_files/log_pipe
if [[ -p $pipe ]]
then
    echo "Logging terminal output." 
else 
    mkfifo $pipe 
fi

#read in the file number 
file_num=$(<config_files/sequence.dat)
num=$((file_num + 1))


#This will save all terminal outputs to a log file:
file_name="outlog_$num.dat" 
tee $file_name < $pipe & 
sudo ./bin/adept_fc > $pipe
