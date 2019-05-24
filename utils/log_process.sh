#!/bin/bash


#read cpu usage of each process: 
rm process_use.dat
pidstat -p ALL 1  > process_use.dat 


