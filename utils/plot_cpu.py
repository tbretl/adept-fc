import os
import fileinput
import re
import matplotlib.pyplot as plt
import numpy as np

#variables:
file_process = "../process_use.dat"
file_memory  = "../memory_use.dat"
#to read data:
process_time = []
process_name = []
cpu_num = []
cpu_use = []




#functions:
def get_lines(filename):
    try:
        with open(filename,'r') as f:
            return len(f.readlines())
    except IOError:
        print("Error: Cannot find file.")


def read_data(filename,time, p_name,cpu_num,cpu_use):
    try:
        with open(filename,'r') as myFile:
            for line in myFile:
                try:
                    data = re.split('    | |\n',line)
                    if float(data[-4]):
#                        time.append(3600*int(line[0:1]) + 60*int(line[3:4]) + int(line[6:7]))
                        time.append(float(line[6:8])+ 60*float(line[0:2]) + 3600*float(line[0:2]))
                        p_name.append(data[-2])
                        cpu_num.append(data[-4])
                        cpu_use.append(data[-6])
                except:
                    pass
    except IOError:
        print("Error: Cannot find file.")


def sort_data(process_time,process_name,cpu_num,cpu_use,data,rows,time,legend):
    for i in xrange(len(process_time)):
        #get location in data matrix
        x = get_row(process_name,i)
        y = get_column(process_time[i])
        #insert values
        time[y] = process_time[i]
        rows[x] = process_name[i]
        data[x,y] = cpu_use[i]
        legend[x,y] = cpu_num[i]


def get_row(_name,j):
    #search through array for a match, if none found take next empty spot
    return 0

def get_column(_time):
    #search through array for a match, if none found take next empty spot
    return 0



#main program:
if __name__ == '__main__':

    #parse the data file:
    read_data(file_process,process_time,process_name,cpu_num,cpu_use)

    a = len(set(process_name)) #gets list of unique names
    b = len(set(process_time)) #gets list of unique times

    #to sort data
    time_plt = np.empty(len(process_time),dtype=float)
    rows_plt = np.empty(len(process_time),dtype=str)
    data_plt = np.empty(shape=(len(process_time),len(process_time)),dtype=float)
    legend_plt = np.empty(len(process_time),dtype=float)

    #visualize the data:
    #plot each process as a line, time vs. %cpu, collored by cpu number, with process text label at max point
    #add a total cpu usage line for each cpu, and a total overall % cpu line
    plt.scatter(process_time,cpu_use)
    plt.xlabel("time [sec]")
    plt.ylabel("% cpu")
    plt.show()

    #print to terminal for debugging:
#    for i in xrange(len(process_name)):
#        print '\n'
#        print process_time[i]
#        print process_name[i]
#        print cpu_num[i]
#        print cpu_use[i]

