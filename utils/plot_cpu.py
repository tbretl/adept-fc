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


module_names = ["adc","adept_fc","autopilot","hitl","monitor","pwm_out","rc_in", "scribe","vnins"]




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
                    if ((line[2] == ":") and (data[-4] != "CPU") and (data[-4] != "")) :
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
        x = get_row(process_name[i],rows)
        y = get_column(process_time[i],time)
        #insert values
        time[y] = process_time[i]
        rows[x] = process_name[i]
        data[x,y] = cpu_use[i]
        legend[x,y] = int(cpu_num[i])


def get_row(name,out_array):
    #search through array for a match, if none found take next empty spot
    for i in xrange(len(out_array)):
        if (out_array[i] == "x"):
            return i
        elif (out_array[i] == name):
            return i

def get_column(time,out_time):
    #search through array for a match, if none found take next empty spot
    for i in xrange(len(out_time)):
        if (out_time[i] == time):
            return i



#main program:
if __name__ == '__main__':

    #parse the data file:
    read_data(file_process,process_time,process_name,cpu_num,cpu_use)

    #to sort data
    a = len(set(process_name)) #gets number of unique names
    time_plt = sorted(set(process_time))
    b = len(time_plt) #gets number of unique times
    rows_plt = np.empty(a,dtype=object)
    rows_plt.fill("x")
    data_plt = np.empty(shape=(a,b),dtype=object)
    legend_plt = np.empty(shape=(a,b),dtype=object)
    cpu_tot = np.zeros(b,dtype=object)


    sort_data(process_time,process_name,cpu_num,cpu_use,data_plt,rows_plt,time_plt,legend_plt)


    #visualize the data:
    #plot each process as a line, time vs. %cpu, collored by cpu number, with process text label at max point
    #add a total cpu usage line for each cpu, and a total overall % cpu line++++++++++++++++++++++++
    plt.plot(time_plt[0],0,'r+-',label="CPU 0")
    plt.plot(time_plt[0],0,'bo-',label="CPU 1")
    plt.plot(time_plt[0],0,'gs-',label="CPU 2")
    plt.plot(time_plt[0],0,'yd-',label="CPU 3")
    plt.legend()
    colors = ['red','blue','green','black']
    for i in xrange(a):
        try:
            if legend_plt[i][0] == 0:
                plt.plot(time_plt,data_plt[i][:],'r+-',linewidth=0.5,ms = 0.8)
            elif legend_plt[i][0] == 1:
                plt.plot(time_plt,data_plt[i][:],'bo-',linewidth=0.5,ms = 0.8)
            elif legend_plt[i][0] == 2:
                plt.plot(time_plt,data_plt[i][:],'gs-',linewidth=0.5,ms = 0.8)
            elif legend_plt[i][0] == 3:
                plt.plot(time_plt,data_plt[i][:],'yd-',linewidth=0.5,ms = 0.8)

            if rows_plt[i] in module_names:
                temp = data_plt[i][:]
                try:
                    ind = int(float(np.where(temp == max(temp))[0]))
                    plt.text(time_plt[ind],max(temp),rows_plt[i],color=colors[legend_plt[i][0]])
                except:
                    ind = 0
                    plt.text(time_plt[ind],temp[ind],rows_plt[i],color=colors[legend_plt[i][0]])

        except:
            print "error"
    #plot total CPU usage
    for i in xrange(b):
        for j in xrange(a):
            try:
                cpu_tot[i] = cpu_tot[i] + float(data_plt[j][i])
            except:
                cpu_tot[i] = cpu_tot[i]

        cpu_tot[i] = cpu_tot[i]/4 #account for the 4 processors



    plt.plot(time_plt,cpu_tot,'k-',linewidth=2)

    plt.grid(color='0.7', linestyle='-', linewidth=1)
    plt.xlabel("time [sec]")
    plt.ylabel("% cpu")
    plt.show()



