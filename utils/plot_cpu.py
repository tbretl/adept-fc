import os
import fileinput
import re
import matplotlib.pyplot as plt
import numpy as np

#variables:
file_process = "process_use.dat"
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
                    if ((line[2] == ":") and (data[-4] != "CPU") and (data[-4] != "")):
                        time.append(float(line[6:8])+ 60*float(line[3:5]) + 3600*float(line[0:2]))
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
        data[x,y] = float(cpu_use[i])/4.0
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
    data_plt = np.zeros(shape=(a,b),dtype=object)
    legend_plt = np.empty(shape=(a,b),dtype=object)
    legend_plt.fill(-1)
    cpu_tot = np.zeros(shape=(b,5),dtype=object)


    sort_data(process_time,process_name,cpu_num,cpu_use,data_plt,rows_plt,time_plt,legend_plt)


    #visualize the data:
    plt.plot(time_plt[0],0,'k-',label="Total % CPU") 
    plt.legend()
    colors = ['red','blue','green','black']
    count = 0
    for i in xrange(a):
        try:
            plt.plot(time_plt,data_plt[i][:],linewidth=0.5,ms = 0.8)

            if rows_plt[i] in module_names:
                temp = data_plt[i][:]
                plt.text(time_plt[int(len(time_plt)/2)+count],temp[int(len(time_plt)/2)],rows_plt[i],color=colors[3])
                count = count + 20 
        except:
            print "error"
    
    #total CPU usage
    for i in xrange(b):
        for j in xrange(a):
            try:
                cpu_tot[i,0] = cpu_tot[i,0] + float(data_plt[j][i])
            except:
                cpu_tot[i,0] = cpu_tot[i,0]

            try: 
                if (legend_plt[j,i] >= 0): 
                    cpu_tot[i,(legend_plt[j,i]+1)] = cpu_tot[i,(legend_plt[j,i]+1)] + float(data_plt[j][i]) 
            except:
                if (legend_plt[j,i] >= 0):
                    cpu_tot[i,(legend_plt[j,i]+1)] = cpu_tot[i,(legend_plt[j,i]+1)] 

    plt.plot(time_plt,cpu_tot[:,0],'k-',linewidth=2)
  
    #plot formatting
    plt.grid(color='0.7', linestyle='-', linewidth=1)
    plt.xlabel("time [sec]")
    plt.ylabel("% cpu")
    plt.title("Total processes: %i  Duration: %i seconds" % (a,(time_plt[-1]-time_plt[0])))

    #plot cpu usege
    plt.figure()
    plt.plot(time_plt,cpu_tot[:,0],'k-',linewidth=2,label="ALL")
    plt.plot(time_plt,cpu_tot[:,1],'r-',linewidth=2,label="CPU 0")
    plt.plot(time_plt,cpu_tot[:,2],'b-',linewidth=2,label="CPU 1") 
    plt.plot(time_plt,cpu_tot[:,3],'g-',linewidth=2,label="CPU 2")
    plt.plot(time_plt,cpu_tot[:,4],'y-',linewidth=2,label="CPU 3")
    plt.legend() 
    #plot formatting
    plt.grid(color='0.7', linestyle='-', linewidth=1)
    plt.xlabel("time [sec]")
    plt.ylabel("% cpu")
    plt.title("Total processes: %i  Duration: %i seconds" % (a,(time_plt[-1]-time_plt[0])))
   
    plt.show()
