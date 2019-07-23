//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//keeps track of process ID's and CPU usage.

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <iomanip>
#include <sys/types.h>
#include <dirent.h>
#include "zcm/zcm-cpp.hpp"
//message types:
#include "status_t.hpp"
#include "red_flag_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
        }


        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

};

bool proc_exist(const std::string PID) {
    DIR* dir;
    struct dirent* ent;
    dir = opendir("/proc");
    while((ent = readdir(dir)) != NULL) {
        if (!PID.compare(ent->d_name)) {
            //check that process is not a zombie
            std::ostringstream file_pid;
            std::string status;
            file_pid << "/proc/" << PID << "/stat";
            std::ifstream pid_stream;
            pid_stream.open(file_pid.str());
            pid_stream >> status >> status >> status;
            pid_stream.close();
            if(status.compare("Z")){
                closedir(dir);
                return true;
            }
        }
    }
    closedir(dir);
    return false;
}

string get_temp(){
    string temperature;
    FILE *stream;
    int max_buffer = 25;
    char buffer[max_buffer];
    stream = popen("vcgencmd measure_temp 2>&1","r");
    if(fgets(buffer, max_buffer, stream) != NULL) temperature.append(buffer);
    pclose(stream);
    return temperature;
}

float get_cpu(){
    size_t times_1[10],times_2[10];
    std::ifstream cpu_stream;
    std::string cpu_line,dump;
    cpu_stream.open("/proc/stat");
    std::getline(cpu_stream,cpu_line);
    cpu_stream.close();
    std::istringstream ss(cpu_line);
    ss >> dump;
    for (int i=0; i<10; i++) {
        ss >> times_1[i];
    }
    usleep(500000);
    cpu_stream.open("/proc/stat");
    std::getline(cpu_stream,cpu_line);
    cpu_stream.close();
    ss.str("");
    ss.clear();
    ss.str(cpu_line);
    ss >> dump;
    for (int i=0; i<10; i++) {
        ss >> times_2[i];
    }
    //compute %usage over the wait time
    float idle_time = static_cast<float>((times_2[3]+times_2[4])-(times_1[3]+times_1[4]));
    float total_time = 0;
    for (int i=0; i<10; i++) {
        total_time += static_cast<float>(times_2[i] - times_1[i]);
    }
    return 100.0*(total_time - idle_time)/total_time;
}

int main(int argc, char* argv[])
{
    //store PIDs of module processes
    std::string proc_names[] = {"rc_in","hitl","vnins","adc","autopilot","scribe","pwm_out"};
    std::stringstream pid_list(argv[1]);
    std::string PIDs[6];
    std::string dump;
    for (int i; i<6; i++){
        if(std::getline(pid_list, dump, ' ')){
            PIDs[i] = dump;
        }
    }
    int name_inds[6];
    if (PIDs[5].empty()) { //hitl mode
        name_inds[0] = 0; name_inds[1] = 1; name_inds[2] = 4;
        name_inds[3] = 5; name_inds[4] = 6; name_inds[5] = 8;
    }else{//flight mode
        name_inds[0] = 0; name_inds[1] = 2; name_inds[2] = 3;
        name_inds[3] = 4; name_inds[4] = 5; name_inds[5] = 6;
    }

    zcm::ZCM zcm {"ipc"};
    //subscribe to incoming channels:
    Handler handlerObject,h0,h1,h2,h3,h4,h5,h6;
    //module status channels:
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    zcm.subscribe("STATUS6",&Handler::read_stat,&h6);

    //structures to publish:
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running
    red_flag_t red_msg;
    memset(&red_msg,1,sizeof(red_msg));

    zcm.start();

    //sequencing file numbers:
    std::ifstream seqFile ("config_files/sequence.dat", std::ifstream::in);
    int fileNum;
    seqFile >> fileNum;
    seqFile.close();
    fileNum++;
    std::ofstream seqFile2 ("config_files/sequence.dat");
    seqFile2 << fileNum;
    seqFile2.close();
    //setup log file
    char file_red[25];
    sprintf(file_red,"FlightLog_%i_red_flag.dat",fileNum);
    std::ofstream logfile_red;
    logfile_red.open(file_red,std::ofstream::out | std::ofstream::app | std::ofstream::binary);

    std::cout << "red_flag started" << std::endl;

    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS7",&module_stat);
        double time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        //get cpu load
        red_msg.cpu = get_cpu();
        logfile_red << std::setprecision(14) << time_now << std::setprecision(6) <<  " CPU%: " << red_msg.cpu << " " << get_temp() << std::endl;
        //check each module PID status
        for (int i=0; i<6; i++){
            red_msg.pid_status[i]=1;
            if (!PIDs[i].empty()){
                if (!proc_exist(PIDs[i])){
                    std::cout << "WARNING: " << proc_names[name_inds[i]] << " is no longer running." << std::endl;
                    logfile_red << std::setprecision(14) << time_now << std::setprecision(6) << " WARNING: " << proc_names[name_inds[i]] << " is no longer running." << std::endl;
                    red_msg.pid_status[i] = 0;
                }
            }
        }
        zcm.publish("RED_FLAG",&red_msg);
    }

    logfile_red.close();

    module_stat.module_status = 0;
    zcm.publish("STATUS7",&module_stat);

    std::cout << "red_flag exiting..." << std::endl;

    zcm.stop();

    return 0;
}
