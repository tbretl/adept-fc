//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <chrono>
#include <zcm/zcm-cpp.hpp>
//message types:
#include "status_t.hpp"

using namespace std;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
            stat.should_exit = 0;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

pid_t run_process(const char* path){

    pid_t child_pid;

    child_pid = fork ();

    if (child_pid != 0){
        //parent
        return child_pid;
    }
    else {
        execl (path, path, NULL);
        std::cout << "an error occurred in execl" << std::endl;
        abort ();
    }

}


int main(int argc, char *argv[])
{
    std::ofstream file{"running"};

    zcm::ZCM zcm {"ipc"};

    //subscribe to channels
    Handler h0,h1,h2,h3,h4,h5,h6,h7;
    zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    zcm.subscribe("STATUS6",&Handler::read_stat,&h6);
    zcm.subscribe("STATUS7",&Handler::read_stat,&h7);

    zcm.start();

    //read in configuration file:
    string line[2];
    bool hitl = false;
    ifstream config_stream;

    config_stream.open("config_files/adept_fc.config");
    config_stream >> line[0] >> line[1];
    hitl = !line[1].compare("true");
    config_stream.close();

    ostringstream pid_list;

    //rc_in
    pid_list << run_process("bin/rc_in") << " ";
    auto start_time = std::chrono::steady_clock::now();
    while (!(h0.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 1000) {
            std::cout << "Timeout on rc_in startup..." << std::endl;
            break;
        }
    }


    if(!hitl){

        //vectornav sensor input
        pid_list << run_process("bin/vnins") << " ";
        start_time = std::chrono::steady_clock::now();
        while (!(h1.stat.module_status==1)){
            auto current_time = std::chrono::steady_clock::now();
            unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
            if (time_ms >= 1000) {
                std::cout << "Timeout on vnins startup..." << std::endl;
            break;
            }
        }


        //ADC sensor input
        pid_list << run_process("bin/adc") << " ";
        start_time = std::chrono::steady_clock::now();
        while(!(h2.stat.module_status==1)){
            auto current_time = std::chrono::steady_clock::now();
            unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
            if (time_ms >= 2000) {
                std::cout << "Timeout on ADC startup..." << std::endl;
            break;
            }
        }


    }
    else {
        //HITL interface
        pid_list << run_process("bin/hitl") << " ";
        start_time = std::chrono::steady_clock::now();
        while (!(h3.stat.module_status==1)){
            auto current_time = std::chrono::steady_clock::now();
            unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
            if (time_ms >= 1000) {
                std::cout << "Timeout on hitl startup..." << std::endl;
            break;
            }
        }
    }

    //autopilot
    pid_list << run_process("bin/autopilot") << " ";
    start_time = std::chrono::steady_clock::now();
    while (!(h4.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 1000) {
            std::cout << "Timeout on autopilot startup..." << std::endl;
        break;
        }
    }

    //Logger module
    pid_list << run_process("bin/scribe") << " ";
    start_time = std::chrono::steady_clock::now();
    while (!(h5.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 1000) {
            std::cout << "Timeout on scribe startup..." << std::endl;
        break;
        }
    }

    //pwm_out
    pid_list << run_process("bin/pwm_out");
    start_time = std::chrono::steady_clock::now();
    while (!(h6.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 10000) {
            std::cout << "Timeout on pwm startup..." << std::endl;
        break;
        }
    }

    string temp_pids = pid_list.str();
    char * send_pids = new char [temp_pids.length() + 1];
    strcpy(send_pids,temp_pids.c_str());
    //red_flag
    pid_t child_pid;
    child_pid = fork ();
    if (child_pid != 0){
        //parent
        //launch system monitor, consume this thread:
        execl ("bin/monitor", "bin/monitor",NULL);
    }
    else {
        execl ("bin/red_flag", "bin/red_flag",send_pids, nullptr);
        std::cout << "an error occurred in execl" << std::endl;
        abort ();
    }
    start_time = std::chrono::steady_clock::now();
    while (!(h7.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 20000) {
            std::cout << "Timeout on red_flag startup..." << std::endl;
        break;
        }
    }

    zcm.stop();




    return 0;
}
