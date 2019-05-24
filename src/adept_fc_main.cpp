//This code launches the autopilot framework
//Aaron Perry, 4/9/2019

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
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

        //create a message object which can be accessed outside of the function call
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

void run_process (const char* path){

    pid_t child_pid;

    child_pid = fork ();

    if (child_pid != 0){
        //parent
    }
    else {
        execl (path, path, NULL);
        std::cout << "an error occurred in execl" << std::endl;
        abort ();
    }

}


int main(int argc, char *argv[])
{
    zcm::ZCM zcm {"ipc"};

    //subscribe to channels
    Handler h0,h1,h2,h3,h4,h5,h6;
    zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    zcm.subscribe("STATUS6",&Handler::read_stat,&h6);

    zcm.start();

    //read in configuration file:
    string line[2];
    bool hitl = false;
    ifstream config_stream;

    config_stream.open("config_files/adept_fc.config");
    config_stream >> line[0] >> line[1];
    hitl = !line[1].compare("true");
    config_stream.close();

    //rc_in
    run_process("bin/rc_in");
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
        run_process("bin/vnins");
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
        run_process("bin/adc");
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
        run_process("bin/hitl");
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
    run_process("bin/autopilot");
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
    run_process("bin/scribe");
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
    run_process("bin/pwm_out");
    start_time = std::chrono::steady_clock::now();
    while (!(h6.stat.module_status==1)){
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= 10000) {
            std::cout << "Timeout on pwm startup..." << std::endl;
        break;
        }
    }

    zcm.stop();

    //launch system monitor, consume this thread:
    execl ("bin/monitor", "bin/monitor",NULL);


    return 0;
}
