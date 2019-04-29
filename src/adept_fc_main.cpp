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
    auto* sub0 = zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    auto* sub1 = zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    auto* sub2 = zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    auto* sub3 = zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    auto* sub4 = zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    auto* sub5 = zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    auto* sub6 = zcm.subscribe("STATUS6",&Handler::read_stat,&h6);

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
    while (!(h0.stat.module_status==1)){}
    std::cout<< "rc_in started" << std::endl;

    if(!hitl){

        //vectornav sensor input
        run_process("bin/vnins");
        while (!(h1.stat.module_status==1)){} //vn20 module not publishing... fix
        std::cout << "VN-200 started" << std::endl;

        //ADC sensor input

    }
    else {
        //HITL interface
        run_process("bin/hitl");
        while (!(h3.stat.module_status==1)){}
        std::cout << "hitl started" << std::endl;
    }

    //autopilot
    run_process("bin/autopilot");
    while (!(h4.stat.module_status==1)){}
    std::cout << "autopilot started" << std::endl;

    //Logger module
    run_process("bin/scribe");
    while (!(h5.stat.module_status==1)){}
    std::cout << "scribe started" << std::endl;

    //pwm_out
    run_process("bin/pwm_out");
    while (!(h6.stat.module_status==1)){}
    std::cout<< "pwm_out started" << std::endl;

    zcm.unsubscribe(sub0);
    zcm.unsubscribe(sub1);
    zcm.unsubscribe(sub2);
    zcm.unsubscribe(sub3);
    zcm.unsubscribe(sub4);
    zcm.unsubscribe(sub5);
    zcm.unsubscribe(sub6);


    zcm.stop();

    //launch system monitor, consume this thread:
    execl ("bin/monitor", "bin/monitor",NULL);



    return 0;
}
