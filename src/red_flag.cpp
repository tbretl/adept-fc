//keeps track of process ID's and CPU usage.
//Aaron Perry 6/4/2019
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <iomanip>
#include <sys/types.h>
#include <dirent.h>
#include "zcm/zcm-cpp.hpp"
//message types:
#include "pwm_t.hpp"
#include "rc_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"

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
            return true;
        }
    }
    return false;
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
//        int temp[] = {0,1,4,5,6,6};
//        name_inds = temp;
    }else{//flight mode
//        int temp[] = {0,2,3,4,5,6};
//        name_inds = temp;
    }


    //length of list determines order of PIDS (hitl vs no hitl)
    //cycle through list and check
    //cout an error if process dies

    //initialize zcm:
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

    zcm.start();

    std::cout << "red_flag started" << std::endl;

    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS7",&module_stat);
        usleep(500000); //2Hz
        //check each module status

        //get cpu load
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS7",&module_stat);

    std::cout << "red_flag exiting..." << std::endl;

    zcm.stop();

    return 0;
}
