//This code is the HITL autopilot module that ESAERO will communicate with
//Aaron Perry, 4/1/2019
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <chrono>
//imessage types:
#include "sensor_data_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        adc_data_t adc;
        status_t stat;

        Handler()
        {
            memset(&adc,0,sizeof(adc));
            memset(&stat, 0, sizeof(stat));
        }

        void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
        {
            adc = *msg;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

double get_gps_time(Handler* adchandle)
{
    double adc_gps = adchandle->adc.time_gps;
    int64_t adc_time = adchandle->adc.time_rpi;
    int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return  adc_gps + (rpi_time - adc_time)/1000000;
}


int main(int argc, char *argv[])
{
    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //initialize message objects
    actuators_t acts;
    memset(&acts, 0, sizeof(acts));

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&handlerObject);

    //for bublishing stat of this module
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    //run zcm as a separate thread:
    zcm.start();

    std::cout << "autopilot started" << std::endl;
    //control loop:
    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS4",&module_stat);
        //compute actuator values:
        acts.de = 0;
        acts.da = 0;
        acts.dr = 0;
        acts.dt[0] = 1;
        acts.dt[1] = 0;
        acts.dt[2] = 0;
        acts.dt[3] = 0;
        acts.dt[4] = 0;
        acts.dt[5] = 0;
        acts.dt[6] = 0;
        acts.dt[7] = 0;
        usleep(5000);

        //timestamp the values:
        acts.time_gps = get_gps_time(&handlerObject);
        //publish the actuator values:
        zcm.publish("ACTUATORS", &acts);
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS4",&module_stat);

    std::cout << "autopilot module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
