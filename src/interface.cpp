//test program to publish messages:
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <zcm/zcm-cpp.hpp>
//message types:
#include "adc_data_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"

using std::string;

//object class to handle incoming messages
class Handler
{
    public:
        ~Handler() = default;

        actuators_t acts;
        status_t stat;

        Handler()
        {
            memset(&acts, 0, sizeof(acts));
            memset(&stat, 0, sizeof(stat));
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            acts = *msg;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

int main()
{
    //Initialize zcm
    zcm::ZCM zcm {"ipc"};

    //create objects to publish
    adc_data_t msg;
    memset(&msg, 0, sizeof(msg));

    //subscribe to channels
    Handler handlerObject;
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);

    //for bublishing stat of this module
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running


    zcm.start();

    std::cout << "hitl started" << std::endl;

    while(!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS3",&module_stat);

        //in hitl mode, use RPI time as GPS time
        msg.time_gps = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        zcm.publish("ADC_DATA", &msg);
        usleep(5000);

    }

    module_stat.module_status = 0;
    zcm.publish("STATUS3",&module_stat);

    std::cout << "HITL interface module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
