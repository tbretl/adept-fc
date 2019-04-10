//This code is the HITL autopilot module that ESAERO will communicate with
//Aaron Perry, 4/1/2019

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
//input message types:
#include "../message_types/sensor_data_t.hpp"
//output message types:
#include "../message_types/actuators_t.hpp"
using std::string;

class Handler
{
    public:
        ~Handler() {}

        sensor_data_t sens = {};

        void read_messages(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            sens = *msg;
        }
};


int main(int argc, char *argv[])
{
    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //initialize message objects
    actuators_t acts;

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("SENSOR_DATA",&Handler::read_messages,&handlerObject);

    //run zcm as a separate thread:
    zcm.start();

    //control loop:
    while (1) {

        usleep(1000000);

        //compute actuator values:
        acts.de = 0;
        acts.da = 0;
        acts.dr = 0;
        acts.dt[0] = 0;
        acts.dt[1] = handlerObject.sens.a_x;
        acts.dt[2] = 0;
        acts.dt[3] = 0;
        acts.dt[4] = 0;
        acts.dt[5] = 0;
        acts.dt[6] = 0;
        acts.dt[7] = 0;

        //publish the actuator values:
        zcm.publish("ACTUATORS", &acts);
    }

    zcm.stop();

    return 0;
}
