//test program to publish messages:

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include "../message_types/sensor_data_t.hpp"
#include "../message_types/actuators_t.hpp"
using std::string;

//object class to handle incoming messages
class Handler
{
    public:
        ~Handler() {}

        //create a message object which can be accessed outside of the function call
        actuators_t acts = {};

        void read_messages(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            //assign message value to accessible message object
            acts = *msg;
        }
};

int main()
{
    //Initialize zcm
    zcm::ZCM zcm {"ipc"};

    //create objects to publish
    sensor_data_t msg;

    //subscribe to channels
    Handler handlerObject;
    zcm.subscribe("ACTUATORS",&Handler::read_messages,&handlerObject);


    zcm.start();

    while(1){
        //publish a random number to the a_x sensor data
        msg.a_x = rand();
        zcm.publish("SENSOR_DATA", &msg);
        usleep(1000000);
        //print actuator dt[1] value returned from autopilot:
        //printf("dt_1 = %.2f\n",handlerObject.acts.dt[1]);
    }

    zcm.stop();

    return 0;
}
