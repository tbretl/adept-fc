//test program to publish messages:

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
//message types:
#include "../message_types/sensor_data_t.hpp"
#include "../message_types/actuators_t.hpp"
#include "../message_types/status_t.hpp"

using std::string;

//object class to handle incoming messages
class Handler
{
    public:
        ~Handler() = default;

        //create a message object which can be accessed outside of the function call
        actuators_t acts = {};
        status_t stat;

        Handler()
        {
            stat.should_exit = 0;
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
    sensor_data_t msg;

    //subscribe to channels
    Handler handlerObject;
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);


    zcm.start();

    while(!handlerObject.stat.should_exit)
    {
        //publish a random number to the a_x sensor data
        msg.a_x = 0;
        msg.a_y = 1;
        msg.a_z = 2;
        zcm.publish("SENSOR_DATA", &msg);
        usleep(1000000);

    }

    std::cout << "HITL interface module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
