//test program to publish messages:

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <zcm/zcm-cpp.hpp>
#include "../message_types/sensor_data_t.hpp"

int main()
{
    //zcm_t *zcm = zcm_create("ipc");
    zcm::ZCM zcm {"ipc"};

    sensor_data_t msg;

    zcm.start();

    while(1){
        msg.a_x = rand();
        //sensor_data_t_publish(zcm,"SENSOR_DATA",&msg);
        zcm.publish("SENSOR_DATA", &msg);
        usleep(1000000);
    }

    zcm.stop();

    //zcm_destroy(zcm);
    return 0;
}
