//test program to publish messages:

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <zcm/zcm.h>
#include "sensor_data_t.h"

int main()
{
    zcm_t *zcm = zcm_create("ipc");

    sensor_data_t msg;


    while(1){
        msg.a_x = rand();
        sensor_data_t_publish(zcm,"SENSOR_DATA",&msg);
        usleep(1000000);
    }

    zcm_destroy(zcm);
    return 0;
}
