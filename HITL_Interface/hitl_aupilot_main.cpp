//This code is the HITL autopilot module that ESAERO will communicate with
//copyright Aaron Perry, 4/1/2019

#include <stdio.h>
#include <unistd.h>
#include <zcm/zcm.h>
#include <pthread.h>

//input message types:
#include "../message_types/sensor_data_t.h"
//output message types:
#include "../message_types/actuators_t.h"


void actuate(zcm_t *zcm,actuators_t &acts)
{
    //send the values out:
    actuators_t_publish(zcm,"ACTUATORS", &acts);
}

void read_messages(const zcm_recv_buf_t  *rbuf, const char *channel,const sensor_data_t *sens,void *usr)
{
    //read each channel, move value to sensor_data variable
    sensor_data_t *sen_pointer = (sensor_data_t*) usr;
    sen_pointer->a_x = sens->a_x;
}

void *zcm_threader(void *args){
    //function to thread the zcm
    zcm_start((zcm_t*)args);
}


int main(int argc, char *argv[])
{
    //initialize zcm
    zcm_t *zcm = zcm_create("ipc");

    //initialize message objects
    actuators_t acts;
    sensor_data_t sensors;

    //subscribe to incoming channels:
    sensor_data_t_subscribe(zcm,"SENSOR_DATA",read_messages,&sensors);

    //run zcm as a separate thread:
    pthread_t zcm_thread;
    int t1;
    t1 = pthread_create(&zcm_thread,NULL,zcm_threader,(void*) zcm);
    pthread_join(t1,NULL);

    //control loop:
    while (1) {

        printf("x accel = %.2f\n",sensors.a_x);
        usleep(1000000);

        //compute actuator values:
        acts.de = 0;
        acts.da = 0;
        acts.dr = 0;
        acts.dt[0] = 0;
        acts.dt[1] = 0;
        acts.dt[2] = 0;
        acts.dt[3] = 0;
        acts.dt[4] = 0;
        acts.dt[5] = 0;
        acts.dt[6] = 0;
        acts.dt[7] = 0;

        //publish the actuator values:
        actuate(zcm, acts);
    }

    //clean exit
    pthread_exit(t1);
    zcm_destroy(zcm);
    return 0;
}
