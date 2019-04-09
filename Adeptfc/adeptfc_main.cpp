//This code launches the autopilot framework
//Aaron Perry, 4/9/2019

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

void *task_threader(void *args){
    //call to execl
    execl((char*) args,NULL,NULL);
    printf("execl failed for: %s\n",(char*) args);
}


int main(int argc, char *argv[])
{

    //Fake sensor send module
    pthread_t send_thread;
    char send_path[50] = "/home/pi/adept-fc/HITL_Interface/sender"; //make sure path is robust for final imlementation
    pthread_create(&send_thread, NULL, task_threader, &send_path);
    printf("sender started\n");


    //Logger module
    pthread_t scribe_thread;
    char scribe_path[50] = "/home/pi/adept-fc/Scribe/scribe"; //make sure path is robust for final imlementation
    pthread_create(&scribe_thread, NULL, task_threader, &scribe_path);
    printf("scribe started\n");

    pthread_detach(send_thread);
    pthread_detach(scribe_thread);

    while(1){
        //keep the program running
        printf("in loop\n");

    }

    return 0;
}
