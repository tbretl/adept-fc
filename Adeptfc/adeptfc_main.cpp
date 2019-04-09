//This code launches the autopilot framework
//copyright Aaron Perry, 4/9/2019

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

    //Logger module
    pthread_t scribe_thread;
    char scribe_path[50] = "/home/pi/adept-fc/Scribe/scribe"; //make sure path is robust for final imlementation
    pthread_create(&scribe_thread, NULL, task_threader, &scribe_path);
    pthread_detach(scribe_thread);


    while(1){
        //keep the program running

    }

    return 0;
}
