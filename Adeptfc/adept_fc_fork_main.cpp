//This code launches the autopilot framework
//Aaron Perry, 4/9/2019

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void run_process (const char* path){
    pid_t child_pid;

    /* Duplicate this process.  */
    child_pid = fork ();

    if (child_pid != 0){
        /* This is the parent process. Do nothing.  */
    }
    else {
        execl (path, NULL,NULL); // make sure memory allocation is appropriate in final application
        /* The execvp function returns only if an error occurs.  */
        printf ("an error occurred in execl\n");
        abort ();
    }

}


int main(int argc, char *argv[])
{

    //Fake sensor send module
    char send_path[50] = "/home/pi/adept-fc/HITL_Interface/sender"; //make sure path is robust for final imlementation
    run_process(send_path);
    printf("sender started\n");

    //Logger module
    char scribe_path[50] = "/home/pi/adept-fc/Scribe/scribe"; //make sure path is robust for final imlementation
    run_process(scribe_path);
    printf("scribe started\n");


    while(1){
        //keep the program running
    }

    return 0;
}
