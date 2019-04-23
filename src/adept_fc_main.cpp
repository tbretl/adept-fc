//This code launches the autopilot framework
//Aaron Perry, 4/9/2019

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>

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
        std::cout << "an error occurred in execl" << std::endl;
        abort ();
    }

}


int main(int argc, char *argv[])
{
    //read in configuration file:
    string line[2];
    bool hitl = false;
    ifstream config_stream;

    config_stream.open("config_files/adept_fc.config");
    config_stream >> line[0] >> line[1];
    hitl = !line[1].compare("true");
    config_stream.close();


    char exe_path[50];

    //rc_in
    strcpy(exe_path,"bin/rc_in");
    run_process(exe_path);
    std::cout<< "rc_in started" << std::endl;

    if(!hitl){
        //ADC sensor input

        //vectornav sensor input
        strcpy(exe_path,"bin/vnins");
        run_process(exe_path);
        std::cout << "VN-200 started" << std::endl;
    }
    else {
        //HITL interface
        strcpy(exe_path,"bin/interface");
        run_process(exe_path);
        std::cout << "HITL interface started" << std::endl;
    }

    //autopilot
    strcpy(exe_path,"bin/autopilot");
    run_process(exe_path);
    std::cout << "autopilot started" << std::endl;

    //Logger module
    strcpy(exe_path,"bin/scribe");
    run_process(exe_path);
    std::cout << "scribe started" << std::endl;

    //pwm_out
    strcpy(exe_path,"bin/pwm_out");
    run_process(exe_path);
    std::cout<< "pwm_out started" << std::endl << "wait for init..." << "\n.\n.\n.\n";
    usleep(15000000);



    //launch system monitor, consume this thread: -only include this if software will not be auto-starting.
    execl ("bin/monitor", NULL,NULL);

    return 0;
}
