//This code logs all of the data it receives to the SD card.
//copyright Aaron Perry, 4/1/2019

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <fstream>
//input message types:
#include "../message_types/sensor_data_t.hpp"
//output message types:
#include "../message_types/actuators_t.hpp"
using std::string;

class Handler
{
    public:
        ~Handler() {}

        char dataline[100] = {0};

        void read_messages(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            sprintf(dataline,"%.2f, %.2f, %.2f\n",msg->a_x,msg->a_y, msg->a_z);
            //file_name->write(dataline,sizeof(dataline));
            printf(dataline);
        }
};


int main(int argc, char *argv[])
{


    //sequencing file numbers:
    std::ifstream seqFile ("sequence.dat", std::ifstream::in);
    int fileNum;
    seqFile >> fileNum;
    seqFile.close();
    fileNum++;
    std::ofstream seqFile2 ("sequence.dat");
    seqFile2 << fileNum;
    seqFile2.close();

    //initialize the log file:
    char fileHold[20];
    sprintf(fileHold,"FlightLog_%i.dat",fileNum);
    char* fileName = fileHold;
    std::ofstream logfile (fileName,std::ofstream::binary);

    //initialize zcm
    zcm::ZCM zcm {"ipc"};


    //subscribe to incoming channels:
    Handler handlerObject;
    handlerObject.file_name = &logfile;
    zcm.subscribe("SENSOR_DATA",&Handler::read_messages,&handlerObject);

    //run zcm as a separate thread:
    zcm.start();

    //control loop:
    while (1) {
        //Possibility of adding start/stop commands here
        //use an infinite loop for now to keep logging going
    }

    zcm.stop();

    return 0;
}
