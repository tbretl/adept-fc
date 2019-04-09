//This code logs all of the data it receives to the SD card.
//Aaron Perry, 4/1/2019

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <cstring>
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
        ~Handler() {
            logfile.close();
        }


        char fileName[20];
        char dataline[50];//need to fix: If this is too small, will result in segmentation fault

        Handler()
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

            sprintf(fileName,"FlightLog_%i.dat",fileNum);
        }


        void read_messages(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
//            sprintf(dataline,"%.2f, %.2f, %.2f\n",msg->a_x,msg->a_y, msg->a_z);
            logfile.open(fileName,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            logfile << msg->a_x << " " << msg->a_y << " " << msg->a_z << "\n";
//            logfile.write(dataline,sizeof(dataline));
            logfile.close();
        }

    private:
    std::ofstream logfile;
};



int main(int argc, char *argv[])
{
    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("SENSOR_DATA",&Handler::read_messages,&handlerObject);

    //run zcm as a separate thread:
    zcm.start();

    while (1) {
        //Possibility of adding start/stop commands here
        //use an infinite loop for now to keep logging going
        //consider interaction with telemetry radio?
    }

    zcm.stop();

    return 0;
}
