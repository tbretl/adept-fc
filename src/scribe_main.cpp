//This code logs all of the data it receives to the SD card.
//Aaron Perry, 4/1/2019

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <zcm/zcm-cpp.hpp>
#include <fstream>
//message types:
#include "../message_types/sensor_data_t.hpp"
#include "../message_types/actuators_t.hpp"
#include "../message_types/status_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() {
            logfile.close();
        }


        char fileName[20];
        char dataline[50];//need to fix: If this is too small, will result in segmentation fault

        status_t stat;

        Handler()
        {
            stat.should_exit = 0;
            //sequencing file numbers:
            std::ifstream seqFile ("config_files/sequence.dat", std::ifstream::in);
            int fileNum;
            seqFile >> fileNum;
            seqFile.close();
            fileNum++;
            std::ofstream seqFile2 ("config_files/sequence.dat");
            seqFile2 << fileNum;
            seqFile2.close();

            sprintf(fileName,"FlightLog_%i.dat",fileNum);
        }


        void read_messages(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            logfile.open(fileName,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            logfile << msg->a_x << " " << msg->a_y << " " << msg->a_z << "\n";
            logfile.close();
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
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
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);

    zcm.start();

    while (!handlerObject.stat.should_exit)
    {
        //run the code
    }

    std::cout << "scribe module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
