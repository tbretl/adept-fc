//This code logs all of the data it receives to the SD card.
//Aaron Perry, 4/1/2019

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <zcm/zcm-cpp.hpp>
#include <fstream>
#include <ctime>
//message types:
#include "../message_types/vnins_data_t.hpp"  //need to test out
#include "sensor_data_t.hpp" //need to finalize
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "rc_t.hpp"
#include "pwm_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;


        char file_rc[25];
        char file_pwm[25];
        char file_acts[25];
        char file_sens[25];
        char file_vn200[25];

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

            sprintf(file_rc,"FlightLog_%i_rc.dat",fileNum);
            sprintf(file_pwm,"FlightLog_%i_pwm.dat",fileNum);
            sprintf(file_acts,"FlightLog_%i_acts.dat",fileNum);
            sprintf(file_sens,"FlightLog_%i_sens.dat",fileNum);
            sprintf(file_vn200,"FlightLog_%i_vn200.dat",fileNum);
        }


        void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
        {
            logfile.open(file_rc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            for (int i=0;i<8;i++)
            {
               logfile << msg->rc_chan[i] << " ";
            }
            logfile << "\n";
            logfile.close();
        }

        void read_pwm(const zcm::ReceiveBuffer* rbuf,const string& chan,const pwm_t *msg)
        {
            logfile.open(file_pwm,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            for (int i=0;i<11;i++)
            {
               logfile << msg->pwm_out[i] << " ";
            }
            logfile << "\n";
            logfile.close();
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            logfile.open(file_acts,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            logfile << msg->da << " " << msg->de << " " << msg->dr << " ";
            for (int i=0;i<8;i++)
            {
               logfile << msg->dt[i] << " ";
            }
            logfile << "\n";
            logfile.close();
        }

        void read_sens(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            logfile.open(file_sens,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            logfile << msg->Vmag << " " << msg->alpha << " " << msg->beta << " " << msg->l_ail << " "
                    << msg->r_ail << " " << msg->l_ele << " " << msg->r_ele << " " << msg->rud << "\n";
            logfile.close();
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            logfile.open(file_vn200,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
            logfile << msg->time << " " << msg->week << " " << msg->tracking << " " << msg->gpsfix << " " << msg->error <<  " "
                    << msg->pitch << " " << msg->roll << " " << msg->yaw << " " << msg->latitude << " "
                    << msg->longitude << " " << msg->altitude << " " << msg->vx << " " << msg->vy << " "
                    << msg->vz << " " << msg->attuncertainty << " " << msg->posuncertainty << " " << msg->veluncertainty << "\n";
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
    zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&handlerObject);
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
    zcm.subscribe("SENSOR_DATA",&Handler::read_sens,&handlerObject);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&handlerObject);

    time_t tstart, tend;
    tstart = time(0);
    zcm.start();

    while (!handlerObject.stat.should_exit)
    {
        //run the code
    }

    std::cout << "scribe module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();
    tend = time(0);
    std::cout << "time logging: " << difftime(tend,tstart) << std::endl;
    return 0;
}
