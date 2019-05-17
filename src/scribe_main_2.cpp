//This code logs all of the data it receives to the SD card.
//Aaron Perry, 4/1/2019

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <zcm/zcm-cpp.hpp>
#include <fstream>
#include <iomanip>
//message types:
#include "vnins_data_t.hpp"
#include "adc_data_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "rc_t.hpp"
#include "pwm_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;
        rc_t rc_in;
        pwm_t pwm;
        actuators_t acts;
        adc_data_t sens;
        vnins_data_t vn200;



        Handler()
        {
            memset(&stat, 0, sizeof(stat));
            memset(&rc_in, 0, sizeof(rc_in));
            memset(&pwm, 0, sizeof(pwm));
            memset(&acts, 0, sizeof(acts));
            memset(&sens, 0, sizeof(sens));
            memset(&vn200, 0, sizeof(vn200));
            stat.should_exit = 0;

        }


        void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
        {
            rc_in = *msg;
        }

        void read_pwm(const zcm::ReceiveBuffer* rbuf,const string& chan,const pwm_t *msg)
        {
            pwm = *msg;
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            acts = *msg;
        }

        void read_sens(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
        {
            sens = *msg;
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            vn200 = *msg;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }


};



int main(int argc, char *argv[])
{

    //initialize file objects
    char file_rc[25];
    char file_pwm[25];
    char file_acts[25];
    char file_sens[25];
    char file_vn200[25];

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

    std::ofstream logfile_rc;
    std::ofstream logfile_pwm;
    std::ofstream logfile_acts;
    std::ofstream logfile_sens;
    std::ofstream logfile_vn200;

    logfile_rc.open(file_rc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_pwm.open(file_pwm,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_acts.open(file_acts,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_sens.open(file_sens,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_vn200.open(file_vn200,std::ofstream::out | std::ofstream::app | std::ofstream::binary);

    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&handlerObject);
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
    zcm.subscribe("ADC_DATA",&Handler::read_sens,&handlerObject);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&handlerObject);

    //for bublishing stat of this module
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    zcm.start();

    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS5",&module_stat);
        usleep(25000); //40 Hz for now

        if (handlerObject.stat.armed)
        {
            //log rc inputs
            for (int i=0;i<8;i++)
            {
                logfile_rc << handlerObject.rc_in.rc_chan[i] << " ";
            }
            logfile_rc << "\n";
            //log pwm inputs
            for (int i=0;i<11;i++)
            {
                logfile_pwm << handlerObject.pwm.pwm_out[i] << " ";
            }
            logfile_pwm << "\n";
            //log actuator values
            logfile_acts << handlerObject.acts.da << " " << handlerObject.acts.de << " " << handlerObject.acts.dr << " ";
            for (int i=0;i<8;i++)
            {
                logfile_acts << handlerObject.acts.dt[i] << " ";
            }
            logfile_acts << "\n";
            //log sensor data
            logfile_sens << handlerObject.sens.time_gpspps << " " << handlerObject.sens.time_gps;
            for (int i=0; i<16; i++)
            {
                logfile_sens << " " << handlerObject.sens.data[i];
            }
            logfile_sens << "\n";
            //log VN200 data
            logfile_vn200 << std::setprecision(14) << handlerObject.vn200.time << std::setprecision(6) << " " << handlerObject.vn200.week << " " << handlerObject.vn200.tracking << " " << handlerObject.vn200.gpsfix << " " << handlerObject.vn200.error <<  " "
                          << handlerObject.vn200.pitch << " " << handlerObject.vn200.roll << " " << handlerObject.vn200.yaw << " " << handlerObject.vn200.latitude << " "
                          << handlerObject.vn200.longitude << " " << handlerObject.vn200.altitude << " " << handlerObject.vn200.vx << " " << handlerObject.vn200.vy << " "
                          << handlerObject.vn200.vz << " " << handlerObject.vn200.attuncertainty << " " << handlerObject.vn200.posuncertainty << " " << handlerObject.vn200.veluncertainty << "\n";
        }
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS5",&module_stat);

    std::cout << "scribe module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    //close log files:
    logfile_rc.close();
    logfile_pwm.close();
    logfile_acts.close();
    logfile_sens.close();
    logfile_vn200.close();

    zcm.stop();

    return 0;
}
