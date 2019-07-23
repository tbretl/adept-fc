//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//This code logs all of the data it receives to the SD card.

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
        std::ostringstream log_buffer;
        int can_write = 1;
        int can_buff = 1;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
        }


        void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
        {
            if (can_buff == 1){
                can_write = 0;
                if(log_buffer << std::setprecision(14) << msg->time_gps << std::setprecision(6) <<" "){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
                }
                for (int i=0;i<8;i++)
                {
                    if(log_buffer << msg->rc_chan[i] << " "){
                    }else{
                        std::cout << "SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_pwm(const zcm::ReceiveBuffer* rbuf,const string& chan,const pwm_t *msg)
        {
            if (can_buff == 1){
                can_write =0;
                if(log_buffer << std::setprecision(14) << msg->time_gps << std::setprecision(6) <<" "){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
                }
                for (int i=0;i<11;i++)
                {
                    if(log_buffer << msg->pwm_out[i] << " "){
                    }else{
                        std::cout << "SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            if (can_buff == 1){
                can_write = 0;
                if(log_buffer << std::setprecision(14) << msg->time_gps << std::setprecision(6) << " "
                              << msg->da << " " << msg->de << " " << msg->dr << " "){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
                }
                for (int i=0;i<8;i++)
                {
                    if(log_buffer << msg->dt[i] << " "){
                    }else{
                        std::cout << "SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
        {
            if (can_buff == 1){
                can_write = 0;
                if(log_buffer << msg->time_gpspps << " " << std::setprecision(14) << msg->time_rpi << " " << msg->time_gps << std::setprecision(6)){
                }else {
                    std::cout << "SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                }
                for (int i=0; i<16; i++)
                {
                    if(log_buffer << " " << msg->data[i]){
                    }else {
                        std::cout << "SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else {
                    std::cout << "SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            if (can_buff == 1){
                can_write = 0;
                if(log_buffer << std::setprecision(6) << msg->time_gpspps << " " << std::setprecision(14) << msg->time << std::setprecision(6) << " " << msg->week << " "
                              << (int)msg->tracking << " " << (int)msg->gpsfix << " " << (int)msg->error <<  " "
                              << msg->roll << " " << msg->pitch << " " << msg->yaw << " " << std::setprecision(10) << msg->latitude << " "
                              << msg->longitude << std::setprecision(6) << " " << msg->altitude << " " << msg->vx << " " << msg->vy << " "
                              << msg->vz << " " << msg->attuncertainty << " " << msg->posuncertainty << " " << msg->veluncertainty << std::endl){
                }else{
                    std::cout << "SCRIBE ERROR: message buffer overflow on vnins." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

        string clear_buffer()
        {
            can_buff = 0;
            if (can_write == 1){
                string out = log_buffer.str();
                log_buffer.str("");
                can_buff = 1;
                return out;
            } else {
                can_buff = 1;
                return "";
            }
        }
};



int main(int argc, char *argv[])
{

    //initialize file objects
    char file_rc[25];
    char file_pwm[25];
    char file_acts[25];
    char file_adc[25];
    char file_vn200[25];

    //sequencing file numbers:
    std::ifstream seqFile ("config_files/sequence.dat", std::ifstream::in);
    int fileNum;
    seqFile >> fileNum;
    seqFile.close();
    fileNum++;

    sprintf(file_rc,"FlightLog_%i_rc.dat",fileNum);
    sprintf(file_pwm,"FlightLog_%i_pwm.dat",fileNum);
    sprintf(file_acts,"FlightLog_%i_acts.dat",fileNum);
    sprintf(file_adc,"FlightLog_%i_adc.dat",fileNum);
    sprintf(file_vn200,"FlightLog_%i_vn200.dat",fileNum);

    std::ofstream logfile_rc;
    std::ofstream logfile_pwm;
    std::ofstream logfile_acts;
    std::ofstream logfile_adc;
    std::ofstream logfile_vn200;

    logfile_rc.open(file_rc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_pwm.open(file_pwm,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_acts.open(file_acts,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_adc.open(file_adc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_vn200.open(file_vn200,std::ofstream::out | std::ofstream::app | std::ofstream::binary);

    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler vnins_handler,adc_handler,acts_handler,pwm_handler,rc_handler,handlerObject;
    zcm.subscribe("RC_IN",&Handler::read_rc,&rc_handler);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&pwm_handler);
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&acts_handler);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&adc_handler);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&vnins_handler);

    //for publishing stat of this module
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    zcm.start();

    std::cout << "scribe started" << std::endl;

    while (!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS5",&module_stat);
        usleep(25000); //40Hz

        if (handlerObject.stat.armed)
        {
            //log rc inputs
            logfile_rc << rc_handler.clear_buffer();
            //log pwm inputs
            logfile_pwm << pwm_handler.clear_buffer();
            //log actuator values
            logfile_acts << acts_handler.clear_buffer();
            //log sensor data
            logfile_adc << adc_handler.clear_buffer();
            //log VN200 data
            logfile_vn200 << vnins_handler.clear_buffer();

        }
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS5",&module_stat);

    std::cout << "scribe module exiting..." << std::endl;

    //close log files:
    logfile_rc.close();
    logfile_pwm.close();
    logfile_acts.close();
    logfile_adc.close();
    logfile_vn200.close();

    zcm.stop();

    return 0;
}
