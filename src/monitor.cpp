//this will be the terminal interface to the rest of the modules
//Aaron Perry, 4/19/2019
//input options: preflight-check command, query any channel, restart a module (safety feature),  clean exit to watchdog mode, clean exit of whole autopilot

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "zcm/zcm-cpp.hpp"
//message types:
#include "pwm_t.hpp"
#include "rc_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "sensor_data_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        sensor_data_t sens;
        rc_t rc_in;
        pwm_t pwm;
        actuators_t acts;
        status_t stat;

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

        void read_status(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

        void read_sens(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            sens = *msg;
        }
};



int main(int argc, char* argv[])
{
    int exit_flag = 0;
    string user_data[2];

    //initialize zcm:
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler message_handler;
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&message_handler);
    zcm.subscribe("RC_IN",&Handler::read_rc,&message_handler);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&message_handler);
    zcm.subscribe("SENSOR_DATA",&Handler::read_sens,&message_handler);
    zcm.subscribe("STATUS",&Handler::read_status,&message_handler);

    //structures to publish:
    status_t sys_status;
    sys_status.should_exit = 0;
    sys_status.armed = 0;


    //run zcm as a separate thread:
    zcm.start();
    zcm.publish("STATUS",&sys_status);

    while (!exit_flag)
    {
        //get user input:
        std::cout <<"Input: <target> <action> (\"help !\" for options):" << std::endl;
        std::cin >> user_data[0] >> user_data[1];


        if (!user_data[0].compare("help"))
        {
            std::cout << "\n\n\n\n\n";
            std::cout << "list of modules and commands: \n" << std::endl;
            std::cout << "<all> - commands sent to all modules:" << std::endl;
            std::cout << "      <exit> - shuts everything down\n" << std::endl;
            std::cout << "<monitor>" << std::endl;
            std::cout << "      <exit> - shuts monitor down" << std::endl;
            std::cout << "      <check> - performs pre-flight check\n" << std::endl;
            std::cout << "<pwm>" << std:: endl;
            std::cout << "      <arm> - enables pwm outputs" << std::endl;
            std::cout << "      <disarm> - sets pwm outputs to disarm value\n" << std::endl;


        }
        else if (!user_data[0].compare("all"))
        {
            if (!user_data[1].compare("exit"))
            {
                std::cout << "Exiting all modules." << std::endl;
                sys_status.should_exit = 1;
                exit_flag = 1;
                zcm.publish("STATUS",&sys_status);
                usleep(5000000);
            }
        }
        else if (!user_data[0].compare("monitor"))
        {
            if (!user_data[1].compare("exit"))
            {
                std::cout << "Exiting monitor: goodbye!." << std::endl;
                exit_flag = 1;
            }
        }
        else if (!user_data[0].compare("pwm"))
        {
            if (!user_data[1].compare("arm"))
            {
                sys_status.armed = 1;
                std::cout << "pwm outputs armed." << std::endl;
                zcm.publish("STATUS",&sys_status);
            }
            else if (!user_data[1].compare("disarm"))
            {
                sys_status.armed = 0;
                std::cout << "pwm outputs disarmed." << std::endl;
                zcm.publish("STATUS",&sys_status);
            }
        }
    }

    zcm.stop();

    return 0;
}
