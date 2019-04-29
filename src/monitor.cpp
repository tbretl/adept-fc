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
#include "vnins_data_t.hpp"

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
        vnins_data_t vn200;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
            memset(&acts, 0, sizeof(acts));
            memset(&rc_in, 0, sizeof(rc_in));
            memset(&pwm, 0, sizeof(pwm));
            memset(&sens, 0, sizeof(sens));
            memset(&vn200, 0, sizeof(vn200));
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

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

        void read_sens(const zcm::ReceiveBuffer* rbuf,const string& chan,const sensor_data_t *msg)
        {
            sens = *msg;
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            vn200 = *msg;
        }
};



int main(int argc, char* argv[])
{
    int exit_flag = 0;
    string user_data[2];

    //initialize zcm:
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler message_handler,h0,h1,h2,h3,h4,h5,h6;
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&message_handler);
    zcm.subscribe("RC_IN",&Handler::read_rc,&message_handler);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&message_handler);
    zcm.subscribe("SENSOR_DATA",&Handler::read_sens,&message_handler);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&message_handler);
    //module status channels:
    zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    zcm.subscribe("STATUS6",&Handler::read_stat,&h6);

    //structures to publish:
    status_t sys_status;
    memset(&sys_status,0,sizeof(sys_status));

    sys_status.should_exit = 0;
    sys_status.armed = 0;

    zcm.start();
    zcm.flush();
    zcm.publish("STATUS",&sys_status);

    while (!exit_flag)
    {
        //get user input:
        std::cout <<"\n\n\nInput: <target> <action> (\"help !\" for options):" << std::endl;
        std::cin >> user_data[0] >> user_data[1];


        if (!user_data[0].compare("help"))
        {
            std::cout << "\n\n\n\n\n";
            std::cout << "list of modules and commands: \n" << std::endl;
            std::cout << "<all> - commands sent to all modules:" << std::endl;
            std::cout << "      <exit> - shuts everything down" << std::endl;
            std::cout << "      <status> - lists module run status\n" << std::endl;
            std::cout << "<monitor>" << std::endl;
            std::cout << "      <exit> - shuts monitor down" << std::endl;
            std::cout << "      <check> - performs pre-flight check\n" << std::endl;
            std::cout << "<pwm>" << std:: endl;
            std::cout << "      <arm> - enables pwm outputs, starts logging" << std::endl;
            std::cout << "      <disarm> - sets pwm outputs to disarm value, stops logging\n" << std::endl;


        }
        else if (!user_data[0].compare("all"))
        {
            if (!user_data[1].compare("exit"))
            {
                std::cout << "Exiting all modules." << std::endl;
                sys_status.should_exit = 1;
                exit_flag = 1;
                zcm.publish("STATUS",&sys_status);
                //wait for confirmation from modules:
                while (!(h0.stat.module_status==0 && h1.stat.module_status==0 && h2.stat.module_status==0
                      && h3.stat.module_status==0 && h4.stat.module_status==0 && h5.stat.module_status==0
                      && h6.stat.module_status==0)){}
                std::cout << "\nGoodbye.\n\n" ;
            }
            if (!user_data[1].compare("status"))
            {
                std::cout << "\nModule status:" << std::endl;
                if (h0.stat.module_status==1)
                {
                    std::cout << "  rc_in: running" << std::endl;
                }
                else if (h0.stat.module_status==0)
                {
                    std::cout << "  rc_in: stopped" << std::endl;
                }

                if (h1.stat.module_status==1)
                {
                    std::cout << "  vn_200: running" << std::endl;
                }
                else if (h1.stat.module_status==0)
                {
                    std::cout << "  vn_200: stopped" << std::endl;
                }

                if (h2.stat.module_status==1)
                {
                    std::cout << "  adc: running" << std::endl;
                }
                else if (h2.stat.module_status==0)
                {
                    std::cout << "  adc: stopped" << std::endl;
                }

                if (h3.stat.module_status==1)
                {
                    std::cout << "  hitl: running" << std::endl;
                }
                else if (h3.stat.module_status==0)
                {
                    std::cout << "  hitl: stopped" << std::endl;
                }

                if (h4.stat.module_status==1)
                {
                    std::cout << "  autopilot: running" << std::endl;
                }
                else if (h4.stat.module_status==0)
                {
                    std::cout << "  autopilot: stopped" << std::endl;
                }

                if (h5.stat.module_status==1)
                {
                    std::cout << "  scribe: running" << std::endl;
                }
                else if (h5.stat.module_status==0)
                {
                    std::cout << "  scribe: stopped" << std::endl;
                }

                if (h6.stat.module_status==1)
                {
                    std::cout << "  pwm_out: running" << std::endl;
                }
                else if (h6.stat.module_status==0)
                {
                    std::cout << "  pwm_out: stopped" << std::endl;
                }

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
                std::cout << "pwm outputs armed.\nlogging started..." << std::endl;
                zcm.publish("STATUS",&sys_status);
            }
            else if (!user_data[1].compare("disarm"))
            {
                sys_status.armed = 0;
                std::cout << "pwm outputs disarmed.\nlogging stopped." << std::endl;
                zcm.publish("STATUS",&sys_status);
            }
        }
    }

    zcm.stop();

    return 0;
}
