//Reads in actuator commands and sends PWM commands out
//Aaron Perry, 4/15/2019
#include <zcm/zcm-cpp.hpp>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <memory>
//message types:
#include "pwm_t.hpp"
#include "rc_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        rc_t rc_in = {};
        status_t stat;

        Handler()
        {
            stat.should_exit = 0;
            stat.armed = 0;
        }


        void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
        {
            rc_in = *msg;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

std::unique_ptr <RCOutput> get_rcout()
{
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
}

int output_scaling(const int& in_val, const double& s_min, const double& s_max, const int& in_min, const int& in_max)
{
    return (s_min + (s_max-s_min)/(in_max-in_min)*(in_val-in_min));
}

int main(int argc, char *argv[])
{
    //load configuration variables
    string dump;
    int mapping[11] = {0,1,2,3,3,3,3,3,3,3,3};
    int num_outputs = 11;
    int pwm_freq = 50;
    int disarm_pwm_servo = 1500;
    int disarm_pwm_esc   = 1100;
    int servo_min = 1100;
    int servo_max = 1900;
    int rc_min = 1000;
    int rc_max = 1995;
    int mode_chan = 4;
    int mode_cutoff = 1500;
    std::ifstream config_stream;


    config_stream.open("config_files/pwm_out.config");
    config_stream >> dump >> pwm_freq;
    config_stream >> dump >> disarm_pwm_servo;
    config_stream >> dump >> disarm_pwm_esc;
    config_stream >> dump >> servo_min;
    config_stream >> dump >> servo_max;
    config_stream >> dump >> rc_min;
    config_stream >> dump >> rc_max;
    config_stream >> dump >> mode_chan;
    config_stream >> dump >> mode_cutoff;
    config_stream.close();
    mode_chan --;

    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //structures to publish
    pwm_t pwm_comm;

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);

    //initialize PWM outputs
    //************************************************************
    auto pwm = get_rcout();

    if (check_apm()) {
        return 1;
    }

    if (getuid()) {
        std::cout << "Not root. Please launch with Sudo." << std::endl;
    }

    for (int i=0;i<=num_outputs; i++)
    {

        if( !(pwm->initialize(i)) )
        {
            return 1;
        }

        pwm->set_frequency(i, pwm_freq);

        if ( !(pwm->enable(i)) )
        {
            return 1;
        }

        usleep(1000000); //without this, initialization of multiple channels fails
    }

    //set disarm pwm values
    for (int i = 0; i<=num_outputs; i++)
    {
       if (i<=2)
       {
            pwm_comm.pwm_out[i] = disarm_pwm_servo;
       }
       else
       {
            pwm_comm.pwm_out[i] = disarm_pwm_esc;
       }
    }

	//done initilizing PWM outputs
    //************************************************************

    zcm.start();

    while (!handlerObject.stat.should_exit)
    {


        if (handlerObject.stat.armed)
        {
            if (handlerObject.rc_in.rc_chan[mode_chan]<mode_cutoff) //manual flight mode:
            {
                 for (int i=0; i<=num_outputs-1; i++)
                {
                    pwm_comm.pwm_out[i] = output_scaling(handlerObject.rc_in.rc_chan[mapping[i]],servo_min,servo_max,rc_min,rc_max);
                    //superimpose maneuver:
                    pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                    std::cout << "manual flight mode" << std::endl;
                }
            }
            else if (handlerObject.rc_in.rc_chan[mode_chan]>=mode_cutoff) //auto flight mode:
            {
                for (int i=0; i<=num_outputs-1; i++)
                {
                    pwm_comm.pwm_out[i] = output_scaling(handlerObject.rc_in.rc_chan[mapping[i]],servo_min,servo_max,rc_min,rc_max);
                    //superimpose maneuver:
                    pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                    std::cout << "auto flight mode" << std::endl;
                }
            }
        }
        else
        {
            for (int i = 0; i<=num_outputs; i++)
            {
                if (i<=2)
                {
                    pwm_comm.pwm_out[i] = disarm_pwm_servo;
                }
                else
                {
                    pwm_comm.pwm_out[i] = disarm_pwm_esc;
                }
            }

        }

        //publish pwm values for logging
        zcm.publish("PWM_OUT", &pwm_comm);
    }

    std::cout << "pwm_out module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
