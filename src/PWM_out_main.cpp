//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//Reads in actuator commands and sends PWM commands out

#include <zcm/zcm-cpp.hpp>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <memory>
#include <chrono>

//message types:
#include "pwm_t.hpp"
#include "rc_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"

using std::string;

class Handler
{
public:
~Handler() = default;

rc_t rc_in;
status_t stat;
adc_data_t adc;
vnins_data_t vnins;

double acts[11]={0};
int mode_emergency = 0;
int rc_emergency = 0;
int64_t last_rc_time = 0;
int64_t last_act_time = 0;
int64_t last_vnins_time = 0;
int64_t last_adc_time = 0;

Handler()
{
        memset(&rc_in,0,sizeof(rc_in));
        memset(&stat,0,sizeof(stat));
        memset(&adc,0,sizeof(adc));
        memset(&vnins,0,sizeof(vnins));
}

void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
{
        rc_in = *msg;
        last_rc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
{
        adc = *msg;
        last_adc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
{
        vnins = *msg;
        last_vnins_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
{
        stat = *msg;
}

void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
{
        last_act_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        acts[0] = msg->da;
        acts[1] = msg->de;
        acts[2] = msg->dr;
        for (int i=3; i<11; i++)
        {
                acts[i] = msg->dt[i-3];
        }
}

};

std::unique_ptr <RCOutput> get_rcout()
{
        auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
        return ptr;
}

// Converts RC in command to PWM command
int output_scaling(const int& in_val, const double& s_min, const double& s_max, const int& in_min, const int& in_max)
{
        return (s_min + (s_max-s_min)/(in_max-in_min)*(in_val-in_min));
}

double get_gps_time(Handler* adchandle)
{
        double adc_gps = adchandle->adc.time_gps;
        int64_t adc_time = adchandle->adc.time_rpi;
        int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        return adc_gps + (rpi_time - adc_time)/1000000.0;
}


int main(int argc, char *argv[])
{
	// Initialize pwm output mapping variables
        int mapping[11] = {0,1,2,3,3,3,3,3,3,3,3};
        int num_outputs = 11;

        // Declare PWM output configuration variables
        string dump;
        std::ifstream config_stream;
	int pwm_freq;
        int disarm_pwm_servo;
        int disarm_pwm_esc;
        int servo_min;
        int servo_max;
        int rc_min;
        int rc_max;
	int ap_arm_chan;
	int ap_arm_cutoff;
	int at_arm_cutoff;
	int ap_engage_chan;
	int ap_engage_cutoff;

	// Load configuration variables
        config_stream.open("/home/pi/adept-fc/config_files/pwm_out.config");
        config_stream >> dump >> pwm_freq;
        config_stream >> dump >> disarm_pwm_servo;
        config_stream >> dump >> disarm_pwm_esc;
        config_stream >> dump >> servo_min;
        config_stream >> dump >> servo_max;
        config_stream >> dump >> rc_min;
        config_stream >> dump >> rc_max;
        config_stream >> dump >> ap_arm_chan;
        config_stream >> dump >> ap_arm_cutoff;
	config_stream >> dump >> at_arm_cutoff;
	config_stream >> dump >> ap_engage_chan;
	config_stream >> dump >> ap_engage_cutoff;
        config_stream.close();

	// Initialize ap arm and engage messege booleans
	bool ap_arm_messege_thrown = false;
	bool at_arm_messege_thrown = false;
	bool ap_engage_messege_thrown = false;

        //initialize zcm
        zcm::ZCM zcm {"ipc"};

        //structures to publish
        pwm_t pwm_comm;
        memset(&pwm_comm, 0, sizeof(pwm_comm));

        //subscribe to incoming channels:
        Handler handlerObject,sens_handler;
        zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);
        zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
        zcm.subscribe("ACTUATORS",&Handler::read_acts,&handlerObject);
        zcm.subscribe("ADC_DATA",&Handler::read_adc,&sens_handler);
        zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&sens_handler);

        //for publishing stat of this module
        status_t module_stat;
        memset(&module_stat,0,sizeof(module_stat));
        module_stat.module_status = 1;//module running

        //i Check PWM output channel status
        auto pwm = get_rcout();
        if (check_apm())
        {
                return 1;
        }

        if (getuid())
        {
                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << " Not root. Please launch with Sudo." << std::endl;
        }

	// Initialize PWM output channels
        for (int i=0; i<num_outputs; i++)
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

                usleep(50000); //without this, initialization of multiple channels fails
        }

        // Set output channels to disarm
        for (int i = 0; i<num_outputs; i++)
        {
		// Surface disarm commands
                if (i<=2)
                {
                        pwm_comm.pwm_out[i] = disarm_pwm_servo;
                }
				
		// Throttle disarm commands
                else
                {
                        pwm_comm.pwm_out[i] = disarm_pwm_esc;
                }
        }

	// Start PWM module
        zcm.start();
        std::cout<< "pwm_out started" << std::endl;

        // Initialize emergency message loss detection
        bool message_thrown[2] = {false,false};
        handlerObject.last_act_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        handlerObject.last_rc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        sens_handler.last_adc_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        sens_handler.last_vnins_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        // Check for an emergency startup
        std::ifstream f("/home/pi/adept-fc/emergency_startup");
        if (f.good())
        {
                handlerObject.stat.armed = 1;
                std::cout << "Armed on emergency startup" << std::endl;
        }

	// Module main loop
        while (!handlerObject.stat.should_exit)
        {
		// Publish status
                zcm.publish("STATUS6",&module_stat);

                // Collect current time (used to determine if communication link is still open between all other modules)
                int64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                
		// If we lose actuator, adc, or vnins data, flight mode is switched to manual
                if ((current_time-handlerObject.last_act_time < 500000) && (current_time-sens_handler.last_adc_time < 500000) && (current_time-sens_handler.last_vnins_time < 500000))
                {
			// No emergency detected
                        handlerObject.mode_emergency = 0;
                        message_thrown[1] = false;
                }
                else
                {
			// At least one module is lost
                        handlerObject.mode_emergency = 1;
                        if (message_thrown[1] == false)
                        {
                                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << " WARNING: Mode emergency detected. Switching to manual flight mode." << std::endl;
                                message_thrown[1] = true;
                        }
                }
				
		// If we lose the RC input, all surfaces and throttles are set to disarm
                if (current_time-handlerObject.last_rc_time < 500000)
                {
			// No emergency detected
                        handlerObject.rc_emergency = 0;
                        message_thrown[0] = false;
                }
                else
                {
			// RC module is lost
                        handlerObject.rc_emergency = 1;
                        if (message_thrown[0] == false)
                        {
                                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << " WARNING: RC emergency detected. Switching to disarm." << std::endl;
                                message_thrown[0] = true;
                        }
                }

                // If the PWM module is armed and the RC controller is detected, run PWM output logic
                if (handlerObject.stat.armed && handlerObject.rc_emergency == 0)
                {
			// Manual flight mode (if arm or engage channels are below minimum cutoffs, or if acts, vnins, or adc is lost)
                        if (handlerObject.rc_in.rc_chan[ap_arm_chan]<ap_arm_cutoff || handlerObject.rc_in.rc_chan[ap_engage_chan]<ap_engage_cutoff || handlerObject.mode_emergency == 1)
                        {		
				// Send disarm messeges
				if(at_arm_messege_thrown && handlerObject.rc_in.rc_chan[ap_arm_chan]<at_arm_cutoff)
				{
					std::cout<<"Autothrottle DISARMED." << std::endl;
					at_arm_messege_thrown = false;
				}
				else if(ap_arm_messege_thrown  && handlerObject.rc_in.rc_chan[ap_arm_chan]<ap_arm_cutoff)
				{
					std::cout<<"Autopilot DISARMED." << std::endl;
					ap_arm_messege_thrown = false;
				}
				
				// Send disengage messeges
				if(ap_engage_messege_thrown && at_arm_messege_thrown && handlerObject.rc_in.rc_chan[ap_engage_chan]<ap_engage_cutoff)
				{
					std::cout<<"Autopilot and Autothrottle DISENGAGED.\n" << std::endl;
					ap_engage_messege_thrown = false;
				}
				else if (ap_engage_messege_thrown && handlerObject.rc_in.rc_chan[ap_engage_chan]<ap_engage_cutoff)
				{
					std::cout<<"Autopilot DISENGAGED.\n" << std::endl;
					ap_engage_messege_thrown = false;
				}
				else if(ap_arm_messege_thrown && ap_engage_messege_thrown && handlerObject.rc_in.rc_chan[ap_arm_chan]<ap_arm_cutoff)
				{
					std::cout<<"Autopilot DISARMED WHILE ENGAGED." << std::endl;
					ap_arm_messege_thrown = true;
				}
				
				// Send armed messeges
				if(!at_arm_messege_thrown && handlerObject.rc_in.rc_chan[ap_arm_chan]>=at_arm_cutoff)
				{
					std::cout<<"Autothrottle ARMED." << std::endl;
					at_arm_messege_thrown = true;
				}
				else if(!ap_arm_messege_thrown  && handlerObject.rc_in.rc_chan[ap_arm_chan]>=ap_arm_cutoff)
				{
					std::cout<<"Autopilot ARMED." << std::endl;
					ap_arm_messege_thrown = true;
				}
							
				// Send pilot commands to aircraft
                                for (int i=0; i<num_outputs; i++)
                                {
                                        pwm_comm.pwm_out[i] = output_scaling(handlerObject.rc_in.rc_chan[mapping[i]],servo_min, servo_max, rc_min, rc_max);
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
								
								
                        }
						
			// Autopilot flight mode
                        else if (handlerObject.rc_in.rc_chan[ap_arm_chan]>=ap_arm_cutoff && handlerObject.rc_in.rc_chan[ap_arm_chan]<at_arm_cutoff && handlerObject.rc_in.rc_chan[ap_engage_chan]>=ap_engage_cutoff && handlerObject.mode_emergency == 0)
                        {
				// Send engage messeges
				if (!ap_engage_messege_thrown && ap_arm_messege_thrown)
				{
					std::cout << "Autopilot ENGAGED WHILE ARMED." << std::endl;
					ap_engage_messege_thrown = true;
				}
				else if (!ap_engage_messege_thrown && !ap_arm_messege_thrown)
				{
					std::cout << "Autopilot ARMED WHILE ENGAGED." << std::endl;
					ap_engage_messege_thrown = true;
					ap_arm_messege_thrown = true;
				}
				else if (ap_engage_messege_thrown && ap_arm_messege_thrown && at_arm_messege_thrown)
				{
					std::cout << "Autothrottle DISARMED WHILE ENGAGED Autopilot ENGAGED." << std::endl;
					at_arm_messege_thrown = false;
				}	
				else if (ap_engage_messege_thrown && !ap_arm_messege_thrown)
				{
					std::cout << "Autopilot ARMED WHILE ENGAGED." << std::endl;
					ap_arm_messege_thrown = true;
				}
								
                                for (int i = 0; i<num_outputs; i++)
                                {
					// Send AP surface commands to aircraft
					if (i<=2)
					{
						pwm_comm.pwm_out[i] = (int)(handlerObject.acts[i]);
						pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
					}
					
					// Send pilot throttle commands to aircraft
					else
					{
						pwm_comm.pwm_out[i] = output_scaling(handlerObject.rc_in.rc_chan[mapping[i]], servo_min, servo_max, rc_min, rc_max);
						pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
					}
                                }
                        }
						
			// Autopilot and autothrottle flight mode
			else if (handlerObject.rc_in.rc_chan[ap_arm_chan]>=at_arm_cutoff && handlerObject.rc_in.rc_chan[ap_engage_chan]>=ap_engage_cutoff && handlerObject.mode_emergency == 0)
			{
				// Send engage messeges
				if (!ap_engage_messege_thrown && ap_arm_messege_thrown && at_arm_messege_thrown)
				{
					std::cout << "Autopilot and Autothrottle ENGAGED WHILE ARMED." << std::endl;
					ap_engage_messege_thrown = true;
				}
				else if (ap_engage_messege_thrown && ap_arm_messege_thrown && !at_arm_messege_thrown)
				{
					std::cout << "Autothrottle ARMED WHILE ENGAGED." << std::endl;
					at_arm_messege_thrown = true;
				}
			
				// Send AP surface and throttle commands to aircraft
				for (int i = 0; i<num_outputs; i++)
				{
					pwm_comm.pwm_out[i] = (int)(handlerObject.acts[i]);
					pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
				}
			}
                }
				
		// If the PWM module is not armed, or the RC controller is not detected, set all outputs to disarm
                else
                {
			// Reset AP messeges
			ap_arm_messege_thrown = false;
			at_arm_messege_thrown = false;
			ap_engage_messege_thrown = false;
						
			// Disarm
                        for (int i = 0; i<num_outputs; i++)
                        {
				// Disarm surfaces
                                if (i<=2)
                                {
                                        pwm_comm.pwm_out[i] = disarm_pwm_servo;
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
								
				// Disarm throttles
                                else
                                {
                                        pwm_comm.pwm_out[i] = disarm_pwm_esc;
                                        pwm->set_duty_cycle(i, pwm_comm.pwm_out[i]);
                                }
                        }
                }
				
                // Timestamp the data
                pwm_comm.time_gps = get_gps_time(&sens_handler);

                // Publish pwm values for logging
                zcm.publish("PWM_OUT", &pwm_comm);
                usleep(10000);
        }

	// Finish and close
        module_stat.module_status = 0;
        zcm.publish("STATUS6",&module_stat);
        zcm.stop();
        std::cout << "pwm_out module exiting..." << std::endl;
        return 0;
}
