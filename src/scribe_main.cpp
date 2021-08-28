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
#include <chrono>
//message types:
#include "autopilot_t.hpp"
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
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
                }
                for (int i=0;i<8;i++)
                {
                    if(log_buffer << msg->rc_chan[i] << " "){
                    }else{
                        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                        " SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on rc_in." << std::endl;
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
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
                }
                for (int i=0;i<11;i++)
                {
                    if(log_buffer << msg->pwm_out[i] << " "){
                    }else{
                        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                        " SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on pwm." << std::endl;
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
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
                }
                for (int i=0;i<8;i++)
                {
                    if(log_buffer << msg->dt[i] << " "){
                    }else{
                        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                        " SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else{
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on actuators." << std::endl;
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
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                }
                for (int i=0; i<16; i++)
                {
                    if(log_buffer << " " << msg->data[i]){
                    }else {
                        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                        " SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                    }
                }
                if(log_buffer << std::endl){
                }else {
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on adc." << std::endl;
                }
                can_write = 1;
            }
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            if (can_buff == 1){
                can_write = 0;
                if(log_buffer << std::setprecision(9) << msg->time_gpspps << " " << std::setprecision(14) << msg->time << std::setprecision(6) << " "
                              << (int)msg->tracking << " " << (int)msg->gpsfix << " " << (int)msg->error <<  " "
                              << msg->roll << " " << msg->pitch << " " << msg->yaw << " " << msg->wx << " " << msg->wy << " " << msg->wz << " " << std::setprecision(10) << msg->latitude << " "
                              << msg->longitude << std::setprecision(6) << " " << msg->altitude << " " << msg->vn << " " << msg->ve << " "
                              << msg->vd << " " << msg->ax << " " << msg->ay << " " << msg->az << std::endl){
                }else{
                    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                    " SCRIBE ERROR: message buffer overflow on vnins." << std::endl;
                }
                can_write = 1;
            }
        }
	
	
	

	
	
        void read_autopilot(const zcm::ReceiveBuffer* rbuf,const string& chan,const autopilot_t *msg)
        {
            if (can_buff == 1){
		    
                can_write = 0;
		#ifdef TEST
			if( log_buffer << std::setprecision(14) << msg->time_gps << " " << std::setprecision(1) << msg->ap_armed_and_engaged << std::setprecision(6) << " "
				      << msg->ele_trim << " " << msg->ail_trim << " " << msg->rud_trim << " " 
				      << msg->thr_trim[0] << " " << msg->thr_trim[1] << " " << msg->thr_trim[2] << " " << msg->thr_trim[3] << " " << msg->thr_trim[4] << " " << msg->thr_trim[5] << " " << msg->thr_trim[6] << " " << msg->thr_trim[7] << " "
				      << msg->yaw_trim_rate << " " 
				      << msg->adc_pres_raw[0] << " " << msg->adc_pres_raw[1] << " " << msg->adc_pres_raw[2] << " " << msg->adc_pres_raw[3] << " " << msg->adc_pres_raw[4] << " "
				      << msg->adc_pres_dPSI[0] << " " << msg->adc_pres_dPSI[1] << " " << msg->adc_pres_dPSI[2] << " " << msg->adc_pres_dPSI[3] << " " << msg->adc_pres_dPSI[4] << " "
				      << msg->p_bar_dPSI << " " << msg->Ca << " " << msg->Cb << " " << msg->Cpt << " " << msg->Cps << " " << msg->Pt_Pa << " " << msg->Ps_Pa << " " << msg->rho << " " 
				      << msg->state[0] << " " << msg->state[1] << " " << msg->state[2] << " " << msg->state[3] << " " << msg->state[4] << " " << msg->state[5] << " " << msg->state[6] << " " << msg->state[7] << " " << msg->state[8] << " " 
				      << msg->state_error[0] << " " << msg->state_error[1] << " " << msg->state_error[2] << " " << msg->state_error[3] << " " << msg->state_error[4] << " " << msg->state_error[5] << " " << msg->state_error[6] << " " << msg->state_error[7] << " " << msg->state_error[8] << " " 
				      << msg->integral_ref[0] << " " << msg->integral_ref[1] << " " << msg->integral_ref[2] << " " << msg->integral[0] << " " << msg->integral[1] << " " << msg->integral[2] << " " 
				      << msg->input_cmd[0] << " " << msg->input_cmd[1] << " " << msg->input_cmd[2] << " " 
				      << msg->input_cmd[3] << " " << msg->input_cmd[4] << " " << msg->input_cmd[5] << " " << msg->input_cmd[6] << " " << msg->input_cmd[7] << " " << msg->input_cmd[8] << " " << msg->input_cmd[9] << " " << msg->input_cmd[10]
				      << msg->true_state[0] << " " << msg->true_state[1] << " " << msg->true_state[2] << " " << msg->true_state[3] << " " << msg->true_state[4] << " " << msg->true_state[5] << " " << msg->true_state[6] << " " << msg->true_state[7] << " " << msg->true_state[8] << " " 
				      << std::endl ){
			}else{
			    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
			    " SCRIBE ERROR: message buffer overflow on autopilot." << std::endl;
			}
		#else
			if( log_buffer << std::setprecision(14) << msg->time_gps << " " << std::setprecision(1) << msg->ap_armed_and_engaged << std::setprecision(6) << " "
				      << msg->ele_trim << " " << msg->ail_trim << " " << msg->rud_trim << " " 
				      << msg->thr_trim[0] << " " << msg->thr_trim[1] << " " << msg->thr_trim[2] << " " << msg->thr_trim[3] << " " << msg->thr_trim[4] << " " << msg->thr_trim[5] << " " << msg->thr_trim[6] << " " << msg->thr_trim[7] << " "
				      << msg->yaw_trim_rate << " " 
				      << msg->adc_pres_raw[0] << " " << msg->adc_pres_raw[1] << " " << msg->adc_pres_raw[2] << " " << msg->adc_pres_raw[3] << " " << msg->adc_pres_raw[4] << " "
				      << msg->adc_pres_dPSI[0] << " " << msg->adc_pres_dPSI[1] << " " << msg->adc_pres_dPSI[2] << " " << msg->adc_pres_dPSI[3] << " " << msg->adc_pres_dPSI[4] << " "
				      << msg->p_bar_dPSI << " " << msg->Ca << " " << msg->Cb << " " << msg->Cpt << " " << msg->Cps << " " << msg->Pt_Pa << " " << msg->Ps_Pa << " " << msg->rho << " " 
				      << msg->state[0] << " " << msg->state[1] << " " << msg->state[2] << " " << msg->state[3] << " " << msg->state[4] << " " << msg->state[5] << " " << msg->state[6] << " " << msg->state[7] << " " << msg->state[8] << " " 
				      << msg->state_error[0] << " " << msg->state_error[1] << " " << msg->state_error[2] << " " << msg->state_error[3] << " " << msg->state_error[4] << " " << msg->state_error[5] << " " << msg->state_error[6] << " " << msg->state_error[7] << " " << msg->state_error[8] << " " 
				      << msg->integral_ref[0] << " " << msg->integral_ref[1] << " " << msg->integral_ref[2] << " " << msg->integral[0] << " " << msg->integral[1] << " " << msg->integral[2] << " " 
				      << msg->input_cmd[0] << " " << msg->input_cmd[1] << " " << msg->input_cmd[2] << " " 
				      << msg->input_cmd[3] << " " << msg->input_cmd[4] << " " << msg->input_cmd[5] << " " << msg->input_cmd[6] << " " << msg->input_cmd[7] << " " << msg->input_cmd[8] << " " << msg->input_cmd[9] << " " << msg->input_cmd[10]
				      << std::endl ){
			}else{
			    std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
			    " SCRIBE ERROR: message buffer overflow on autopilot." << std::endl;
			}
		#endif
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
    char file_autopilot[30];

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
    sprintf(file_autopilot,"FlightLog_%i_autopilot.dat",fileNum);

    std::ofstream logfile_rc;
    std::ofstream logfile_pwm;
    std::ofstream logfile_acts;
    std::ofstream logfile_adc;
    std::ofstream logfile_vn200;
    std::ofstream logfile_autopilot;

    logfile_rc.open(file_rc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_pwm.open(file_pwm,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_acts.open(file_acts,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_adc.open(file_adc,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_vn200.open(file_vn200,std::ofstream::out | std::ofstream::app | std::ofstream::binary);
    logfile_autopilot.open(file_autopilot,std::ofstream::out | std::ofstream::app | std::ofstream::binary);

    //write headers
    #ifdef TEST
	logfile_autopilot << "gps_time[s] engaged[-]"
				 " ele_trim[deg] ail_trim[deg] rud_trim[deg] t1_trim[%] t2_trim[%] t3_trim[%] t4_trim[%] t5_trim[%] t6_trim[%] t7_trim[%] t8_trim[%]" 
				 " yaw_trim_rate[deg/s]"
				 " p1[ADC] p2[ADC] p3[ADC] p4[ADC] p5[ADC] p1[dPSI] p2[dPSI] p3[dPSI] p4[dPSI] p5[dPSI] p_bar[dPSI] Ca[-] Cb[-] Cpt[-] Cps[-] Pt[Pa] Ps[Pa] rho[kg/m^3]" 
				 " vel[m/s] aoa[deg] wy[deg/s] pitch[deg] beta[deg] wx[deg/s] wz[deg/s] roll[deg] yaw[deg]"
				 " vel_error[m/s] aoa_error[deg] wy_error[deg/s] pitch_error[deg] beta_error[deg] wx_error[deg/s] wz_error[deg/s] roll_error[deg] yaw_error[deg]"
				 " vel_ref[m/s] pitch_ref[deg] yaw_ref[deg] vel_int[m] pitch_int[deg-s] yaw_int[deg-s]" 
				 " ele_cmd[deg] ail_cmd[deg] rud_cmd[deg] t1_cmd[%] t2_cmd[%] t3_cmd[%] t4_cmd[%] t5_cmd[%] t6_cmd[%] t7_cmd[%] t8_cmd[%]"
				 " true_vel[m/s] true_aoa[deg] true_wy[deg/s] true_pitch[deg] true_beta[deg] true_wx[deg/s] true_wz[deg/s] true_roll[deg] true_yaw[deg]" << std::endl;

    #else
	logfile_autopilot << "gps_time[s] engaged[-]"
				 " ele_trim[deg] ail_trim[deg] rud_trim[deg] t1_trim[%] t2_trim[%] t3_trim[%] t4_trim[%] t5_trim[%] t6_trim[%] t7_trim[%] t8_trim[%]" 
				 " yaw_trim_rate[deg/s]"
				 " p1[ADC] p2[ADC] p3[ADC] p4[ADC] p5[ADC] p1[dPSI] p2[dPSI] p3[dPSI] p4[dPSI] p5[dPSI] p_bar[dPSI] Ca[-] Cb[-] Cpt[-] Cps[-] Pt[Pa] Ps[Pa] rho[kg/m^3]" 
				 " vel[m/s] aoa[deg] wy[deg/s] pitch[deg] beta[deg] wx[deg/s] wz[deg/s] roll[deg] yaw[deg]"
				 " vel_error[m/s] aoa_error[deg] wy_error[deg/s] pitch_error[deg] beta_error[deg] wx_error[deg/s] wz_error[deg/s] roll_error[deg] yaw_error[deg]"
				 " vel_ref[m/s] pitch_ref[deg] yaw_ref[deg] vel_int[m] pitch_int[deg-s] yaw_int[deg-s]" 
				 " ele_cmd[deg] ail_cmd[deg] rud_cmd[deg] t1_cmd[%] t2_cmd[%] t3_cmd[%] t4_cmd[%] t5_cmd[%] t6_cmd[%] t7_cmd[%] t8_cmd[%]" << std::endl;
    #endif
		     
    logfile_vn200 << "gps_pps[ns] gps_time[s] gps_tracking gps_fix gps_error roll[deg] pitch[deg] yaw[deg] wx[rad/s] wy[rad/s] wz[rad/s]"
                     " lat[deg] lon[deg] alt[m] vn[m/s] ve[m/s] vd[m/s] ax[m/s2] ay[m/s2] az[m/s2]" << std::endl;

    logfile_adc << "pps[us] rpi_time[s] gps_time[s] uCH0 uCH1 uCH2 uCH3 uCH4 uCH5 uCH6 uCH7 dCH0 dCH1 dCH2 dCH3 dCH4 dCH5 dCH6 dCH7" << std::endl;

    logfile_acts << "gps_time[s] da[] de[] dr[] dt1[] dt2[] dt3[] dt4[] dt5[] dt6[] dt7[] dt8[]" << std::endl;

    logfile_pwm << "gps_time[s] pwm0 pwm1 pwm2 pwm3 pwm4 pwm5 pwm6 pwm7 pwm8 pwm9 pwm10" << std::endl;

    logfile_rc << "gps_time[s] CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8" << std:: endl;

    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler autopilot_handler,vnins_handler,adc_handler,acts_handler,pwm_handler,rc_handler,handlerObject;
    zcm.subscribe("RC_IN",&Handler::read_rc,&rc_handler);
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("PWM_OUT",&Handler::read_pwm,&pwm_handler);
    zcm.subscribe("ACTUATORS",&Handler::read_acts,&acts_handler);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&adc_handler);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&vnins_handler);
    zcm.subscribe("AUTOPILOT",&Handler::read_autopilot,&autopilot_handler);

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
	    //log autopilot data
            logfile_autopilot << autopilot_handler.clear_buffer();

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
    logfile_autopilot.close();

    zcm.stop();

    return 0;
}
