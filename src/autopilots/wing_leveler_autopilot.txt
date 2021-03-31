//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//
 
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <chrono>
#include <math.h>
#include <assert.h>
#include <iomanip>
#ifdef LOG
#include <fstream>
#endif
 
// Message types
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"
 
using std::string;
 
// Debugging mode parameters
#ifdef DEBUGGING_MODE
bool done_debugging = false;
int debug_point = 0;
std::string description[22] = {"Trim","-Bad","+Bad","Pitch Up","Pitch Down","Roll Right","Roll Left","Yaw Right","Yaw Left","Pitch Rate Up", "Pitch Rate Down", "Roll Rate Right", "Roll Rate Left", "Yaw Rate Right", "Yaw Rate Left", "Positive AoA", "Negative AoA", "Right Slip", "Left Slip", "High Vel", "Low Vel", "Trim"};
double controlled_input[3]  = { 0.0, 1.0, 0.0 };
double controlled_state[22] = { 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
double debug_pit[22] = { 0.00000, -100.0, 100.0, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_rol[22] = { 0.00000, -100.0, 100.0, 0.00000,  0.00000, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_yaw[22] = { 0.00000,  0.000, 0.000, 0.00000,  0.00000, 0.00000,  0.00000, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_wxx[22] = { 0.00000, -100.0, 100.0, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_wyy[22] = { 0.00000, -100.0, 100.0, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_wzz[22] = { 0.00000, -100.0, 100.0, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.50000, -0.50000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000, 0.00000, 0.00000};
double debug_AoA[22] = { 0.01780, -100.0, 100.0, 0.01780,  0.01780, 0.01780,  0.01780, 0.01780,  0.01780, 0.01780,  0.01780, 0.01780,  0.01780, 0.01780,  0.01780, 0.10000, -0.10000, 0.01780,  0.01780, 0.01780, 0.01780, 0.01780};
double debug_bet[22] = { 0.00000, -100.0, 100.0, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.00000,  0.00000, 0.10000, -0.10000, 0.00000, 0.00000, 0.00000};
double debug_vel[22] = { 30.5755, -100.0, 100.0, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 30.5755,  30.5755, 40.0000, 20.0000, 30.5755};
double debug_ele[22] = { 0.0, 0.0, 0.0, 1.0, -1.0,  0.0,  0.0,  0.0,  0.0, 1.0, -1.0,  0.0,  0.0,  0.0,  0.0, -1.0,  1.0,  0.0,  0.0, -1.0, 1.0, 0.0 };
double debug_ail[22] = { 0.0, 0.0, 0.0, 0.0,  0.0, -1.0,  1.0, -1.0,  1.0, 0.0,  0.0, -1.0,  1.0,  1.0, -1.0,  0.0,  0.0, -1.0,  1.0,  0.0, 0.0, 0.0 };
double debug_rud[22] = { 0.0, 0.0, 0.0, 0.0,  0.0, -1.0,  1.0,  1.0, -1.0, 0.0,  0.0, -1.0,  1.0,  1.0, -1.0,  0.0,  0.0, -1.0,  1.0,  0.0, 0.0, 0.0 };
#endif
 
// Class used to handle incoming ZCM messages
class Handler
{
public:
~Handler() = default;
 
adc_data_t adc;
status_t stat;
vnins_data_t vnins;
 
Handler()
{
        memset(&adc, 0, sizeof(adc));
        memset(&stat, 0, sizeof(stat));
        memset(&vnins, 0, sizeof(vnins));
}
 
void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
{
        adc = *msg;
}
 
void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
{
        stat = *msg;
}
 
void read_vnins(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
{
        vnins = *msg;
}
 
};
 
// Used to gather GPS time from adc
double get_gps_time(Handler* adchandle)
{
        double adc_gps = adchandle->adc.time_gps;
        int64_t adc_time = adchandle->adc.time_rpi;
        int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        return adc_gps + (rpi_time - adc_time)/1000000.0;
}
 
// Evaluates an (n,m) degree polynomial over X and Y
double evl_ply(double coeffs[], int size, double X, double Y)
{
        int curr_x = 0;
        int curr_y = 0;
        int x_order = 0;
        int y_order = 0;
        double evaluation = 0.0;
 
        for (int i = 0; i < size; i++)
        {
                evaluation += coeffs[i] * pow(X, curr_x) * pow(Y, curr_y);
                if (curr_x == 0)
                {
                        curr_x = x_order + 1;
                        x_order = curr_x;
                }
                else
                {
                        curr_x = curr_x - 1;
                }
                if (curr_y == y_order)
                {
                        curr_y = 0;
                        y_order = y_order + 1;
                }
                else
                {
                        curr_y = curr_y + 1;
                }
        }
 
        return evaluation;
}
 
// Evaluates an n degree polynomial over X
double evl_exp(double coeffs[], int size, double X)
{
        double evaluation = 0.0;
 
        for (int i = 0; i < size; i++)
        {
                evaluation += coeffs[i] * pow(X, size - i - 1);
        }
 
        return evaluation;
}
 
int main(int argc, char *argv[])
{
        // Conversion constants for the adc inputs (polynomial coefficients from c0*x^0 to cn*x^n)
        double ps1_con[2] = { 4.8261e-06, -0.19188 }; // To dPSI
        double ps2_con[2] = { 4.8269e-06, -0.19142 }; // To dPSI
        double ps3_con[2] = { 4.8141e-06, -0.19211 }; // To dPSI
        double ps4_con[2] = { 4.8152e-06, -0.19283 }; // To dPSI
        double ps5_con[2] = { 4.8028e-06, -0.19231 }; // To dPSI
 
        // Conversion constants for deflection commands to PWM commands
        double ail_PWM_con[2] = { -19.9874, 1499.3311 };
        double ele_PWM_con[3] = { 0.027712, 10.6043, 1517.7811 };
        double rud_PWM_con[2] = { -12.3453, 1550.7985 };
 
        // Conversion constants for the converted adc data
        double AoA_con[15] = { 0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.065129, 0.29331, 0.10864, 0.57212, 0.12463, 0.022992, 0.029209, 0.089836, 0.057237, 0.016901 };
        double bet_con[15] = { -0.045676, 0.090171, 13.8048, 0.002027, 0.94476, 0.29254, 0.12192, -0.66955, -0.020875, -0.33687, 0.023934, -0.10505, -0.019041, -0.054968, -0.018293 };
        double cps_con[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
        double cpt_con[15] = { -0.21483, 0.036139, 0.0037369, -0.12377, -0.034201, -0.11844, 0.0022027, 0.0040131, 0.0047189, 0.0026645, 0.00010707, 0.0023433, 0.0079094, 0.0034925, -0.001166 };
 
        // Controller constants
        double k_lat[2][5] = { {0, 0.14855, 0, 0.4, 0}, {0, 0, 0, 0, 0} }; // Lateral controller gains of form u_lat = -k_lat * x_lat
 
        // Trim conditions
        double vel_trm = 30.5755; // m/s
        double AoA_trm = 0.0178; // rad
        double wyy_trm = 0; // rad/s
        double pit_trm = 0; // rad
        double bet_trm = 0; // rad
        double wxx_trm = 0; // rad/s
        double wzz_trm = 0; // rad/s
        double rol_trm = 0; // rad
        double yaw_trm = 0; // rad
        double rho_trm = 1.1666; // in kg/m^3
 
        // State limits
        double vel_min = 0; // Minimum acceptable value. Any values lower are considered improper readings.
        double vel_max = 60; // Maximum acceptable value. Any values higher are considered improper readings.
        double AoA_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double AoA_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double wyy_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double wyy_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double pit_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
        double pit_max = 1.0472; // Maximum acceptable value. Any values higher are considered improper readings.
        double bet_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double bet_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double wxx_min = -1.0471; // Minimum acceptable value. Any values lower are considered improper readings.
        double wxx_max = 1.0471; // Maximum acceptable value. Any values higher are considered improper readings.
        double wzz_min = -0.5236; // Minimum acceptable value. Any values lower are considered improper readings.
        double wzz_max = 0.5236; // Maximum acceptable value. Any values higher are considered improper readings.
        double rol_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
        double rol_max = 1.0472; // Maximum acceptable value. Any values higher are considered improper readings.
 
        // Previous states
        double vel_pre = 0.0; // m/s
        double AoA_pre = 0.0; // rad
        double wyy_pre = 0.0; // rad/s
        double pit_pre = 0.0; // rad
        double bet_pre = 0.0; // rad
        double wxx_pre = 0.0; // rad/s
        double wzz_pre = 0.0; // rad/s
        double rol_pre = 0.0; // rad
 
        // Input trim values
        double ele_trm = -0.0832; // rad
        double ail_trm = 0; // rad
        double rud_trm = 0; // rad
        double tr0_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr1_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr2_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr3_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr4_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr5_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr6_trm = 0.49188; //percent throttle [0.0, 1.0]
        double tr7_trm = 0.49188; //percent throttle [0.0, 1.0]
 
        // Input PWM limits
        int ele_PWM_min = 1226;
        int ele_PWM_max = 1774;
        int ail_PWM_min = 1186;
        int ail_PWM_max = 1826;
        int rud_PWM_min = 1300;
        int rud_PWM_max = 1740;
        int thr_PWM_min = 1085;
        int thr_PWM_max = 1904;
 
        // Declare all other pressure values used
        double ps1;
        double ps2;
        double ps3;
        double ps4;
        double ps5;
        double pes_avg;
        double pes_tot;
        double pes_stc;
 
        // Declare all other values used in ADC transformation
        double cof_AoA;
        double cof_bet;
        double cof_ptt;
        double cof_pst;
 
        // Declare all other state values used
        double AoA;
        double bet;
        double vel;
        double yaw;
        double pit;
        double rol;
        double wxx;
        double wyy;
        double wzz;
        double lat_sts[5];
 
        // Declare all other input values used
        double lat_in0;
        double lat_in1;
        double ele_ang_cmd;
        double ail_ang_cmd;
        double rud_ang_cmd;
        int ele_PWM_cmd;
        int ail_PWM_cmd;
        int rud_PWM_cmd;
        int tr0_PWM_cmd;
        int tr1_PWM_cmd;
        int tr2_PWM_cmd;
        int tr3_PWM_cmd;
        int tr4_PWM_cmd;
        int tr5_PWM_cmd;
        int tr6_PWM_cmd;
        int tr7_PWM_cmd;
 
        // Calibration parameters
        #ifdef CALIBRATION_AIL
        int ail_PWM_cal = ail_PWM_min;
        int ail_stp = (int)(((double)ail_PWM_max - (double)ail_PWM_min) / 15.0);
        bool done_initial_wait = false;
        bool done = false;
        #elif defined (CALIBRATION_ELE)
        int ele_PWM_cal = ele_PWM_min;
        int ele_stp = (int)(((double)ele_PWM_max - (double)ele_PWM_min) / 15.0);
        bool done_initial_wait = false;
        bool done = false;
        #elif defined (CALIBRATION_RUD)
        int rud_PWM_cal = rud_PWM_min;
        int rud_stp = (int)(((double)rud_PWM_max - (double)rud_PWM_min) / 15.0);
        bool done_initial_wait = false;
        bool done = false;
        #endif
 
        // Calibration check parameters
        #ifdef CALICHECK_AIL
        double cur_ail_ang = -10.0; // in deg
        double ail_stp = 5.0; // in deg
        bool done_initial_wait = false;
        bool done = false;
        #elif defined (CALICHECK_ELE)
        double cur_ele_ang = -10.0; // in deg
        double ele_stp = 5.0; // in deg
        bool done_initial_wait = false;
        bool done = false;
        #elif defined (CALICHECK_RUD)
        double cur_rud_ang = -10.0; // in deg
        double rud_stp = 5.0; // in deg
        bool done_initial_wait = false;
        bool done = false;
        #endif
 
        // Initialize an iterator
        int cur_itr = 1;
 
        // Initialize zcm and message objects
        zcm::ZCM zcm{ "ipc" };
        actuators_t acts;
        memset(&acts, 0, sizeof(acts));
 
        // Subscribe to incoming channels
        Handler handlerObject;
        zcm.subscribe("STATUS", &Handler::read_stat, &handlerObject);
        zcm.subscribe("ADC_DATA", &Handler::read_adc, &handlerObject);
        zcm.subscribe("VNINS_DATA", &Handler::read_vnins, &handlerObject);
 
        // For publishing stat of this module
        status_t module_stat;
        memset(&module_stat, 0, sizeof(module_stat));
        module_stat.module_status = 1;//module running
 
        // Run zcm as a separate thread
        zcm.start();
        #ifdef DEBUGGING_MODE
        std::cout << "WARNING: WING AUTOPILOT STARTED IN DEBUGGING MODE. DO NOT FLY. " << std::endl;
        #elif defined (LOG)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN LOGGING MODE. DO NOT FLY. " << std::endl;
        #elif defined (CALIBRATION_AIL)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN AILERON CALIBRATION MODE. DO NOT FLY. " << std::endl;
        #elif defined (CALIBRATION_ELE)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN ELEVATOR CALIBRATION MODE. DO NOT FLY. " << std::endl;
        #elif defined (CALIBRATION_RUD)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN RUDDER CALIBRATION MODE. DO NOT FLY. " << std::endl;
        #elif defined (CALICHECK_AIL)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN AILERON CALIBRATION CHECK. DO NOT FLY. " << std::endl;
        #elif defined (CALICHECK_ELE)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN ELEVATOR CALIBRATION CHECK. DO NOT FLY. " << std::endl;
        #elif defined (CALICHECK_RUD)
        std::cout << "WARNING: WING AUTOPILOT STARTED IN RUDDER CALIBRATION CHECK. DO NOT FLY. " << std::endl;
        #else
        std::cout << "WING autopilot started" << std::endl;
        #endif
 
        // Initialize data logger
        #ifdef LOG
        const char* file = "AP_Log.txt";
        std::ofstream logfile;
        logfile.open(file,std::ofstream::out | std::ofstream::trunc);
        logfile << "P_1 [dPSI], P_2 [dPSI], P_3 [dPSI], P_4 [dPSI], P_5 [dPSI], cof_AoA [-], cof_bet [-], ";
        logfile << "cof_pt [-], cof_ps [-], AoA [rad], Beta [rad], P_Tot [Pa], P_Stat [Pa], Vel [m/s], Pitch [rad], ";
        logfile << "Roll [rad], Yaw [rad], Pitch_Rate [rad/s], Roll_Rate [rad/s], Yaw_Rate [rad/s], Ele_Cmd [deg], ";
        logfile << "Ail_Cmd [deg], Rud_Cmd [deg], GPS_Time" << std::endl;
        #endif
 
        // Control loop:
        while (!handlerObject.stat.should_exit)
        {
                // Publish the status of this module
                zcm.publish("STATUS4", &module_stat);
 
                // Gather raw data from ADC and convert it to readable data
                ps1 = evl_exp(ps1_con, 2, (double)handlerObject.adc.data[0] ); // uCH0
                ps2 = evl_exp(ps2_con, 2, (double)handlerObject.adc.data[1] ); // uCH1
                ps3 = evl_exp(ps3_con, 2, (double)handlerObject.adc.data[2] ); // uCH2
                ps4 = evl_exp(ps4_con, 2, (double)handlerObject.adc.data[3] ); // uCH3
                ps5 = evl_exp(ps5_con, 2, (double)handlerObject.adc.data[4] ); // uCH4
 
                pes_avg = (ps2 + ps3 + ps4 + ps5)* 0.25; // in dPSI
                cof_AoA = (ps4 - ps5) / (ps1 - pes_avg); // unitless
                cof_bet = (ps3 - ps2) / (ps1 - pes_avg); // unitless
                cof_ptt = evl_ply(cpt_con, 15, cof_AoA, cof_bet); // unitless
                cof_pst = evl_ply(cps_con, 21, cof_AoA, cof_bet); // unitless
 
                // Use pressure coefficient data to calculate AoA, beta, total pressure, static pressure
                AoA = 0.01745 * evl_ply(AoA_con, 15, cof_AoA, cof_bet);    // in rad
                bet = 0.01745 * evl_ply(bet_con, 15, cof_AoA, cof_bet);    // in rad
                pes_tot = (ps1 - cof_ptt * (ps1 - pes_avg)) * 6894.76; // in Pa
                pes_stc = (pes_avg - cof_pst * (ps1 - pes_avg)) * 6894.76; // in Pa
 
                // If the pressure readings indicate imaginary velocity, assing the velocity as the previous good value
                if (pes_tot >= 0.0 || pes_stc >= 0.0 || pes_tot - pes_stc >= 0.0)
                {
                        vel = vel_pre; // in m/s
                }
                else
                {
                        vel = sqrt((2.0 * (abs(pes_tot) - abs(pes_stc))) / (rho_trm)); // in m/s
                }
 
                // INS data
                pit = 0.017453 * handlerObject.vnins.pitch; // in rad
                rol = 0.017453 * handlerObject.vnins.roll;  // in rad
                yaw = 0.017453 * handlerObject.vnins.yaw;   // in rad
                wxx = handlerObject.vnins.wx;    // in rad/s (roll rate)
                wyy = handlerObject.vnins.wy;    // in rad/s (pitch rate)
                wzz = handlerObject.vnins.wz;    // in rad/s (yaw rate)
 
                // In debugging mode, intercept the state inputs
                #ifdef DEBUGGING_MODE
                pit = debug_pit[debug_point];
                rol = debug_rol[debug_point];
                yaw = debug_yaw[debug_point];
                wxx = debug_wxx[debug_point];
                wyy = debug_wyy[debug_point];
                wzz = debug_wzz[debug_point];
                AoA = debug_AoA[debug_point];
                bet = debug_bet[debug_point];
                vel = debug_vel[debug_point];
                #endif
 
                // Bad state rejection (Assign previous good value of state if measured state is out of range)
                vel = (vel < vel_min || vel > vel_max) ? vel_pre : vel;
                AoA = (AoA < AoA_min || AoA > AoA_max) ? AoA_pre : AoA;
                wyy = (wyy < wyy_min || wyy > wyy_max) ? wyy_pre : wyy;
                pit = (pit < pit_min || pit > pit_max) ? pit_pre : pit;
                bet = (bet < bet_min || bet > bet_max) ? bet_pre : bet;
                wxx = (wxx < wxx_min || wxx > wxx_max) ? wxx_pre : wxx;
                wzz = (wzz < wzz_min || wzz > wzz_max) ? wzz_pre : wzz;
                rol = (rol < rol_min || rol > rol_max) ? rol_pre : rol;
 
                // Collect previous state values
                vel_pre = vel; // m/s
                AoA_pre = AoA; // rad
                wyy_pre = wyy; // rad/s
                pit_pre = pit; // rad
                bet_pre = bet; // rad
                wxx_pre = wxx; // rad/s
                wzz_pre = wzz; // rad/s
                rol_pre = rol; // rad
 
                // Calculate -1.0 * (state errors)
                lat_sts[0] = bet_trm - bet;
                lat_sts[1] = wxx_trm - wxx;
                lat_sts[2] = wzz_trm - wzz;
                lat_sts[3] = rol_trm - rol;
                lat_sts[4] = yaw_trm - yaw;
 
                // Calculate input deltas based on state (u-u0) = -K * (x - x0)
                lat_in0 = k_lat[0][0] * lat_sts[0] + k_lat[0][1] * lat_sts[1] + k_lat[0][2] * lat_sts[2] + k_lat[0][3] * lat_sts[3] + k_lat[0][4] * lat_sts[4]; // u[0] - u_0[0] for the lat subsystem
                lat_in1 = k_lat[1][0] * lat_sts[0] + k_lat[1][1] * lat_sts[1] + k_lat[1][2] * lat_sts[2] + k_lat[1][3] * lat_sts[3] + k_lat[1][4] * lat_sts[4]; // u[1] - u_0[1] for the lat subsystem
 
                // Convert angle commands to degrees
                ele_ang_cmd = 57.29578 * ele_trm; // in degrees
                ail_ang_cmd = 57.29578 * (lat_in0 + ail_trm); // in degrees
                rud_ang_cmd = 57.29578 * (lat_in1 + rud_trm); // in degrees
 
                // Send data to log
                #ifdef LOG
                logfile << ps1 << ", " << ps2 << ", " << ps3 << ", " << ps4 << ", " << ps5 << ", " << cof_AoA << ", " << cof_bet << ", " << pes_tot << ", " << cof_pst << ", ";
                logfile << AoA << ", " << bet << ", " << pes_tot << ", " << pes_stc << ", " << vel << ", ";
                logfile << pit << ", " << rol << ", " << yaw << ", " << wyy << ", " << wxx << ", " << wzz << ", ";
                logfile << ele_ang_cmd << ", " << ail_ang_cmd << ", " << rud_ang_cmd << ", " << get_gps_time(&handlerObject) << std::endl;
                #endif
 
                // Convert angular deflection commands to PWM commands
                #ifdef CALICHECK_AIL
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 3, ele_ang_cmd);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 2, cur_ail_ang);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 2, rud_ang_cmd);
                #elif defined (CALICHECK_ELE)
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 3, cur_ele_ang);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 2, ail_ang_cmd);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 2, rud_ang_cmd);
                #elif defined (CALICHECK_RUD)
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 3, ele_ang_cmd);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 2, ail_ang_cmd);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 2, cur_rud_ang);
                #else
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 3, ele_ang_cmd);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 2, ail_ang_cmd);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 2, rud_ang_cmd);
                #endif
 
                // Convert throttle commands to PWM commands
                tr0_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr0_trm;
                tr1_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr1_trm;
                tr2_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr2_trm;
                tr3_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr3_trm;
                tr4_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr4_trm;
                tr5_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr5_trm;
                tr6_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr6_trm;
                tr7_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * tr7_trm;
 
                // Ensure PWM commands are within safe limits
                ele_PWM_cmd = ele_PWM_cmd > ele_PWM_max ? ele_PWM_max : ele_PWM_cmd;
                ele_PWM_cmd = ele_PWM_cmd < ele_PWM_min ? ele_PWM_min : ele_PWM_cmd;
                ail_PWM_cmd = ail_PWM_cmd > ail_PWM_max ? ail_PWM_max : ail_PWM_cmd;
                ail_PWM_cmd = ail_PWM_cmd < ail_PWM_min ? ail_PWM_min : ail_PWM_cmd;
                rud_PWM_cmd = rud_PWM_cmd > rud_PWM_max ? rud_PWM_max : rud_PWM_cmd;
                rud_PWM_cmd = rud_PWM_cmd < rud_PWM_min ? rud_PWM_min : rud_PWM_cmd;
                tr0_PWM_cmd = tr0_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr0_PWM_cmd;
                tr0_PWM_cmd = tr0_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr0_PWM_cmd;
                tr1_PWM_cmd = tr1_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr1_PWM_cmd;
                tr1_PWM_cmd = tr1_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr1_PWM_cmd;
                tr2_PWM_cmd = tr2_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr2_PWM_cmd;
                tr2_PWM_cmd = tr2_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr2_PWM_cmd;
                tr3_PWM_cmd = tr3_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr3_PWM_cmd;
                tr3_PWM_cmd = tr3_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr3_PWM_cmd;
                tr4_PWM_cmd = tr4_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr4_PWM_cmd;
                tr4_PWM_cmd = tr4_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr4_PWM_cmd;
                tr5_PWM_cmd = tr5_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr5_PWM_cmd;
                tr5_PWM_cmd = tr5_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr5_PWM_cmd;
                tr6_PWM_cmd = tr6_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr6_PWM_cmd;
                tr6_PWM_cmd = tr6_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr6_PWM_cmd;
                tr7_PWM_cmd = tr7_PWM_cmd > thr_PWM_max ? thr_PWM_max : tr7_PWM_cmd;
                tr7_PWM_cmd = tr7_PWM_cmd < thr_PWM_min ? thr_PWM_min : tr7_PWM_cmd;
 
                // Assign actuator values by giving PWM commands
                #ifdef DEBUGGING_MODE
                // Check state inputs
                if (debug_point == 1 || debug_point == 2)
                {
                        assert (pit == 0.00000);
                        assert (rol == 0.00000);
                        assert (yaw == 0.00000);
                        assert (wxx == 0.00000);
                        assert (wyy == 0.00000);
                        assert (wzz == 0.00000);
                        assert (AoA == 0.01780);
                        assert (bet == 0.00000);
                        assert (vel == 30.5755);
                }
 
                // Check state range
                assert (pit <= pit_max);
                assert (pit >= pit_min);
                assert (rol <= rol_max);
                assert (rol >= rol_min);
                assert (wxx <= wxx_max);
                assert (wxx >= wxx_min);
                assert (wyy <= wyy_max);
                assert (wyy >= wyy_min);
                assert (wzz <= wzz_max);
                assert (wzz >= wzz_min);
                assert (AoA <= AoA_max);
                assert (AoA >= AoA_min);
                assert (bet <= bet_max);
                assert (bet >= bet_min);
                assert (vel <= vel_max);
                assert (vel >= vel_min);
 
                // Check command range
                assert (ele_PWM_cmd <= ele_PWM_max);
                assert (ele_PWM_cmd >= ele_PWM_min);
                assert (ail_PWM_cmd <= ail_PWM_max);
                assert (ail_PWM_cmd <= ail_PWM_max);
                assert (rud_PWM_cmd >= rud_PWM_min);
                assert (rud_PWM_cmd >= rud_PWM_min);
                assert (tr0_PWM_cmd <= thr_PWM_max);
                assert (tr0_PWM_cmd >= thr_PWM_min);
                assert (tr1_PWM_cmd <= thr_PWM_max);
                assert (tr1_PWM_cmd >= thr_PWM_min);
                assert (tr2_PWM_cmd <= thr_PWM_max);
                assert (tr2_PWM_cmd >= thr_PWM_min);
                assert (tr3_PWM_cmd <= thr_PWM_max);
                assert (tr3_PWM_cmd >= thr_PWM_min);
                assert (tr4_PWM_cmd <= thr_PWM_max);
                assert (tr4_PWM_cmd >= thr_PWM_min);
                assert (tr5_PWM_cmd <= thr_PWM_max);
                assert (tr5_PWM_cmd >= thr_PWM_min);
                assert (tr6_PWM_cmd <= thr_PWM_max);
                assert (tr6_PWM_cmd >= thr_PWM_min);
                assert (tr7_PWM_cmd <= thr_PWM_max);
                assert (tr7_PWM_cmd >= thr_PWM_min);
 
                // Check control surface commands
                if (!done_debugging)
                {
                    if (controlled_state[debug_point] == 0.0 || controlled_input[0] == 0.0 || debug_ele[debug_point] == 0.0)
                    {
                            assert (abs(ele_ang_cmd - 57.29578 * ele_trm) <= 0.01);
                    }
                    else
                    {
                            assert (signbit(ele_ang_cmd - 57.29578 * ele_trm) == signbit(debug_ele[debug_point]));
                    }
                    if (controlled_state[debug_point] == 0.0 || controlled_input[1] == 0.0 || debug_ail[debug_point] == 0.0)
                    {
                            assert (abs(ail_ang_cmd - 57.29578 * ail_trm) <= 0.01);
                    }
                    else
                    {
                            assert (signbit(ail_ang_cmd - 57.29578 * ail_trm) == signbit(debug_ail[debug_point]));
                    }
                    if (controlled_state[debug_point] == 0.0 || controlled_input[2] == 0.0 || debug_rud[debug_point] == 0.0)
                    {
                            assert (abs(rud_ang_cmd - 57.29578 * rud_trm) <= 0.01);
                    }
                    else
                    {
                            assert (signbit(rud_ang_cmd - 57.29578 * rud_trm) == signbit(debug_rud[debug_point]));
                    }
                }
 
                // Set the actuator values
                acts.de = ele_PWM_cmd;
                acts.da = ail_PWM_cmd;
                acts.dr = rud_PWM_cmd;
                acts.dt[0] = tr0_PWM_cmd;
                acts.dt[1] = tr1_PWM_cmd;
                acts.dt[2] = tr2_PWM_cmd;
                acts.dt[3] = tr3_PWM_cmd;
                acts.dt[4] = tr4_PWM_cmd;
                acts.dt[5] = tr5_PWM_cmd;
                acts.dt[6] = tr6_PWM_cmd;
                acts.dt[7] = tr7_PWM_cmd;
 
                #elif defined (CALIBRATION_AIL)
                acts.de = ele_PWM_cmd;
                acts.da = ail_PWM_cal;
                acts.dr = rud_PWM_cmd;
                acts.dt[0] = tr0_PWM_cmd;
                acts.dt[1] = tr1_PWM_cmd;
                acts.dt[2] = tr2_PWM_cmd;
                acts.dt[3] = tr3_PWM_cmd;
                acts.dt[4] = tr4_PWM_cmd;
                acts.dt[5] = tr5_PWM_cmd;
                acts.dt[6] = tr6_PWM_cmd;
                acts.dt[7] = tr7_PWM_cmd;
 
                #elif defined (CALIBRATION_ELE)
                acts.de = ele_PWM_cal;
                acts.da = ail_PWM_cmd;
                acts.dr = rud_PWM_cmd;
                acts.dt[0] = tr0_PWM_cmd;
                acts.dt[1] = tr1_PWM_cmd;
                acts.dt[2] = tr2_PWM_cmd;
                acts.dt[3] = tr3_PWM_cmd;
                acts.dt[4] = tr4_PWM_cmd;
                acts.dt[5] = tr5_PWM_cmd;
                acts.dt[6] = tr6_PWM_cmd;
                acts.dt[7] = tr7_PWM_cmd;
 
                #elif defined (CALIBRATION_RUD)
                acts.de = ele_PWM_cmd;
                acts.da = ail_PWM_cmd;
                acts.dr = rud_PWM_cal;
                acts.dt[0] = tr0_PWM_cmd;
                acts.dt[1] = tr1_PWM_cmd;
                acts.dt[2] = tr2_PWM_cmd;
                acts.dt[3] = tr3_PWM_cmd;
                acts.dt[4] = tr4_PWM_cmd;
                acts.dt[5] = tr5_PWM_cmd;
                acts.dt[6] = tr6_PWM_cmd;
                acts.dt[7] = tr7_PWM_cmd;
 
                #else
                acts.de = ele_PWM_cmd;
                acts.da = ail_PWM_cmd;
                acts.dr = rud_PWM_cmd;
                acts.dt[0] = tr0_PWM_cmd;
                acts.dt[1] = tr1_PWM_cmd;
                acts.dt[2] = tr2_PWM_cmd;
                acts.dt[3] = tr3_PWM_cmd;
                acts.dt[4] = tr4_PWM_cmd;
                acts.dt[5] = tr5_PWM_cmd;
                acts.dt[6] = tr6_PWM_cmd;
                acts.dt[7] = tr7_PWM_cmd;
                #endif
 
                // Sleep for 10 ms to remove CPU stress
                usleep(10000);
 
                // Debugging stuff
                #ifdef DEBUGGING_MODE
                if (cur_itr == 1)
                {
                        if (!done_debugging)
                        {
                                if (debug_point < 21 && debug_point != 0)
                                {
                                        std::cout << std::fixed;
                                        std::cout << std::setprecision(2);
                                        if (debug_point <= 9)
                                        {
                                                std::cout<< "DEBUG POINT " << debug_point<< "   |  ";
                                        }
                                        else
                                        {
                                                std::cout<< "DEBUG POINT " << debug_point<< "  |  ";
                                        }
                                        std::cout<< std::setw(15) << description[debug_point] << "  |  " << std::flush;
                                        if (ele_ang_cmd >= 0.0 && ele_ang_cmd < 10.0)
                                        {
                                                std::cout<< "Ele CMD =   " << ele_ang_cmd << " deg  |  ";
                                        }
                                        else if (ele_ang_cmd >= 10.0)
                                        {
                                                std::cout<< "Ele CMD =  " << ele_ang_cmd << " deg  |  ";
                                        }
                                        else if (ele_ang_cmd < 0.0 && ele_ang_cmd > -10.0)
                                        {
                                                std::cout<< "Ele CMD =  " << ele_ang_cmd << " deg  |  ";
                                        }
                                        else
                                        {
                                                std::cout<< "Ele CMD = " << ele_ang_cmd << " deg  |  ";
                                        }
                                        if (ail_ang_cmd >= 0.0 && ail_ang_cmd < 10.0)
                                        {
                                                std::cout<< "Ail CMD =   " << ail_ang_cmd << " deg  |  ";
                                        }
                                        else if (ail_ang_cmd >= 10.0)
                                        {
                                                std::cout<< "Ail CMD =  " << ail_ang_cmd << " deg  |  ";
                                        }
                                        else if (ail_ang_cmd < 0.0 && ail_ang_cmd > -10.0)
                                        {
                                                std::cout<< "Ail CMD =  " << ail_ang_cmd << " deg  |  ";
                                        }
                                        else
                                        {
                                                std::cout<< "Ail CMD = " << ail_ang_cmd << " deg  |  ";
                                        }
                                        if (rud_ang_cmd >= 0.0 && rud_ang_cmd < 10.0)
                                        {
                                                std::cout<< "Rud CMD =   " << rud_ang_cmd << " deg";
                                        }
                                        else if (rud_ang_cmd >= 10.0)
                                        {
                                                std::cout<< "Rud CMD =  " << rud_ang_cmd << " deg";
                                        }
                                        else if (rud_ang_cmd < 0.0 && rud_ang_cmd > -10.0)
                                        {
                                                std::cout<< "Rud CMD =  " << rud_ang_cmd << " deg";
                                        }
                                        else
                                        {
                                                std::cout<< "Rud CMD = " << rud_ang_cmd << " deg";
                                        }
                                        std::cout<< std::endl;
                                }
                                if (debug_point >= 21)
                                {
                                        std::cout<<"ALL TESTS PASSED!"<<std::endl;
                                        done_debugging = true;
                                }
                        }
                }
                if (cur_itr == 700)
                {
                    cur_itr = 0;
                    debug_point++;
                }
                #elif defined(CALIBRATION_AIL)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Aileron PWM Command: " << ail_PWM_cal << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (ail_PWM_cal + ail_stp <= ail_PWM_max)
                        {
                                ail_PWM_cal += ail_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #elif defined(CALIBRATION_ELE)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Elevator PWM Command: " << ele_PWM_cal << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (ele_PWM_cal + ele_stp <= ele_PWM_max)
                        {
                                ele_PWM_cal += ele_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #elif defined(CALIBRATION_RUD)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Rudder PWM Command: " << rud_PWM_cal << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (rud_PWM_cal + rud_stp <= rud_PWM_max)
                        {
                                rud_PWM_cal += rud_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #elif defined(CALICHECK_AIL)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Aileron Angular Command: " << cur_ail_ang << " deg" << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (cur_ail_ang + ail_stp <= 10.0)
                        {
                                cur_ail_ang += ail_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #elif defined(CALICHECK_ELE)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Elevator Angular Command: " << cur_ele_ang << " deg" << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (cur_ele_ang + ele_stp <= 10.0)
                        {
                                cur_ele_ang += ele_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #elif defined(CALICHECK_RUD)
                if (!done_initial_wait && cur_itr == 600)
                {
                        done_initial_wait = true;
                        cur_itr = 0;
                }
                if (done_initial_wait && cur_itr == 1 && !done)
                {
                        std::cout<<"Rudder Angular Command: " << cur_rud_ang << " deg" << std::endl;
                }
                if (done_initial_wait && cur_itr == 1500)
                {
                        if (cur_rud_ang + rud_stp <= 10.0)
                        {
                                cur_rud_ang += rud_stp;
                        }
                        else
                        {
                                done = true;
                        }
                        cur_itr = 0;
                }
                #else
                cur_itr--;
                #endif
 
                // Iterator iterator
                cur_itr++;
 
                // Timestamp the values:
                acts.time_gps = get_gps_time(&handlerObject);
 
                // Publish the actuator values:
                zcm.publish("ACTUATORS", &acts); 
 
        }
 
        // Close the log
        #ifdef LOG
        logfile.close();
        #endif
 
        // Publish terminating status
        module_stat.module_status = 0;
        zcm.publish("STATUS4",&module_stat);
        std::cout << "autopilot module exiting..." << std::endl;
 
        // Stop ZCM
        zcm.stop();
 
        return 0;
}