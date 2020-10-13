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
 
// Message types
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"
 
using std::string;
 
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
        double ele_PWM_con[3] = { 0.041213, 11.2938, 1529.0023 };
        double rud_PWM_con[2] = { -12.3453, 1550.7985 };
 
        // Conversion constants for the converted adc data
        double AoA_con[15] = { 0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.065129, 0.29331, 0.10864, 0.57212, 0.12463, 0.022992, 0.029209, 0.089836, 0.057237, 0.016901 };
        double bet_con[15] = { -0.045676, 0.090171, 13.8048, 0.002027, 0.94476, 0.29254, 0.12192, -0.66955, -0.020875, -0.33687, 0.023934, -0.10505, -0.019041, -0.054968, -0.018293 };
        double cps_con[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
        double cpt_con[15] = { -0.21483, 0.036139, 0.0037369, -0.12377, -0.034201, -0.11844, 0.0022027, 0.0040131, 0.0047189, 0.0026645, 0.00010707, 0.0023433, 0.0079094, 0.0034925, -0.001166 };
 
        // Controller constants
        double k[11][9] = { {0.004639, 0.378, -0.11071, -0.38004, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0.81164, 0.15878, -0.1148, 0.4868, 0.67803}, {0, 0, 0, 0, 0.27624, 0.012545, -0.14409, 0.028823, -0.061911}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, -0.037781, 0.0073251, 0.073957, 0.030925, 0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, -0.037781, 0.0073251, 0.073957, 0.030925, 0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, -0.037781, 0.0073251, 0.073957, 0.030925, 0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, -0.037781, 0.0073251, 0.073957, 0.030925, 0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, 0.037781, -0.0073251, -0.073957, -0.030925, -0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, 0.037781, -0.0073251, -0.073957, -0.030925, -0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, 0.037781, -0.0073251, -0.073957, -0.030925, -0.12702}, {0.00025922, -0.0046156, -0.0040125, -0.0064646, 0.037781, -0.0073251, -0.073957, -0.030925, -0.12702} }; // Controller gains of form u = -k * x
 
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
        double sts[9];
 
        // Declare all other input values used
        double in0;
        double in1;
        double in2;
        double in3;
        double in4;
        double in5;
        double in6;
        double in7;
        double in8;
        double in9;
        double in10;
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
        std::cout << "autopilot started" << std::endl;
 
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
                sts[0] = vel_trm - vel;
                sts[1] = AoA_trm - AoA;
                sts[2] = wyy_trm - wyy;
                sts[3] = pit_trm - pit;
                sts[4] = bet_trm - bet;
                sts[5] = wxx_trm - wxx;
                sts[6] = wzz_trm - wzz;
                sts[7] = rol_trm - rol;
                sts[8] = yaw_trm - yaw;
 
                // Calculate input deltas based on state (u-u0) = -K * (x - x0)
                in0 = k[0][0] * sts[0] + k[0][1] * sts[1] + k[0][2] * sts[2] + k[0][3] * sts[3] + k[0][4] * sts[4] + k[0][5] * sts[5] + k[0][6] * sts[6] + k[0][7] * sts[7] + k[0][8] * sts[8]; // u[0] - u_0[0] for the system
                in1 = k[1][0] * sts[0] + k[1][1] * sts[1] + k[1][2] * sts[2] + k[1][3] * sts[3] + k[1][4] * sts[4] + k[1][5] * sts[5] + k[1][6] * sts[6] + k[1][7] * sts[7] + k[1][8] * sts[8]; // u[1] - u_0[1] for the system
                in2 = k[2][0] * sts[0] + k[2][1] * sts[1] + k[2][2] * sts[2] + k[2][3] * sts[3] + k[2][4] * sts[4] + k[2][5] * sts[5] + k[2][6] * sts[6] + k[2][7] * sts[7] + k[2][8] * sts[8]; // u[2] - u_0[2] for the system
                in3 = k[3][0] * sts[0] + k[3][1] * sts[1] + k[3][2] * sts[2] + k[3][3] * sts[3] + k[3][4] * sts[4] + k[3][5] * sts[5] + k[3][6] * sts[6] + k[3][7] * sts[7] + k[3][8] * sts[8]; // u[3] - u_0[3] for the system
                in4 = k[4][0] * sts[0] + k[4][1] * sts[1] + k[4][2] * sts[2] + k[4][3] * sts[3] + k[4][4] * sts[4] + k[4][5] * sts[5] + k[4][6] * sts[6] + k[4][7] * sts[7] + k[4][8] * sts[8]; // u[4] - u_0[4] for the system
                in5 = k[5][0] * sts[0] + k[5][1] * sts[1] + k[5][2] * sts[2] + k[5][3] * sts[3] + k[5][4] * sts[4] + k[5][5] * sts[5] + k[5][6] * sts[6] + k[5][7] * sts[7] + k[5][8] * sts[8]; // u[5] - u_0[5] for the system
                in6 = k[6][0] * sts[0] + k[6][1] * sts[1] + k[6][2] * sts[2] + k[6][3] * sts[3] + k[6][4] * sts[4] + k[6][5] * sts[5] + k[6][6] * sts[6] + k[6][7] * sts[7] + k[6][8] * sts[8]; // u[6] - u_0[6] for the system
                in7 = k[7][0] * sts[0] + k[7][1] * sts[1] + k[7][2] * sts[2] + k[7][3] * sts[3] + k[7][4] * sts[4] + k[7][5] * sts[5] + k[7][6] * sts[6] + k[7][7] * sts[7] + k[7][8] * sts[8]; // u[7] - u_0[7] for the system
                in8 = k[8][0] * sts[0] + k[8][1] * sts[1] + k[8][2] * sts[2] + k[8][3] * sts[3] + k[8][4] * sts[4] + k[8][5] * sts[5] + k[8][6] * sts[6] + k[8][7] * sts[7] + k[8][8] * sts[8]; // u[8] - u_0[8] for the system
                in9 = k[9][0] * sts[0] + k[9][1] * sts[1] + k[9][2] * sts[2] + k[9][3] * sts[3] + k[9][4] * sts[4] + k[9][5] * sts[5] + k[9][6] * sts[6] + k[9][7] * sts[7] + k[9][8] * sts[8]; // u[9] - u_0[9] for the system
                in10 = k[10][0] * sts[0] + k[10][1] * sts[1] + k[10][2] * sts[2] + k[10][3] * sts[3] + k[10][4] * sts[4] + k[10][5] * sts[5] + k[10][6] * sts[6] + k[10][7] * sts[7] + k[10][8] * sts[8]; // u[10] - u_0[10] for the system
 
                // Convert angle commands to degrees
                ele_ang_cmd = 57.29578 * (in0 + ele_trm); // in degrees
                ail_ang_cmd = 57.29578 * (in1 + ail_trm); // in degrees
                rud_ang_cmd = 57.29578 * (in2 + rud_trm); // in degrees
 
                // Convert angular deflection commands to PWM commands
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 3, ele_ang_cmd);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 2, ail_ang_cmd);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 2, rud_ang_cmd);
 
                // Convert throttle commands to PWM commands
                tr0_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in3 + tr0_trm);
                tr1_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in4 + tr1_trm);
                tr2_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in5 + tr2_trm);
                tr3_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in6 + tr3_trm);
                tr4_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in7 + tr4_trm);
                tr5_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in8 + tr5_trm);
                tr6_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in9 + tr6_trm);
                tr7_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (in10 + tr7_trm);
 
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
 
                // Sleep for 10 ms to remove CPU stress
                usleep(10000);
 
                // Debugging stuff
                if (cur_itr % 100 == 0)
                {
                        /*
                        std::cout.precision(1);
                        std::cout<< "Ele CMD = " << ele_ang_cmd << " deg   |   ";
                        std::cout<< "Ele PWM = " << ele_PWM_cmd << "   |   ";
                        std::cout<< "Ail CMD = " << ail_ang_cmd << " deg   |   ";
                        std::cout<< "Ail PWM = " << ail_PWM_cmd << "   |   ";
                        std::cout<< "Rud CMD = " << rud_ang_cmd << " deg   |   ";
                        std::cout<< "Rud PWM = " << rud_PWM_cmd << "   |   ";
                        std::cout<< "Thr CMD = " << (lon_in1 + tr0_trm)*100.0 << " %   |   ";
                        std::cout<< "Thr PWM = " << tr0_PWM_cmd;
                        std::cout<< "\t\r" << std::flush;
                        */
                        cur_itr = 1;
                }
 
                // Iterator iterator
                cur_itr++;
 
                // Timestamp the values:
                acts.time_gps = get_gps_time(&handlerObject);
 
                // Publish the actuator values:
                zcm.publish("ACTUATORS", &acts); 
 
        }
 
        // Publish terminating status
        module_stat.module_status = 0;
        zcm.publish("STATUS4",&module_stat);
        std::cout << "autopilot module exiting..." << std::endl;
 
        // Stop ZCM
        zcm.stop();
 
        return 0;
}
