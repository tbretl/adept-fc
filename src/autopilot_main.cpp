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

        /*
        // Conversion constants for ADC to control surface deflections
        double rel_con[4] = { -1.8360e-12,  6.0969e-08, -0.00190000,  34.3486000 };
        double lel_con[5] = { -4.1426e-16,  3.9725e-11, -1.4238e-06,  0.01960000, 68.407500 };
        double ral_con[8] = {  4.8223e-28, -9.3369e-23,  7.6526e-18,  3.4398e-13, 9.1555e-9, -1.4428e-4, 1.2474000, 4588.2000 };
        double lal_con[5] = { -1.7482e-16,  1.3728e-11, -4.0790e-07,  0.00450000, 4.0069000 };
        double rud_con[5] = {  1.8988e-15, -2.1584e-10,  9.2592e-06, -0.17560000, 1212.7000 };
         */

        // Conversion constants for deflection commands to PWM commands
        double ail_PWM_con[6] = { -2.5025e-5,  7.2442e-4, 0.0042000, -0.11670, -19.7993,  1498.60 };
        double ele_PWM_con[6] = { -1.0598e-6, -5.2453e-5, 7.6461e-5,  0.06800,  11.4908,  1528.60 };
        double rud_PWM_con[7] = {  9.3548e-8, -6.1896e-6, 2.2718e-5,  0.00270,  0.00990, -12.6829, 1547.5 };

        // Conversion constants for the converted adc data
        double AoA_con[15] = {  0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.06513,  0.29331,  0.10864,  0.57212,  0.12463,  0.022992,  0.02921,  0.08984,  0.057237,  0.016901 };
        double bet_con[15] = { -0.04568,  0.09017,  13.8048,  0.00203,  0.94476,  0.29254,  0.12192, -0.66955, -0.02088, -0.33687,  0.023934, -0.10505, -0.01904, -0.054968, -0.018293 };
        double cps_con[21] = {  0.59235, -0.00321, -0.00458, -0.09805,  0.01015, -0.09770, -0.01585, -0.00249, -0.02244, -0.00376, -0.004233, -0.00394,  0.01926, -0.001497, -0.0014977, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
        double cpt_con[15] = { -0.21483,  0.03614,  0.00374, -0.12377, -0.03420, -0.11844,  0.00220,  0.00401,  0.00472,  0.00266,  0.000107,  0.00234,  0.00791,  0.003493, -0.001166 };

        // Wing Leveling Controller constants
        double k_lon[2][4] = { {0.00000, 0.00000, 0.00000, 0.00000},  {0.00000, 0.00000, 0.00000, 0.00000} };// Longitudinal controller gains of form u_lon = -k_lon * x_lon
        double k_lat[2][5] = { {0.00000, 0.14855, 0.00000, 0.40000, 0.00000},  {0.00000, 0.00000, 0.00000, 0.00000, 0.00000} };// Lateral controller gains of form u_lat = -k_lat * x_lat

        // Trim conditions
        double vel_trm = 30.576; // m/s
        double AoA_trm = 0.0178; // rad
        double wyy_trm = 0.0000; // rad/s
        double pit_trm = 0.0000; // rad
        double bet_trm = 0.0000; // rad
        double wxx_trm = 0.0000; // rad/s
        double wzz_trm = 0.0000; // rad/s
        double rol_trm = 0.0000; // rad
        double yaw_trm = 0.0000; // rad
        double rho_trm = 1.1666; // in kg/m^3

        // State limits
        double vel_min =  0.0000; // Minimum acceptable value. Any values lower are considered improper readings.
        double vel_max =  60.000; // Maximum acceptable value. Any values higher are considered improper readings.
        double AoA_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double AoA_max =  0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double wyy_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double wyy_max =  0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double pit_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
        double pit_max =  1.0472; // Maximum acceptable value. Any values higher are considered improper readings.
        double bet_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
        double bet_max =  0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
        double wxx_min = -1.0471; // Minimum acceptable value. Any values lower are considered improper readings.
        double wxx_max =  1.0471; // Maximum acceptable value. Any values higher are considered improper readings.
        double wzz_min = -0.5236; // Minimum acceptable value. Any values lower are considered improper readings.
        double wzz_max =  0.5236; // Maximum acceptable value. Any values higher are considered improper readings.
        double rol_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
        double rol_max =  1.0472; // Maximum acceptable value. Any values higher are considered improper readings.

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
        double ail_trm =  0.0000; // rad
        double rud_trm =  0.0000; // rad
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
        double lon_sts[4];
        double lat_sts[5];

        /*
        // Declare all other control surface values used
        double ele_lft;
        double ele_rgt;
        double ail_lft;
        double ail_rgt;
        double rud;
        */

        // Declare all other input values used
        double lon_in0;
        double lon_in1;
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
                /*
                ail_rgt = evl_exp(ral_con, 4, (double)handlerObject.adc.data[8] ); // dCH0
                ail_lft = evl_exp(lal_con, 5, (double)handlerObject.adc.data[9] ); // dCH1
                ele_rgt = evl_exp(rel_con, 8, (double)handlerObject.adc.data[10]); // dCH2
                ele_lft = evl_exp(lel_con, 5, (double)handlerObject.adc.data[11]); // dCH3
                rud     = evl_exp(rud_con, 5, (double)handlerObject.adc.data[12]); // dCH4
                */

                // Calculate pressure coefficient data
                pes_avg = (ps2 + ps3 + ps4 + ps5)* 0.25;          // in dPSI
                cof_AoA = (ps4 - ps5) / (ps1 - pes_avg);          // unitless
                cof_bet = (ps3 - ps2) / (ps1 - pes_avg);          // unitless
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
                lon_sts[0] = vel_trm - vel;
                lon_sts[1] = AoA_trm - AoA;
                lon_sts[2] = wyy_trm - wyy;
                lon_sts[3] = pit_trm - pit;
                lat_sts[0] = bet_trm - bet;
                lat_sts[1] = wxx_trm - wxx;
                lat_sts[2] = wzz_trm - wzz;
                lat_sts[3] = rol_trm - rol;
                lat_sts[4] = yaw_trm - yaw;

                // Calculate lon inputs based on lon state (u-u0) = -K_lon * (x - x0)
                lon_in0 = k_lon[0][0] * lon_sts[0] + k_lon[0][1] * lon_sts[1] + k_lon[0][2] * lon_sts[2] + k_lon[0][3] * lon_sts[3]; // u[0] - u[0]_0 for the lon sys
                lon_in1 = k_lon[1][0] * lon_sts[0] + k_lon[1][1] * lon_sts[1] + k_lon[1][2] * lon_sts[2] + k_lon[1][3] * lon_sts[3]; // u[1] - u[1]_0 for the lon sys

                // Calculate lat inputs based on lat state (u-u0) = -K_lat * (x - x0)
                lat_in0 = k_lat[0][0] * lat_sts[0] + k_lat[0][1] * lat_sts[1] + k_lat[0][2] * lat_sts[2] + k_lat[0][3] * lat_sts[3] + k_lat[0][4] * lat_sts[4]; // u[0] - u[0]_0 for the lat sys
                lat_in1 = k_lat[1][0] * lat_sts[0] + k_lat[1][1] * lat_sts[1] + k_lat[1][2] * lat_sts[2] + k_lat[1][3] * lat_sts[3] + k_lat[1][4] * lat_sts[4]; // u[1] - u[1]_0 for the lat sys

                // Convert angle commands to degrees
                ele_ang_cmd = 57.29578*(lon_in0 + ele_trm); // in degrees
                ail_ang_cmd = 57.29578*(lat_in0 + ail_trm); // in degrees
                rud_ang_cmd = 57.29578*(lat_in1 + rud_trm); // in degrees

                // Convert angular degree commands to PWM commands
                ele_PWM_cmd = (int) evl_exp(ele_PWM_con, 6, ele_ang_cmd);
                ail_PWM_cmd = (int) evl_exp(ail_PWM_con, 6, ail_ang_cmd);
                rud_PWM_cmd = (int) evl_exp(rud_PWM_con, 7, rud_ang_cmd);
                tr0_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr0_trm);
                tr1_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr1_trm);
                tr2_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr2_trm);
                tr3_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr3_trm);
                tr4_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr4_trm);
                tr5_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr5_trm);
                tr6_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr6_trm);
                tr7_PWM_cmd = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (lon_in1 + tr7_trm);

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
