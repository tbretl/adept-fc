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
//message types:
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"

using std::string;
using namespace std;

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

        void read_vnins(const zcm::ReceiveBuffer* rbuf, const string& chan, const vnins_data_t* msg)
        {
            vnins = *msg;
        }
};

double get_gps_time(Handler* adchandle)
{
    double adc_gps = adchandle->adc.time_gps;
    int64_t adc_time = adchandle->adc.time_rpi;
    int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return  adc_gps + (rpi_time - adc_time)/1000000.0;
}

float evaluate_poly(float coeffs[], float X, float Y) {
    int order = sizeof(coeffs);
    int curr_x = 0;
    int curr_y = 0;
    int x_order = 0;
    int y_order = 0;
    float evaluation = 0.0;

    for (int i; i < order, i++) {
        evaluation += coeffs[i] * pow(X, curr_x) * pow(Y, curr_y);
        if (curr_x == 0) {
            curr_x = x_order + 1;
            x_order = curr_x;
        }
        else {
            curr_x = curr_x - 1;
        }
        if (curr_y == y_order) {
            curr_y = 0;
            y_order = y_order + 1;
        }
        else {
            curr_y = curr_y + 1;
        }
    }

    return evaluation;
}

int main(int argc, char *argv[])
{
    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //initialize message objects
    actuators_t acts;
    memset(&acts, 0, sizeof(acts));

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&handlerObject);
    zcm.subscribe("VNINS_DATA", &Handler::read_vnins, &handlerObject);

    //for publishing stat of this module
    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    //run zcm as a separate thread:
    zcm.start();

    std::cout << "autopilot started" << std::endl;

    //_________________________________START_PASTE_HERE_________________________________//

    // Conversion constants for the adc inputs (polynomial coefficients from c0*x^0 to cn*x^n)
    float P1_c[2] = { -0.19188, 4.8261e-06 }; // To dPSI
    float P2_c[2] = { -0.19142, 4.8269e-06 }; // To dPSI
    float P3_c[2] = { -0.19211, 4.8141e-06 }; // To dPSI
    float P4_c[2] = { -0.19283, 4.8152e-06 }; // To dPSI
    float P5_c[2] = { -0.19231, 4.8028e-06 }; // To dPSI
    float r_ail_c[8] = { -4588.2213, 1.2474, -0.00014428, 9.1555e-09, -3.4398e-13, 7.6526e-18, -9.3369e-23, 4.8223e-28 }; // To degrees
    float l_ail_c[5] = { 4.0069, 0.004463, -4.079e-07, 1.3728e-11, -1.7482e-16 }; // To degrees
    float r_ele_c[4] = { 34.3489, -0.001876, 6.0969e-08, -1.836e-12 }; // To degrees
    float l_ele_c[5] = { -68.4075, 0.019632, -1.4238e-06, 3.9725e-11, -4.1426e-16 }; // To degrees
    float rud_c[5] = { 1212.7197, -0.17556, 9.2592e-06, -2.1584e-10, 1.8988e-15 }; // To degrees

    // Conversion constants for the converted adc data
    float alpha_c[21] = { 0.14923, -13.9738, -0.47878, -0.81364, -0.35622, -0.068444, 0.25439, 0.15764, 0.56846, 0.1074, 0.004642, -0.001176, 0.12197, 0.067263, 0.010847, -0.0021979, -0.013264, 0.01506, 0.001978, -0.0045936, 0.0017014 };
    float beta_c[21] = { -0.023385, 0.054318, 13.7456, 0.026516, 0.80759, 0.22348, 0.14394, -0.7424, -0.035833, -0.26609, 0.023261, -0.080187, -0.028077, -0.024876, -0.002183, 0.00034037, 0.0094684, -0.017756, 0.012607, 0.016329, -0.0074097 };
    float cps_c[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
    float cpt_c[21] = { -0.21685, 0.02875, 0.011455, -0.11873, -0.046709, -0.11643, 0.0059041, -0.0044851, 0.0085611, 0.0011521, -0.0017282, 0.0058674, 0.0096475, 0.0052426, -0.0020061, -0.00079832, 0.00095876, 0.0007281, 0.0020926, -0.0008775, -0.0001821 };

    // Controller constants
    float k_lon[2][4] = { {0.0044319, 0.3739, -0.10855, -0.37623},  {0.002028, -0.037446, -0.03146, -0.050771} };
    float k_lat[2][5] = { {1.1324, 0.16596, -0.075796, 0.57311, 0.90946},  {0.22587, 0.011704, -0.2124, 0.0059284, -0.16631} };

    // Trim conditions
    float V_0 = 30.5755; // m/s
    float alpha_0 = 0.0178; // rad
    float q_0 = 0; // rad/s
    float theta_0 = 0; // rad
    float beta_0 = 0; // rad
    float p_0 = 0; // rad/s
    float r_0 = 0; // rad/s
    float phi_0 = 0; // rad
    float psi_0 = 0; // rad

    // Input trim values
    float de_0 = -0.0832; // rad
    float da_0 = 0; // rad
    float dr_0 = 0; // rad
    float dt_0_0 = 0; //percent throttle [0.0, 1.0]
    float dt_1_0 = 0; //percent throttle [0.0, 1.0]
    float dt_2_0 = 0; //percent throttle [0.0, 1.0]
    float dt_3_0 = 0; //percent throttle [0.0, 1.0]
    float dt_4_0 = 0; //percent throttle [0.0, 1.0]
    float dt_5_0 = 0; //percent throttle [0.0, 1.0]
    float dt_6_0 = 0; //percent throttle [0.0, 1.0]
    float dt_7_0 = 0; //percent throttle [0.0, 1.0]

    // Declare all other values used
    float P1;
    float P2;
    float P3;
    float P4;
    float P5;
    float r_ail;
    float l_ail;
    float r_ele;
    float l_ele;
    float rud;
    float P_avg;
    float C_alpha;
    float C_beta;
    float alpha;
    float beta;
    float cps;
    float cpt;
    float Pt;
    float Ps;
    float rho;
    float V;
    float yaw;
    float pitch;
    float roll;
    float wx;
    float wy;
    float wz;
    float lon_states[4];
    float lat_states[5];
    float u_lon_0;
    float u_lon_1;
    float u_lat_0;
    float u_lat_1;

    //control loop:
    while (!handlerObject.stat.should_exit)
    {
        //publish the status of this module
        zcm.publish("STATUS4", &module_stat);

        // Raw data from adc
        P1 = (float)adc.data[0]; // uCH0 --> uncalibrated P1 for 5 hole probe
        P2 = (float)adc.data[1]; // uCH1 --> uncalibrated P2 for 5 hole probe
        P3 = (float)adc.data[2]; // uCH2 --> uncalibrated P3 for 5 hole probe
        P4 = (float)adc.data[3]; // uCH3 --> uncalibrated P4 for 5 hole probe
        P5 = (float)adc.data[4]; // uCH4 --> uncalibrated P5 for 5 hole probe
        r_ail = (float)adc.data[8]; // dCH0 --> uncalibrated right aileron position
        l_ail = (float)adc.data[9]; // dCH1 --> uncalibrated left aileron position 
        r_ele = (float)adc.data[10]; // dCH2 --> uncalibrated right elevator position
        l_ele = (float)adc.data[11]; // dCH3 --> uncalibrated left elevator position 
        rud = (float)adc.data[12]; // dCH4 --> uncalibrated rudder position

        // Conversion of raw adc data
        P1 = P1_c[0] * pow(P1, 0) + P1_c[1] * pow(P1, 1); // in dPSI
        P2 = P2_c[0] * pow(P2, 0) + P2_c[1] * pow(P2, 1); // in dPSI
        P3 = P3_c[0] * pow(P3, 0) + P3_c[1] * pow(P3, 1); // in dPSI
        P4 = P4_c[0] * pow(P4, 0) + P4_c[1] * pow(P4, 1); // in dPSI
        P5 = P5_c[0] * pow(P5, 0) + P5_c[1] * pow(P5, 1); // in dPSI
        r_ail = r_ail_c[0] * pow(r_ail, 0) + r_ail_c[1] * pow(r_ail, 1) + r_ail_c[2] * pow(r_ail, 2) + r_ail_c[3] * pow(r_ail, 3) + r_ail_c[4] * pow(r_ail, 4) + r_ail_c[5] * pow(r_ail, 5) + r_ail_c[6] * pow(r_ail, 6) + r_ail_c[7] * pow(r_ail, 7); // in degrees
        l_ail = l_ail_c[0] * pow(l_ail, 0) + l_ail_c[1] * pow(l_ail, 1) + l_ail_c[2] * pow(l_ail, 2) + l_ail_c[3] * pow(l_ail, 3) + l_ail_c[4] * pow(l_ail, 4); // in degrees
        r_ele = r_ele_c[0] * pow(r_ele, 0) + r_ele_c[1] * pow(r_ele, 1) + r_ele_c[2] * pow(r_ele, 2) + r_ele_c[3] * pow(r_ele, 3); // in degrees
        l_ele = l_ele_c[0] * pow(l_ele, 0) + l_ele_c[1] * pow(l_ele, 1) + l_ele_c[2] * pow(l_ele, 2) + l_ele_c[3] * pow(l_ele, 3) + l_ele_c[4] * pow(l_ele, 4); // in degrees
        rud = rud_c[0] * pow(rud, 0) + rud_c[1] * pow(rud, 1) + rud_c[2] * pow(rud, 2) + rud_c[3] * pow(rud, 3) + rud_c[4] * pow(rud, 4); // in degrees

        // Conversion of converted adc data to state data
        P_avg = (P2 + P3 + P4 + P5) * 0.25; // in dPSI
        C_alpha = (P4 - P5) / (P1 - P_avg); // unitless
        C_beta = (P3 - P2) / (P1 - P_avg); // unitless
        alpha = 0.01745329251 * evaluate_poly(alpha_c, C_alpha, C_beta); // in rad
        beta = 0.01745329251 * evaluate_poly(beta_c, C_alpha, C_beta); // in rad
        cps = evaluate_poly(cps_c, C_alpha, C_beta); // unitless
        cpt = evaluate_poly(cpt_c, C_alpha, C_beta); // unitless
        Pt = (P1 - cpt * (P1 - P_avg)) * 6894.76; // in Pa
        Ps = (P_avg - cps * (P1 - P_avg)) * 6894.76; // in Pa
        rho = 1.1666; // in kg/m^3
        V = sqrt((2.0 * (Pt - Ps)) / (rho)); // in m/s

        // INS data
        yaw = 0.01745329251 * vnins.yaw; // in rad
        pitch = 0.01745329251 * vnins.pitch; // in rad
        roll = 0.01745329251 * vnins.roll; // in rad
        wx = vnins.wx; // in rad/s (roll rate)
        wy = vnins.wy; // in rad/s (pitch rate)
        wz = vnins.wz; // in rad/s (yaw rate)

        // Calculate states
        lon_states = { V - V_0, alpha - alpha_0, wy - q_0, pitch - theta_0 }; // lon state (x - x0)
        lat_states = { beta - beta_0, wx - p_0, wz - r_0, roll - phi_0, yaw - psi_0 }; // lat state (x - x0)

        // Calculate lon inputs based on lon state (u-u0) = -K_lon * (x - x0)
        u_lon_0 = -1.0 * k_lon[0][0] * lon_states[0] + -1.0 * k_lon[0][1] * lon_states[1] + -1.0 * k_lon[0][2] * lon_states[2] + -1.0 * k_lon[0][3] * lon_states[3]; // u[0] - u[0]_0 for the lon sys
        u_lon_1 = -1.0 * k_lon[1][0] * lon_states[0] + -1.0 * k_lon[1][1] * lon_states[1] + -1.0 * k_lon[1][2] * lon_states[2] + -1.0 * k_lon[1][3] * lon_states[3]; // u[1] - u[1]_0 for the lon sys

        // Calculate lat inputs based on lat state (u-u0) = -K_lat * (x - x0)
        u_lat_0 = -1.0 * k_lat[0][0] * lat_states[0] + -1.0 * k_lat[0][1] * lat_states[1] + -1.0 * k_lat[0][2] * lat_states[2] + -1.0 * k_lat[0][3] * lat_states[3] + -1.0 * k_lat[0][4] * lat_states[4]; // u[0] - u[0]_0 for the lat sys
        u_lat_1 = -1.0 * k_lat[1][0] * lat_states[0] + -1.0 * k_lat[1][1] * lat_states[1] + -1.0 * k_lat[1][2] * lat_states[2] + -1.0 * k_lat[1][3] * lat_states[3] + -1.0 * k_lat[1][4] * lat_states[4]; // u[1] - u[1]_0 for the lat sys

        //assign actuator values
        acts.de = u_lon_0 + de_0;
        acts.da = u_lat_0 + da_0;
        acts.dr = u_lat_1 + dr_0;
        acts.dt[0] = u_lon_1 + dt_0_0;
        acts.dt[1] = u_lon_1 + dt_1_0;
        acts.dt[2] = u_lon_1 + dt_2_0;
        acts.dt[3] = u_lon_1 + dt_3_0;
        acts.dt[4] = u_lon_1 + dt_4_0;
        acts.dt[5] = u_lon_1 + dt_5_0;
        acts.dt[6] = u_lon_1 + dt_6_0;
        acts.dt[7] = u_lon_1 + dt_7_0;

        //timestamp the values:
        acts.time_gps = get_gps_time(&handlerObject);

        //publish the actuator values:
        zcm.publish("ACTUATORS", &acts);

    }

    //_________________________________END_PASTE_HERE_________________________________//

    module_stat.module_status = 0;
    zcm.publish("STATUS4",&module_stat);

    std::cout << "autopilot module exiting..." << std::endl;

    zcm.stop();

    return 0;
}