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

    // Conversion constants for the adc inputs (polynomial coefficients from c0*x^0 to cn*x^n)
    float P1_c[2] = { -0.1919 , 4.8261e-6 };   // to dPSI
    float P2_c[2] = { -0.1914 , 4.8269e-6 };   // to dPSI
    float P3_c[2] = { -0.1921 , 4.8141e-6 };   // to dPSI
    float P4_c[2] = { -0.1928 , 4.8152e-6 };   // to dPSI
    float P5_c[2] = { -0.1923 , 4.8028e-6 };   // to dPSI
    float r_ail_c[8] = { -4588.22127869987,  1.24736602820097,  -0.000144283275423259,  9.15548039659827e-09,  -3.43976232741310e-13,  7.65257606251227e-18,  -9.33690805019608e-23, 4.82228861152217e-28 }; // To degrees
    float l_ail_c[5] = { 4.00693268911691,  0.00446296410138788,  -4.07904582198262e-07,  1.37282533801535e-11,  -1.74819546339617e-16 };   // To degrees
    float r_ele_c[4] = { 34.3489086697409, -0.00187596042234432, 6.09685744650086e-08, -1.83603683769599e-12 };                             // To degrees
    float l_ele_c[5] = { -68.4074639202074, 0.0196324817007588, -1.42377037860344e-06, 3.97250998811081e-11, -4.14256459583756e-16 };       // To degrees
    float rud_c[5] = { 1212.71967168363, -0.175564188670768, 9.25919188784856e-06, -2.15842131358955e-10, 1.89881913868532e-15 };           // To degrees

    // Conversion constants for the converted adc data
    float alpha_c[21] = { 0.14923, -13.9738, -0.47878, -0.81364, -0.35622, -0.068444, 0.25439, 0.15764, 0.56846, 0.1074, 0.004642, -0.001176, 0.12197, 0.067263, 0.010847, -0.0021979, -0.013264, 0.01506, 0.001978, -0.0045936, 0.0017014 };                           // to degrees
    float beta_c[21] = { -0.023385, 0.054318, 13.7456, 0.026516, 0.80759, 0.22348, 0.14394, -0.7424, -0.035833, -0.26609, 0.023261, -0.080187, -0.028077, -0.024876, -0.002183, 0.00034037, 0.0094684, -0.017756, 0.012607, 0.016329, -0.0074097 };                     // to degrees
    float cps_c[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 }; // to unitless
    float cpt_c[21] = { -0.21685, 0.02875, 0.011455, -0.11873, -0.046709, -0.11643, 0.0059041, -0.0044851, 0.0085611, 0.0011521, -0.0017282, 0.0058674, 0.0096475, 0.0052426, -0.0020061, -0.00079832, 0.00095876, 0.0007281, 0.0020926, -0.0008775, -0.0001821 };      // to unitless

    //control loop:
    while (!handlerObject.stat.should_exit)
    {
        //publish the status of this module
        zcm.publish("STATUS4",&module_stat);

        // Raw data from adc
        float P1 = (float)adc.data[0];       // uCH0 --> uncalibrated P1 for 5 hole
        float P2 = (float)adc.data[1];       // uCH1 --> uncalibrated P2 for 5 hole
        float P3 = (float)adc.data[2];       // uCH2 --> uncalibrated P3 for 5 hole
        float P4 = (float)adc.data[3];       // uCH3 --> uncalibrated P4 for 5 hole
        float P5 = (float)adc.data[4];       // uCH4 --> uncalibrated P5 for 5 hole
        float r_ail = (float)adc.data[8];    // dCH0 --> uncalibrated right aileron position
        float l_ail = (float)adc.data[9];    // dCH1 --> uncalibrated left aileron position 
        float r_ele = (float)adc.data[10];   // dCH2 --> uncalibrated right elevator position 
        float l_ele = (float)adc.data[11];   // dCH3 --> uncalibrated left elevator position 
        float rud = (float)adc.data[12];     // dCH4 --> uncalibrated rudder position

        // Conversion of raw adc data
        P1 = P1_c[0] + P1_c[1] * P1;   // in dPSI
        P2 = P2_c[0] + P2_c[1] * P2;   // in dPSI
        P3 = P3_c[0] + P3_c[1] * P3;   // in dPSI
        P4 = P4_c[0] + P4_c[1] * P4;   // in dPSI
        P5 = P5_c[0] + P5_c[1] * P5;   // in dPSI
        r_ail = r_ail_c[0] + r_ail_c[1] * r_ail + r_ail_c[2] * pow(r_ail, 2) + r_ail_c[3] * pow(r_ail, 3) + r_ail_c[4] * pow(r_ail, 4) + r_ail_c[5] * pow(r_ail, 5) + r_ail_c[6] * pow(r_ail, 6) + r_ail_c[7] * pow(r_ail, 7); // in degrees
        l_ail = l_ail_c[0] + l_ail_c[1] * l_ail + l_ail_c[2] * pow(l_ail, 2) + l_ail_c[3] * pow(l_ail, 3) + l_ail_c[4] * pow(l_ail, 4);        // in degrees
        r_ele = r_ele_c[0] + r_ele_c[1] * r_ele + r_ele_c[2] * pow(r_ele, 2) + r_ele_c[3] * pow(r_ele, 3);                                     // in degrees
        l_ele = l_ele_c[0] + l_ele_c[1] * l_ele + l_ele_c[2] * pow(l_ele, 2) + l_ele_c[3] * pow(l_ele, 3) + l_ele_c[4] * pow(l_ele, 4);        // in degrees
        rud = rud_c[0] + rud_c[1] * rud + rud_c[2] * pow(rud, 2) + rud_c[3] * pow(rud, 3) + rud_c[4] * pow(rud, 4);                            // in degrees

        // Conversion of converted adc data to state data
        float P_avg = (P2 + P3 + P4 + P5) *0.25;       // in dPSI
        float C_alpha = (P4 - P5) / (P1 - P_avg);      // unitless
        float C_beta = (P3 - P2) / (P1 - P_avg);       // unitless
        float alpha = 0.01745329251 * evaluate_poly(alpha_c, C_alpha, C_beta);   // in rad
        float beta = 0.01745329251 * evaluate_poly(beta_c, C_alpha, C_beta);     // in rad
        float cps = evaluate_poly(cps_c, C_alpha, C_beta);    // unitless
        float cpt = evaluate_poly(cpt_c, C_alpha, C_beta);    // unitless
        float Pt = (P1 - cpt * (P1 - P_avg)) * 6894.76;       // in Pa
        float Ps = (P_avg - cps * (P1 - P_avg)) * 6894.76;    // in Pa
        float rho = 1.1425;                                   // in kg/m^3
        float V = sqrt((2 * (Pt - Ps)) / (rho));              // in m/s

        // INS data
        float yaw = vnins.yaw;        // in degrees
        float pitch = vnins.pitch;    // in degrees
        float roll = vnins.roll;      // in degrees
        float wx = vnins.wx;          // in rad/s (roll rate)
        float wy = vnins.wy;          // in rad/s (pitch rate)
        float wz = vnins.wz;          // in rad/s (yaw rate)

        //assign actuator values
        acts.de = 0;
        acts.da = 0;
        acts.dr = 0;
        acts.dt[0] = 1;
        acts.dt[1] = 0;
        acts.dt[2] = 0;
        acts.dt[3] = 0;
        acts.dt[4] = 0;
        acts.dt[5] = 0;
        acts.dt[6] = 0;
        acts.dt[7] = 0;
        usleep(10000);

        //timestamp the values:
        acts.time_gps = get_gps_time(&handlerObject);

        //publish the actuator values:
        zcm.publish("ACTUATORS", &acts);
    }

    module_stat.module_status = 0;
    zcm.publish("STATUS4",&module_stat);

    std::cout << "autopilot module exiting..." << std::endl;

    zcm.stop();

    return 0;
}
