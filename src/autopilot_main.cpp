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

double get_gps_time(Handler* adchandle)
{
    double adc_gps = adchandle->adc.time_gps;
    int64_t adc_time = adchandle->adc.time_rpi;
    int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return  adc_gps + (rpi_time - adc_time)/1000000.0;
}

double evaluate_poly(double coeffs[], int size, double X, double Y) {
    int curr_x = 0;
    int curr_y = 0;
    int x_order = 0;
    int y_order = 0;
    double evaluation = 0.0;

    for (int i = 0; i < size; i++) {
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
    
    //_________________________________START_PASTE_HERE_________________________________//

    // Conversion constants for the adc inputs (polynomial coefficients from c0*x^0 to cn*x^n)
    double P1_c[2] = { -0.19188, 4.8261e-06 }; // To dPSI
    double P2_c[2] = { -0.19142, 4.8269e-06 }; // To dPSI
    double P3_c[2] = { -0.19211, 4.8141e-06 }; // To dPSI
    double P4_c[2] = { -0.19283, 4.8152e-06 }; // To dPSI
    double P5_c[2] = { -0.19231, 4.8028e-06 }; // To dPSI

    // Conversion constants for the converted adc data
    double alpha_c[15] = { 0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.065129, 0.29331, 0.10864, 0.57212, 0.12463, 0.022992, 0.029209, 0.089836, 0.057237, 0.016901 };
    double beta_c[15] = { -0.045676, 0.090171, 13.8048, 0.002027, 0.94476, 0.29254, 0.12192, -0.66955, -0.020875, -0.33687, 0.023934, -0.10505, -0.019041, -0.054968, -0.018293 };
    double cps_c[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
    double cpt_c[15] = { -0.21483, 0.036139, 0.0037369, -0.12377, -0.034201, -0.11844, 0.0022027, 0.0040131, 0.0047189, 0.0026645, 0.00010707, 0.0023433, 0.0079094, 0.0034925, -0.001166 };

    // Wing Leveling Controller constants
    double k_lon[2][4] = { {0, 0, 0, 0},  {0, 0, 0, 0} }; // Longitudinal controller gains of form u_lon = -k_lon * x_lon
    double k_lat[2][5] = { {0, 0.14855, 0, 0.4, 0},  {0, 0, 0, 0, 0} }; // Lateral controller gains of form u_lat = -k_lat * x_lat

    // Trim conditions
    double V_0 = 30.5755; // m/s
    double alpha_0 = 0.0178; // rad
    double q_0 = 0; // rad/s
    double theta_0 = 0; // rad
    double beta_0 = 0; // rad
    double p_0 = 0; // rad/s
    double r_0 = 0; // rad/s
    double phi_0 = 0; // rad
    double psi_0 = 0; // rad
    double rho = 1.1666; // in kg/m^3

    // State limits
    double V_min = 0; // Minimum acceptable value. Any values lower are considered improper readings.
    double V_max = 60; // Maximum acceptable value. Any values higher are considered improper readings.
    double alpha_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
    double alpha_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
    double q_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
    double q_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
    double theta_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
    double theta_max = 1.0472; // Maximum acceptable value. Any values higher are considered improper readings.
    double beta_min = -0.6108; // Minimum acceptable value. Any values lower are considered improper readings.
    double beta_max = 0.6108; // Maximum acceptable value. Any values higher are considered improper readings.
    double p_min = -1.0471; // Minimum acceptable value. Any values lower are considered improper readings.
    double p_max = 1.0471; // Maximum acceptable value. Any values higher are considered improper readings.
    double r_min = -0.5236; // Minimum acceptable value. Any values lower are considered improper readings.
    double r_max = 0.5236; // Maximum acceptable value. Any values higher are considered improper readings.
    double phi_min = -1.0472; // Minimum acceptable value. Any values lower are considered improper readings.
    double phi_max = 1.0472; // Maximum acceptable value. Any values higher are considered improper readings.

    // Previous states
    double V_prev = 0.0; // m/s
    double alpha_prev = 0.0; // rad
    double q_prev = 0.0; // rad/s
    double theta_prev = 0.0; // rad
    double beta_prev = 0.0; // rad
    double p_prev = 0.0; // rad/s
    double r_prev = 0.0; // rad/s
    double phi_prev = 0.0; // rad

    // Input trim values
    double de_0 = -0.0832; // rad
    double da_0 = 0; // rad
    double dr_0 = 0; // rad
    double dt_0_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_1_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_2_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_3_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_4_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_5_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_6_0 = 0.49188; //percent throttle [0.0, 1.0]
    double dt_7_0 = 0.49188; //percent throttle [0.0, 1.0]

    // Declare all other values used
    double P1;
    double P2;
    double P3;
    double P4;
    double P5;
    double P_avg;
    double C_alpha;
    double C_beta;
    double alpha;
    double beta;
    double cps;
    double cpt;
    double Pt;
    double Ps;
    double V;
    double yaw;
    double pitch;
    double roll;
    double wx;
    double wy;
    double wz;
    double lon_states[4];
    double lat_states[5];
    double u_lon_0;
    double u_lon_1;
    double u_lat_0;
    double u_lat_1;
    double r_ail;
    double l_ail;
    double r_ele;
    double l_ele;
    double rud;
    double elevator_angle_command;
    double aileron_angle_command;
    double rudder_angle_command;
    int curr_iteration = 1;

    //initialize zcm
    zcm::ZCM zcm{ "ipc" };

    //initialize message objects
    actuators_t acts;
    memset(&acts, 0, sizeof(acts));

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS", &Handler::read_stat, &handlerObject);
    zcm.subscribe("ADC_DATA", &Handler::read_adc, &handlerObject);
    zcm.subscribe("VNINS_DATA", &Handler::read_vnins, &handlerObject);

    //for publishing stat of this module
    status_t module_stat;
    memset(&module_stat, 0, sizeof(module_stat));
    module_stat.module_status = 1;//module running

    //run zcm as a separate thread:
    zcm.start();

    std::cout << "autopilot started" << std::endl;

    //control loop:
    while (!handlerObject.stat.should_exit)
    {
        //publish the status of this module
        zcm.publish("STATUS4", &module_stat);

        // Raw data from adc
        P1 = P1_c[0] + P1_c[1] * (double)handlerObject.adc.data[0]; // uCH0 --> P1 for 5 hole probe
        P2 = P2_c[0] + P2_c[1] * (double)handlerObject.adc.data[1]; // uCH1 --> P2 for 5 hole probe
        P3 = P3_c[0] + P3_c[1] * (double)handlerObject.adc.data[2]; // uCH2 --> P3 for 5 hole probe
        P4 = P4_c[0] + P4_c[1] * (double)handlerObject.adc.data[3]; // uCH3 --> P4 for 5 hole probe
        P5 = P5_c[0] + P5_c[1] * (double)handlerObject.adc.data[4]; // uCH4 --> P5 for 5 hole probe
        r_ail = (double)handlerObject.adc.data[8];  // dCH0 -> r_ail
	l_ail = (double)handlerObject.adc.data[9];  // dCH1 -> l_ail
	r_ele = (double)handlerObject.adc.data[10]; // dCH2 -> r_ele 
	l_ele = (double)handlerObject.adc.data[11]; // dCH2 -> l_ele
	rud =   (double)handlerObject.adc.data[12]; // dCH3 -> rud

	// Convert control surface data to radian values
	r_ail = pow(r_ail,7)*4.8223e-28 + pow(r_ail,6)*-9.3369e-23 + pow(r_ail,5)*7.6526e-18 + pow(r_ail,4)*-3.4398e-13 + pow(r_ail,3)*9.1555e-9 + pow(r_ail,2)*-1.4428e-4 + r_ail*1.2474 - 4.5882e3; // in deg
	l_ail = pow(l_ail,4)*-1.7482e-16 + pow(l_ail,3)*1.3728e-11 + pow(l_ail,2)*-4.0790e-7 + l_ail*0.0045 + 4.0069; // in deg
	r_ele = pow(r_ele,3)*-1.8360e-12 + pow(r_ele,2)*6.0969e-8 + r_ele*-0.0019 + 34.3486; // in deg
	l_ele = pow(l_ele,4)*-4.1426e-16 + pow(l_ele,3)*3.9725e-11 + pow(l_ele,2)*-1.4238e-6 + l_ele*0.0196 - 68.4075; // in deg
	rud = pow(rud,4)*1.8988e-15 + pow(rud,3)*-2.1584e-10 + pow(rud,2)*9.2592e-6 + rud*-0.1756 + 1.2127e3; // in deg

        // Conversion of converted adc data to state data
        P_avg = (P2 + P3 + P4 + P5) * 0.25; // in dPSI
        C_alpha = (P4 - P5) / (P1 - P_avg); // unitless
        C_beta = (P3 - P2) / (P1 - P_avg); // unitless
        alpha = 0.01745329251 * evaluate_poly(alpha_c, 15, C_alpha, C_beta); // in rad
        beta = 0.01745329251 * evaluate_poly(beta_c, 15, C_alpha, C_beta); // in rad
        cps = evaluate_poly(cps_c, 21, C_alpha, C_beta); // unitless
        cpt = evaluate_poly(cpt_c, 15, C_alpha, C_beta); // unitless
        Pt = (P1 - cpt * (P1 - P_avg)) * 6894.76; // in Pa
        Ps = (P_avg - cps * (P1 - P_avg)) * 6894.76; // in Pa

        // If the pressure readings indicate imaginary velocity, assing the velocity as the previous good value
        if (Pt >= 0.0 || Ps >= 0.0 || Pt - Ps >= 0.0) {
            //std::cout << "Pressure readings rejected" << std::endl;
            V = V_prev;
        }
        else {
            V = sqrt((2.0 * (abs(Pt) - abs(Ps))) / (rho)); // in m/s
        }

        // INS data
        yaw = 0.01745329251 * handlerObject.vnins.yaw; // in rad
        pitch = 0.01745329251 * handlerObject.vnins.pitch; // in rad
        roll = 0.01745329251 * handlerObject.vnins.roll; // in rad
        wx = handlerObject.vnins.wx; // in rad/s (roll rate)
        wy = handlerObject.vnins.wy; // in rad/s (pitch rate)
        wz = handlerObject.vnins.wz; // in rad/s (yaw rate)

        // Bad state rejection
        V = (V < V_min || V > V_max) ? V_prev : V;
        alpha = (alpha < alpha_min || alpha > alpha_max) ? alpha_prev : alpha;
        wy = (wy < q_min || wy > q_max) ? q_prev : wy;
        pitch = (pitch < theta_min || pitch > theta_max) ? theta_prev : pitch;
        beta = (beta < beta_min || beta > beta_max) ? beta_prev : beta;
        wx = (wx < p_min || wx > p_max) ? p_prev : wx;
        wz = (wz < r_min || wz > r_max) ? r_prev : wz;
        roll = (roll < phi_min || roll > phi_max) ? phi_prev : roll;

        // Calculate states
        lon_states[0] = V - V_0;
        lon_states[1] = alpha - alpha_0;
        lon_states[2] = wy - q_0;
        lon_states[3] = pitch - theta_0;
        lat_states[0] = beta - beta_0;
        lat_states[1] = wx - p_0;
        lat_states[2] = wz - r_0;
        lat_states[3] = roll - phi_0;
        lat_states[4] = yaw - psi_0;

        // Calculate lon inputs based on lon state (u-u0) = -K_lon * (x - x0)
        u_lon_0 = -1.0 * k_lon[0][0] * lon_states[0] + -1.0 * k_lon[0][1] * lon_states[1] + -1.0 * k_lon[0][2] * lon_states[2] + -1.0 * k_lon[0][3] * lon_states[3]; // u[0] - u[0]_0 for the lon sys
        u_lon_1 = -1.0 * k_lon[1][0] * lon_states[0] + -1.0 * k_lon[1][1] * lon_states[1] + -1.0 * k_lon[1][2] * lon_states[2] + -1.0 * k_lon[1][3] * lon_states[3]; // u[1] - u[1]_0 for the lon sys

        // Calculate lat inputs based on lat state (u-u0) = -K_lat * (x - x0)
        u_lat_0 = -1.0 * k_lat[0][0] * lat_states[0] + -1.0 * k_lat[0][1] * lat_states[1] + -1.0 * k_lat[0][2] * lat_states[2] + -1.0 * k_lat[0][3] * lat_states[3] + -1.0 * k_lat[0][4] * lat_states[4]; // u[0] - u[0]_0 for the lat sys
        u_lat_1 = -1.0 * k_lat[1][0] * lat_states[0] + -1.0 * k_lat[1][1] * lat_states[1] + -1.0 * k_lat[1][2] * lat_states[2] + -1.0 * k_lat[1][3] * lat_states[3] + -1.0 * k_lat[1][4] * lat_states[4]; // u[1] - u[1]_0 for the lat sys

        // Collect previous state values
        V_prev = V; // m/s
        alpha_prev = alpha; // rad
        q_prev = wy; // rad/s
        theta_prev = pitch; // rad
        beta_prev = beta; // rad
        p_prev = wx; // rad/s
        r_prev = wz; // rad/s
        phi_prev = roll; // rad

	// Convert angle commands to PWM commands
	elevator_angle_command = 57.29578*(u_lon_0 + de_0); // in degrees
	elevator_angle_command = pow(elevator_angle_command,4)*-5.1386e-6 + pow(elevator_angle_command,3)*-1.9281e-4 + pow(elevator_angle_command,2)*0.0783 + elevator_angle_command*10.4587 + 1.5023e3; // in PWM
	elevator_angle_command = elevator_angle_command > 1774.0 ? 1774.0 : elevator_angle_command;
	elevator_angle_command = elevator_angle_command < 1226.0 ? 1226.0 : elevator_angle_command;
	aileron_angle_command = 57.29578*(u_lat_0 + da_0); // in degrees
	aileron_angle_command = pow(aileron_angle_command,4)*8.6261e-4 + pow(aileron_angle_command,3)*-0.0068 + pow(aileron_angle_command,2)*-0.0638 + aileron_angle_command*-17.4873 + 1.4620e3; // in PWM
	aileron_angle_command = aileron_angle_command > 1826.0 ? 1826.0 : aileron_angle_command;
	aileron_angle_command = aileron_angle_command < 1186.0 ? 1186.0 : aileron_angle_command;	
	rudder_angle_command = 57.29578*(u_lat_1 + dr_0) - 5.0; // in degrees
	rudder_angle_command = rudder_angle_command*-11.7637 + 1.4615e3; // in PWM
	rudder_angle_command = rudder_angle_command > 1740.0 ? 1740.0 : rudder_angle_command;
	rudder_angle_command = rudder_angle_command < 1085.0 ? 1085.0 : rudder_angle_command;


        //assign actuator values
        acts.de = (int)elevator_angle_command;
        acts.da = (int)aileron_angle_command; 
        acts.dr = (int)rudder_angle_command;
        acts.dt[0] = u_lon_1 + dt_0_0;
        acts.dt[1] = u_lon_1 + dt_1_0;
        acts.dt[2] = u_lon_1 + dt_2_0;
        acts.dt[3] = u_lon_1 + dt_3_0;
        acts.dt[4] = u_lon_1 + dt_4_0;
        acts.dt[5] = u_lon_1 + dt_5_0;
        acts.dt[6] = u_lon_1 + dt_6_0;
        acts.dt[7] = u_lon_1 + dt_7_0;
        usleep(10000);
	
	if (curr_iteration % 500 == 0){
		std::cout<<std::endl<<std::endl;
		
		std::cout<< "Elevator command = " << (u_lon_0 + de_0)*57.325 << " deg" << std::endl;
		std::cout<< "l_ele = " << l_ele << " deg" << std::endl;
		std::cout<< "ele PWM = " << (int)elevator_angle_command << std::endl << std::endl;
		
		std::cout<< "Aileron command = " << (u_lat_0 + da_0)*57.325 << " deg" << std::endl;
		std::cout<< "l_ail = " << l_ail << " deg" << std::endl;
		std::cout<< "ail PWM = " << (int)aileron_angle_command << std::endl << std::endl;
		
		std::cout<< "Rudder command = " << (u_lat_1 + dr_0)*57.325 << " deg" << std::endl;
		std::cout<< "rud = " << rud << " deg" << std::endl;
		std::cout<< "rud PWM = " << (int)rudder_angle_command << std::endl << std::endl;
	}

	curr_iteration++;
        
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
