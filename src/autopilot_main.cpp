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
#include <iomanip>
#include <fstream>
#include <vector>
#ifdef TEST
	#include <stdlib.h>
	#include <time.h>
#endif

// Message types
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"
#include "rc_t.hpp"

using std::string;
using namespace std;

// Class used to handle incoming ZCM messages
class Handler
{
	public:
		~Handler() = default;

		adc_data_t adc;
		status_t stat;
		vnins_data_t vnins;
		rc_t rc_in;

		Handler()
		{
			memset(&adc, 0, sizeof(adc));
			memset(&stat, 0, sizeof(stat));
			memset(&vnins, 0, sizeof(vnins));
			memset(&rc_in, 0 ,sizeof(rc_in));
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

		void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
		{
			rc_in = *msg;
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
double eval_2D_poly(double coeffs[], int size, double X, double Y)
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
double eval_1D_poly(double coeffs[], int size, double X)
{
	double evaluation = 0.0;

	for (int i = 0; i < size; i++)
	{
		evaluation += coeffs[i] * pow(X, size - i - 1);
	}

	return evaluation;
}

// Converts RC in command to PWM command
int output_scaling(const int& in_val, const double& s_min, const double& s_max, const int& in_min, const int& in_max)
{
	return (s_min + (s_max-s_min)/(in_max-in_min)*(in_val-in_min));
}

#ifdef TEST
	// FE steps states based on dynamics, inputs, and current states
	void step_states(vector<vector<double>> A, vector<vector<double>> B, vector<double>* states, vector<double> inputs, double delta_t)
	{
		double Axi = 0.0;
		double Bui = 0.0;
		vector<double> new_states = vector<double>(9,0.0);
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 9; j++)
			{
				Axi+=A[i][j] * states->at(j);
			}
			for(int j = 0; j < 11; j++)
			{
				Bui+=B[i][j] * inputs[j];
			}
			new_states[i] = states->at(i) + delta_t * (Axi + Bui);
			Axi = 0.0;
			Bui = 0.0;
		}
		for (int i = 0; i < 9; i++)
		{
			states->at(i) = new_states[i];
		}

	}
	
	// Gets a random double between -1.0 and 1.0
	double get_rand()
	{
		return 2.0 * ((double) rand() / (double) RAND_MAX) - 1.0;
	}
#endif

int main(int argc, char *argv[])
{
	// **************************************************** CALIBRATION DATA **************************************************** //
	// Conversion constants for the adc inputs
	double ps1_consts[2] = { 4.8261e-06, -0.19188 }; // From ADC RAW to dPSI
	double ps2_consts[2] = { 4.8269e-06, -0.19142 }; // From ADC RAW to dPSI
	double ps3_consts[2] = { 4.8141e-06, -0.19211 }; // From ADC RAW to dPSI
	double ps4_consts[2] = { 4.8152e-06, -0.19283 }; // From ADC RAW to dPSI
	double ps5_consts[2] = { 4.8028e-06, -0.19231 }; // From ADC RAW to dPSI

	// Conversion constants for deflection commands to PWM commands
	double ail_PWM_consts[2] = { -19.9874, 1499.3311 };           // From deflection in degrees to PWM CMD
	double ele_PWM_consts[3] = { 0.027712, 10.6043, 1517.7811 };  // From deflection in degrees to PWM CMD
	double rud_PWM_consts[2] = { -12.3453, 1550.7985 };           // From deflection in degrees to PWM CMD

	// Conversion constants for the converted adc data
	double AoA_consts[15] = { 0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.065129, 0.29331, 0.10864, 0.57212, 0.12463, 0.022992, 0.029209, 0.089836, 0.057237, 0.016901 };
	double bet_consts[15] = { -0.045676, 0.090171, 13.8048, 0.002027, 0.94476, 0.29254, 0.12192, -0.66955, -0.020875, -0.33687, 0.023934, -0.10505, -0.019041, -0.054968, -0.018293 };
	double coeff_ps_consts[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
	double coeff_pt_consts[15] = { -0.21483, 0.036139, 0.0037369, -0.12377, -0.034201, -0.11844, 0.0022027, 0.0040131, 0.0047189, 0.0026645, 0.00010707, 0.0023433, 0.0079094, 0.0034925, -0.001166 };

	// **************************************************** AUTOPILOT CONFIG DATA **************************************************** //
	// Atmospheric conditions
	double rho; // kg/m^3

	// Input PWM limits
	int ele_PWM_min;  // PWM
	int ele_PWM_max;  // PWM
	int ail_PWM_min;  // PWM
	int ail_PWM_max;  // PWM
	int rud_PWM_min;  // PWM
	int rud_PWM_max;  // PWM
	int thr_PWM_min;  // PWM
	int thr_PWM_max;  // PWM

	// State trim conditions
	double vel_trim; // m/s
	double AoA_trim; // rad
	double wyy_trim; // rad/s
	double pit_trim; // rad
	double bet_trim; // rad
	double wxx_trim; // rad/s
	double wzz_trim; // rad/s
	double rol_trim; // rad
	double yaw_trim; // rad

	// State limits. Any higher or lower values are considered faulty readings.
	double vel_min; // m/s
	double vel_max; // m/s
	double AoA_lim; // rad
	double wyy_lim; // rad/s
	double pit_lim; // rad
	double bet_lim; // rad
	double wxx_lim; // rad/s
	double wzz_lim; // rad/s
	double rol_lim; // rad

	// Config loading variables
	string ap_type;
	string dump;
	std::ifstream ap_config_stream;

	// Load config data
	ap_config_stream.open("/home/pi/adept-fc/config_files/autopilot_main.config");
	ap_config_stream >> dump >> ap_type;
	ap_config_stream >> dump >> rho;
	ap_config_stream >> dump >> ele_PWM_min;
	ap_config_stream >> dump >> ele_PWM_max;
	ap_config_stream >> dump >> ail_PWM_min;
	ap_config_stream >> dump >> ail_PWM_max;
	ap_config_stream >> dump >> rud_PWM_min;
	ap_config_stream >> dump >> rud_PWM_max;
	ap_config_stream >> dump >> thr_PWM_min;
	ap_config_stream >> dump >> thr_PWM_max;
	ap_config_stream >> dump >> vel_trim;
	ap_config_stream >> dump >> AoA_trim;
	ap_config_stream >> dump >> wyy_trim;
	ap_config_stream >> dump >> pit_trim;
	ap_config_stream >> dump >> bet_trim;
	ap_config_stream >> dump >> wxx_trim;
	ap_config_stream >> dump >> wzz_trim;
	ap_config_stream >> dump >> rol_trim;
	ap_config_stream >> dump >> yaw_trim;
	ap_config_stream >> dump >> vel_min;
	ap_config_stream >> dump >> vel_max;
	ap_config_stream >> dump >> AoA_lim;
	ap_config_stream >> dump >> wyy_lim;
	ap_config_stream >> dump >> pit_lim;
	ap_config_stream >> dump >> bet_lim;
	ap_config_stream >> dump >> wxx_lim;
	ap_config_stream >> dump >> wzz_lim;
	ap_config_stream >> dump >> rol_lim;
	ap_config_stream.close();

	// **************************************************** GAIN DATA **************************************************** //
	// Controller constants
	double k[11][9];

	// Loading variables
	std::ifstream gain_stream;

	// Load config data
	string ap_gains_load_path = "/home/pi/adept-fc/config_files/autopilot_gains/" + ap_type + ".dat";
	std::cout << "AP Gains @ " << ap_gains_load_path << std::endl;
	gain_stream.open(ap_gains_load_path);
	if(!gain_stream.good())
	{
		std::cout << "WARNING: COULD NOT FIND AP GAINS." << std::endl;
		return 1;
	}
	for (int i = 0; i < 11; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			gain_stream >> k[i][j];
		}
	}
	gain_stream.close();

	#ifdef TEST
		// **************************************************** AUTOPILOT TEST DATA **************************************************** //
		// Absolute state noise magnitude
		double state_noise[9] = { 9.00e-1,  8.73e-3,  5.24e-4,  5.24e-4,  8.73e-3,  5.24e-4,  3.49e-3,  5.24e-4,  3.49e-3 }; 
								//vel m/s,  AoA rad,  wyy 1/s,  pit rad,  bet rad,  wxx 1/s,  wzz 1/s,  rol rad,  yaw rad
		double true_absolute_states[9] = { vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim }; 
		vector<double> true_state_errors = vector<double>(9, 0.0);
		srand (time(NULL));
		
		// Sequencing file numbers
		std::ifstream seqFile ("config_files/sequence.dat", std::ifstream::in);
		int fileNum;
		seqFile >> fileNum;
		seqFile.close();
		fileNum++;

		// File name
		char file_ap_test[25];
		sprintf(file_ap_test,"FlightLog_%i_ap_test.dat",fileNum);

		// Create log for artificial state data
		std::ofstream logfile_ap_test;
		logfile_ap_test.open(file_ap_test, std::ofstream::out | std::ofstream::trunc | std::ofstream::binary);
		logfile_ap_test << "gps_time[s] test raw_vel[m/s] vel[m/s] true_vel[m/s] raw_AoA[rad] AoA[rad] true_AoA[rad] wyy[rad/s] true_wyy[rad] pit[rad] true_pit[rad] raw_bet[rad] bet[rad] true_bet[rad] wxx[rad/s] true_wxx[rad/s] wzz[rad/s] true_wzz[rad/s] rol[rad] true_rol[rad] yaw[rad] true_yaw[rad]" << std::endl;

		// Generate test parameters
		int num_tests=16;
		int curr_test_number = 0;
		bool initial_state_set = false;
		double initial_states[16][9]{
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 			//Trim conditions
			{vel_trim, AoA_trim, wyy_trim, pit_trim+0.75*pit_lim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim},	//Nose up
			{vel_trim, AoA_trim, wyy_trim, pit_trim-0.75*pit_lim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim},	//Nose down	
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim+0.75*rol_lim, yaw_trim},	//Roll right
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim-0.75*rol_lim, yaw_trim},	//Roll left
			{vel_trim, AoA_trim+0.75*AoA_lim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 	//AoA up
			{vel_trim, AoA_trim-0.75*AoA_lim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 	//AoA down
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim+0.75*bet_lim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 	//Beta right
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim-0.75*bet_lim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 	//Beta left
			{0.8333*vel_max, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 			//Velocity up
			{1.5000*vel_min, AoA_trim, wyy_trim, pit_trim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim}, 			//Velocity down
			{vel_trim, AoA_trim+0.75*AoA_lim, wyy_trim, pit_trim+0.75*pit_lim, bet_trim, wxx_trim, wzz_trim, rol_trim, yaw_trim},	//Longitudinal perturbation
			{vel_trim, AoA_trim, wyy_trim, pit_trim, bet_trim+0.75*bet_lim, wxx_trim, wzz_trim, rol_trim+0.75*rol_lim, yaw_trim},	//Lateral perturbation
			{vel_trim, AoA_trim+0.75*AoA_lim, wyy_trim, pit_trim+0.75*pit_lim, bet_trim+0.75*bet_lim, wxx_trim, wzz_trim, rol_trim+0.75*rol_lim, yaw_trim},		//Wind state perturbation
			{0.8333*vel_max, AoA_trim+0.75*AoA_lim, wyy_trim, pit_trim+0.75*pit_lim, bet_trim+0.75*bet_lim, wxx_trim, wzz_trim, rol_trim+0.75*rol_lim, yaw_trim},	//Full state + perturbation
			{1.5000*vel_min, AoA_trim-0.75*AoA_lim, wyy_trim, pit_trim-0.75*pit_lim, bet_trim-0.75*bet_lim, wxx_trim, wzz_trim, rol_trim-0.75*rol_lim, yaw_trim}	//Full state - perturbation
		};

		// **************************************************** SYSTEM DYNAMICS **************************************************** //
		std::ifstream dynamics_stream;
		vector<vector<double>> A = vector<vector<double>>(9, vector<double>(9, 0.0));
		vector<vector<double>> B = vector<vector<double>>(9, vector<double>(11, 0.0));

		// Load dynamics matrices
		dynamics_stream.open("/home/pi/adept-fc/config_files/matrices.dat");
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 9; j++)
			{
				dynamics_stream >> A[i][j];
			}
		}
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 11; j++)
			{
				dynamics_stream >> B[i][j];
			}
		}
		dynamics_stream.close();
	#endif

	// **************************************************** INPUT TRIM DATA **************************************************** //
	// Declare AP RC channels and cutoffs
	std::ifstream rc_config_stream;
	int rc_dump;
	int servo_min;
	int servo_max;
	int rc_min;
	int rc_max;
	int ap_engage_chan;
	int ap_engage_cutoff;

	// Load rc configuration variables (Ignore the unnecessary ones)
	rc_config_stream.open("/home/pi/adept-fc/config_files/pwm_out.config");
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> servo_min;
	rc_config_stream >> dump >> servo_max;
	rc_config_stream >> dump >> rc_min;
	rc_config_stream >> dump >> rc_max;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> ap_engage_chan;
	rc_config_stream >> dump >> ap_engage_cutoff;
	rc_config_stream.close();

	// Input trim values
	bool trim_values_set = false;
	double ele_trim = 0.0; // rad
	double ail_trim = 0.0; // rad
	double rud_trim = 0.0; // rad
	double thr_trim = 0.0; // percent throttle
	
	// Unfiltered RC command variables
	double unfiltered_RC_ele_cmd;  // rad
	double unfiltered_RC_ail_cmd;  // rad
	double unfiltered_RC_rud_cmd;  // rad
	double unfiltered_RC_thr_cmd;  // percent throttle
	
	// Time constant variables
	double delta_t = 0.01;    // seconds
	double time_const = 0.1;  // seconds
	double alpha = 1.0 - exp(-delta_t / time_const);
	

	// **************************************************** OTHER VARIABLES **************************************************** //
	// Previous states
	double vel_prev = 0.0; // m/s
	double AoA_prev = 0.0; // rad
	double wyy_prev = 0.0; // rad/s
	double pit_prev = 0.0; // rad
	double bet_prev = 0.0; // rad
	double wxx_prev = 0.0; // rad/s
	double wzz_prev = 0.0; // rad/s
	double rol_prev = 0.0; // rad

	// Pressure values used in ADC to state conversion
	double ps1;
	double ps2;
	double ps3;
	double ps4;
	double ps5;
	double p_bar;
	double total_pressure;
	double static_pressure;

	// Declare all other calibration coefficients used in ADC to state conversion
	double coeff_AoA;
	double coeff_bet;
	double coeff_pt;
	double coeff_ps;

	// Declare all other state values used
	double unfiltered_AoA;
	double AoA = AoA_trim;
	double unfiltered_bet;
	double bet = bet_trim;
	double unfiltered_vel;
	double vel = vel_trim;
	double yaw;
	double pit;
	double rol;
	double wxx;
	double wyy;
	double wzz;
	vector<double> states = vector<double>(9, 0.0);

	// Declare all other input values used
	vector<double> inputs = vector<double>(11, 0.0);
	double ele_ang_cmd;
	double ail_ang_cmd;
	double rud_ang_cmd;
	int ele_PWM_cmd;
	int ail_PWM_cmd;
	int rud_PWM_cmd;
	int thr_PWM_cmd[8];

	// Initialize zcm and message objects
	zcm::ZCM zcm{ "ipc" };
	actuators_t acts;
	memset(&acts, 0, sizeof(acts));

	// Subscribe to incoming channels
	Handler handlerObject;
	zcm.subscribe("STATUS", &Handler::read_stat, &handlerObject);
	zcm.subscribe("ADC_DATA", &Handler::read_adc, &handlerObject);
	zcm.subscribe("VNINS_DATA", &Handler::read_vnins, &handlerObject);
	zcm.subscribe("RC_IN", &Handler::read_rc, &handlerObject);

	// For publishing stat of this module
	status_t module_stat;
	memset(&module_stat, 0, sizeof(module_stat));
	module_stat.module_status = 1;

	// Run zcm as a separate thread
	zcm.start();
	#ifdef TEST
	std::cout << "WARNING autopilot started in test mode. DO NOT FLY." << std::endl;
	#else
	std::cout << "autopilot started" << std::endl;
	#endif

	// Control loop:
	while (!handlerObject.stat.should_exit)
	{
		// Publish the status of this module
		zcm.publish("STATUS4", &module_stat);

		// Every loop update the trim values
		if(!trim_values_set)
		{
			// Gather PWM trim commands from RC
			double ail_trim_PWM = (double) output_scaling(handlerObject.rc_in.rc_chan[0],servo_min, servo_max, rc_min, rc_max);
			double ele_trim_PWM = (double) output_scaling(handlerObject.rc_in.rc_chan[1],servo_min, servo_max, rc_min, rc_max);
			double rud_trim_PWM = (double) output_scaling(handlerObject.rc_in.rc_chan[2],servo_min, servo_max, rc_min, rc_max);
			double thr_trim_PWM = (double) output_scaling(handlerObject.rc_in.rc_chan[3],servo_min, servo_max, rc_min, rc_max);

			// Ensure trim commands are within acceptable range
			ail_trim_PWM = ail_trim_PWM < ail_PWM_min ? ail_PWM_min : ail_trim_PWM;
			ail_trim_PWM = ail_trim_PWM > ail_PWM_max ? ail_PWM_max : ail_trim_PWM;
			ele_trim_PWM = ele_trim_PWM < ele_PWM_min ? ele_PWM_min : ele_trim_PWM;
			ele_trim_PWM = ele_trim_PWM > ele_PWM_max ? ele_PWM_max : ele_trim_PWM;
			rud_trim_PWM = rud_trim_PWM < rud_PWM_min ? rud_PWM_min : rud_trim_PWM;
			rud_trim_PWM = rud_trim_PWM > rud_PWM_max ? rud_PWM_max : rud_trim_PWM;
			thr_trim_PWM = thr_trim_PWM < thr_PWM_min ? thr_PWM_min : thr_trim_PWM;
			thr_trim_PWM = thr_trim_PWM > thr_PWM_max ? thr_PWM_max : thr_trim_PWM;

			// Convert to rad and percent throttle commands
			unfiltered_RC_ele_cmd = 0.01745*((-ele_PWM_consts[1]+sqrt(ele_PWM_consts[1]*ele_PWM_consts[1]-4.0*ele_PWM_consts[0]*(ele_PWM_consts[2]-ele_trim_PWM)))/(2.0*ele_PWM_consts[0])); // rad
			unfiltered_RC_ail_cmd = 0.01745*((ail_trim_PWM - ail_PWM_consts[1])/ail_PWM_consts[0]); // rad
			unfiltered_RC_rud_cmd = 0.01745*((rud_trim_PWM - rud_PWM_consts[1])/rud_PWM_consts[0]); // rad
			unfiltered_RC_thr_cmd = ((thr_trim_PWM-(double)thr_PWM_min)/((double)thr_PWM_max-(double)thr_PWM_min)); // percent throttle
			
			// Check unfiltered cmds are valid
			unfiltered_RC_ele_cmd = isnan(unfiltered_RC_ele_cmd) ? ele_trim : unfiltered_RC_ele_cmd;
			unfiltered_RC_ail_cmd = isnan(unfiltered_RC_ail_cmd) ? ail_trim : unfiltered_RC_ail_cmd;
			unfiltered_RC_rud_cmd = isnan(unfiltered_RC_rud_cmd) ? rud_trim : unfiltered_RC_rud_cmd;
			unfiltered_RC_thr_cmd = isnan(unfiltered_RC_thr_cmd) ? thr_trim : unfiltered_RC_thr_cmd;
			
			// Single-pole low-pass filter
			ele_trim += alpha * (unfiltered_RC_ele_cmd - ele_trim);
			ail_trim += alpha * (unfiltered_RC_ail_cmd - ail_trim);
			rud_trim += alpha * (unfiltered_RC_rud_cmd - rud_trim);
			thr_trim += alpha * (unfiltered_RC_thr_cmd - thr_trim);
		}	

		// When AP engages, lock trim values
		if(handlerObject.rc_in.rc_chan[ap_engage_chan]>=ap_engage_cutoff && !trim_values_set)
		{
			// Wait for PWM module to complete its messeges
			usleep(5000);	

			// Send trim value message to user
			std::cout<<"Elevator trim: " << ele_trim*180.0/3.14159 << " deg" << std::endl; 
			std::cout<<"Aileron trim: " << ail_trim*180.0/3.14159 << " deg" <<  std::endl;
			std::cout<<"Rudder trim: " << rud_trim*180.0/3.14159 << " deg" <<  std::endl;
			std::cout<<"Throttle trim: " << 100.0*thr_trim << " %" <<  std::endl;

			// Lock trim values
			trim_values_set = true;
		}

		// When AP disengages, unlock trim values
		else if(handlerObject.rc_in.rc_chan[ap_engage_chan]<ap_engage_cutoff && trim_values_set)
		{
			trim_values_set = false;

			#ifdef TEST
				initial_state_set = false;
			#endif
		}

		#ifdef TEST
			// Set initial states based on current test number
			if (trim_values_set && !initial_state_set && curr_test_number<num_tests)
			{
				// User interface
				std::cout << "Autopilot test " << curr_test_number+1 << " / " << num_tests << "..." << std::endl;

				// Set initial states
				for (int i = 0; i < 9; i++)
				{
					true_absolute_states[i] = initial_states[curr_test_number][i];
				}
				
				// Read the true absolute states with sensor noise
				unfiltered_vel = true_absolute_states[0] + get_rand()*state_noise[0];
				vel = vel_trim;
				unfiltered_AoA = true_absolute_states[1] + get_rand()*state_noise[1];
				AoA = AoA_trim;
				wyy = true_absolute_states[2] + get_rand()*state_noise[2];
				pit = true_absolute_states[3] + get_rand()*state_noise[3];
				unfiltered_bet = true_absolute_states[4] + get_rand()*state_noise[4];
				bet = bet_trim;
				wxx = true_absolute_states[5] + get_rand()*state_noise[5];
				wzz = true_absolute_states[6] + get_rand()*state_noise[6];
				rol = true_absolute_states[7] + get_rand()*state_noise[7];
				yaw = true_absolute_states[8] + get_rand()*state_noise[8];

				// Reset previous state memory
				vel_prev = 0.0;
				AoA_prev = 0.0;
				wyy_prev = 0.0;
				pit_prev = 0.0;
				bet_prev = 0.0;
				wxx_prev = 0.0;
				wzz_prev = 0.0;
				rol_prev = 0.0;

				// Update test number and initial state set conditions
				curr_test_number++;
				initial_state_set = true;

			}
		#else
			// Gather raw data from ADC and convert it to readable data
			ps1 = eval_1D_poly(ps1_consts, 2, (double)handlerObject.adc.data[0] ); // uCH0 to ps1
			ps2 = eval_1D_poly(ps2_consts, 2, (double)handlerObject.adc.data[1] ); // uCH1 to ps2
			ps3 = eval_1D_poly(ps3_consts, 2, (double)handlerObject.adc.data[2] ); // uCH2 to ps3
			ps4 = eval_1D_poly(ps4_consts, 2, (double)handlerObject.adc.data[3] ); // uCH3 to ps4
			ps5 = eval_1D_poly(ps5_consts, 2, (double)handlerObject.adc.data[4] ); // uCH4 to ps5

			// Calculate 5 hole probe coefficients
			p_bar = (ps2 + ps3 + ps4 + ps5)* 0.25; // in dPSI
			coeff_AoA = (ps4 - ps5) / (ps1 - p_bar); // unitless
			coeff_bet = (ps2 - ps3) / (ps1 - p_bar); // unitless
			coeff_pt = eval_2D_poly(coeff_pt_consts, 15, coeff_AoA, coeff_bet); // unitless
			coeff_ps = eval_2D_poly(coeff_ps_consts, 21, coeff_AoA, coeff_bet); // unitless

			// Use pressure coefficient data to calculate AoA, beta, total pressure, static pressure
			unfiltered_AoA = 0.01745 * eval_2D_poly(AoA_consts, 15, coeff_AoA, coeff_bet);    // in rad
			unfiltered_bet = 0.01745 * eval_2D_poly(bet_consts, 15, coeff_AoA, coeff_bet);    // in rad
			total_pressure = (ps1 - coeff_pt * (ps1 - p_bar)) * 6894.76; // in Pa
			static_pressure = (p_bar - coeff_ps * (ps1 - p_bar)) * 6894.76; // in Pa

			// If the pressure readings indicate imaginary velocity, assign the velocity as the previous good value
			if (total_pressure >= 0.0 || static_pressure >= 0.0 || abs(total_pressure) <= abs(static_pressure))
			{
				unfiltered_vel = vel_prev; // in m/s
			}
			else
			{
				unfiltered_vel = sqrt((2.0 * (abs(total_pressure) - abs(static_pressure))) / (rho)); // in m/s
			}

			// Gather INS data
			rol = 0.017453 * handlerObject.vnins.roll;  // in rad
			pit = 0.017453 * handlerObject.vnins.pitch; // in rad
			yaw = 0.017453 * handlerObject.vnins.yaw;   // in rad
			wxx = handlerObject.vnins.wx;    // in rad/s (roll rate)
			wyy = handlerObject.vnins.wy;    // in rad/s (pitch rate)
			wzz = handlerObject.vnins.wz;    // in rad/s (yaw rate)
		#endif

		// Bad state rejection (Assign previous good value of state if measured state is out of range)
		unfiltered_vel = (unfiltered_vel < vel_min || unfiltered_vel > vel_max) ? vel_prev : unfiltered_vel;
		unfiltered_AoA = (unfiltered_AoA < -AoA_lim || unfiltered_AoA > AoA_lim) ? AoA_prev : unfiltered_AoA;
		wyy = (wyy < -wyy_lim || wyy > wyy_lim) ? wyy_prev : wyy;
		pit = (pit < -pit_lim || pit > pit_lim) ? pit_prev : pit;
		unfiltered_bet = (unfiltered_bet < -bet_lim || unfiltered_bet > bet_lim) ? bet_prev : unfiltered_bet;
		wxx = (wxx < -wxx_lim || wxx > wxx_lim) ? wxx_prev : wxx;
		wzz = (wzz < -wzz_lim || wzz > wzz_lim) ? wzz_prev : wzz;
		rol = (rol < -rol_lim || rol > rol_lim) ? rol_prev : rol;
		
		// Apply single-pole low-pass filter to all 5 hole probe wind data
		AoA += alpha * (unfiltered_AoA - AoA);  // in rad
		bet += alpha * (unfiltered_bet - bet);  // in rad
		vel += alpha * (unfiltered_vel - vel);  // in m/s

		// Collect previous state values
		vel_prev = unfiltered_vel; // m/s
		AoA_prev = unfiltered_AoA; // rad
		wyy_prev = wyy; // rad/s
		pit_prev = pit; // rad
		bet_prev = unfiltered_bet; // rad
		wxx_prev = wxx; // rad/s
		wzz_prev = wzz; // rad/s
		rol_prev = rol; // rad

		// Calculate state errors
		states[0] = (vel - vel_trim);
		states[1] = (AoA - AoA_trim);
		states[2] = (wyy - wyy_trim);
		states[3] = (pit - pit_trim);
		states[4] = (bet - bet_trim);
		states[5] = (wxx - wxx_trim);
		states[6] = (wzz - wzz_trim);
		states[7] = (rol - rol_trim);
		states[8] = (yaw - yaw_trim);

		// Calculate input deltas based on state (u - u0) = -K * (x - x0)
		for (int i = 0; i < 11; i++)
		{
			inputs[i] = 0.0;
			for (int j = 0; j < 9; j++)
			{
				inputs[i] += -1.0*k[i][j]*states[j];
			}
		}

		// Convert angle commands to surface PWM
		ele_ang_cmd = 57.29578 * (inputs[0] + ele_trim); // in degrees
		ail_ang_cmd = 57.29578 * (inputs[1] + ail_trim); // in degrees
		rud_ang_cmd = 57.29578 * (inputs[2] + rud_trim); // in degrees
		ele_PWM_cmd = (int) eval_1D_poly(ele_PWM_consts, 3, ele_ang_cmd); // in PWM
		ail_PWM_cmd = (int) eval_1D_poly(ail_PWM_consts, 2, ail_ang_cmd); // in PWM 
		rud_PWM_cmd = (int) eval_1D_poly(rud_PWM_consts, 2, rud_ang_cmd); // in PWM

		// Ensure surface PWM commands are within safe limits
		ele_PWM_cmd = ele_PWM_cmd > ele_PWM_max ? ele_PWM_max : ele_PWM_cmd;
		ele_PWM_cmd = ele_PWM_cmd < ele_PWM_min ? ele_PWM_min : ele_PWM_cmd;
		ail_PWM_cmd = ail_PWM_cmd > ail_PWM_max ? ail_PWM_max : ail_PWM_cmd;
		ail_PWM_cmd = ail_PWM_cmd < ail_PWM_min ? ail_PWM_min : ail_PWM_cmd;
		rud_PWM_cmd = rud_PWM_cmd > rud_PWM_max ? rud_PWM_max : rud_PWM_cmd;
		rud_PWM_cmd = rud_PWM_cmd < rud_PWM_min ? rud_PWM_min : rud_PWM_cmd;

		// Convert throttle commands to PWM commands and ensure within safe limits
		for (int i = 0; i < 8; i++)
		{
			thr_PWM_cmd[i] = (int) thr_PWM_min + (thr_PWM_max - thr_PWM_min) * (inputs[i+3] + thr_trim);
			thr_PWM_cmd[i] = thr_PWM_cmd[i] > thr_PWM_max ? thr_PWM_max : thr_PWM_cmd[i];
			thr_PWM_cmd[i] = thr_PWM_cmd[i] < thr_PWM_min ? thr_PWM_min : thr_PWM_cmd[i];
		}

		// Send input PWM commands to PWM module
		acts.de = ele_PWM_cmd;
		acts.da = ail_PWM_cmd;
		acts.dr = rud_PWM_cmd;
		for (int i = 0; i < 8; i++)
		{
			acts.dt[i] = thr_PWM_cmd[i];
		}

		// Timestamp the actuator values
		acts.time_gps = get_gps_time(&handlerObject);

		// Publish the actuator values
		zcm.publish("ACTUATORS", &acts);

		#ifdef TEST
			// If the initial states are already set and a test is running, update the states based on the linear dynamics of the system
			if(trim_values_set && initial_state_set && curr_test_number<=num_tests)
			{
				// Log current unfiltered noisy states, filtered noisy states, and ground truth
				logfile_ap_test << acts.time_gps << " " << curr_test_number << " ";
				logfile_ap_test << unfiltered_vel << " " << vel << " " << true_absolute_states[0] << " ";
				logfile_ap_test << unfiltered_AoA << " " << AoA << " " << true_absolute_states[1] << " ";
				logfile_ap_test << wyy << " " << true_absolute_states[2] << " ";
				logfile_ap_test << pit << " " << true_absolute_states[3] << " ";
				logfile_ap_test << unfiltered_bet << " " << bet << " " << true_absolute_states[4] << " ";
				logfile_ap_test << wxx << " " << true_absolute_states[5] << " ";
				logfile_ap_test << wzz << " " << true_absolute_states[6] << " ";
				logfile_ap_test << rol << " " << true_absolute_states[7] << " ";
				logfile_ap_test << yaw << " " << true_absolute_states[8] << std::endl;
				
				// Calculate state errors of ground truth
				true_state_errors[0] = (true_absolute_states[0] - vel_trim);
				true_state_errors[1] = (true_absolute_states[1] - AoA_trim);
				true_state_errors[2] = (true_absolute_states[2] - wyy_trim);
				true_state_errors[3] = (true_absolute_states[3] - pit_trim);
				true_state_errors[4] = (true_absolute_states[4] - bet_trim);
				true_state_errors[5] = (true_absolute_states[5] - wxx_trim);
				true_state_errors[6] = (true_absolute_states[6] - wzz_trim);
				true_state_errors[7] = (true_absolute_states[7] - rol_trim);
				true_state_errors[8] = (true_absolute_states[8] - yaw_trim);
				
				// Step the ground truth state errors based on the linear system dynamics
				step_states( A, B, &true_state_errors, inputs, delta_t);
				
				// Update the ground truth from the ground truth state errors
				true_absolute_states[0] = (true_state_errors[0] + vel_trim);
				true_absolute_states[1] = (true_state_errors[1] + AoA_trim);
				true_absolute_states[2] = (true_state_errors[2] + wyy_trim);
				true_absolute_states[3] = (true_state_errors[3] + pit_trim);
				true_absolute_states[4] = (true_state_errors[4] + bet_trim);
				true_absolute_states[5] = (true_state_errors[5] + wxx_trim);
				true_absolute_states[6] = (true_state_errors[6] + wzz_trim);
				true_absolute_states[7] = (true_state_errors[7] + rol_trim);
				true_absolute_states[8] = (true_state_errors[8] + yaw_trim);
				
				// Apply noise to the ground truth for AP sensors
				unfiltered_vel = true_absolute_states[0] + get_rand()*state_noise[0];
				unfiltered_AoA = true_absolute_states[1] + get_rand()*state_noise[1];
				wyy = true_absolute_states[2] + get_rand()*state_noise[2];
				pit = true_absolute_states[3] + get_rand()*state_noise[3];
				unfiltered_bet = true_absolute_states[4] + get_rand()*state_noise[4];
				wxx = true_absolute_states[5] + get_rand()*state_noise[5];
				wzz = true_absolute_states[6] + get_rand()*state_noise[6];
				rol = true_absolute_states[7] + get_rand()*state_noise[7];
				yaw = true_absolute_states[8] + get_rand()*state_noise[8];
			}
		#endif

		// Sleep for 10 ms to remove CPU stress
		usleep(10000);				

	}

	// Publish terminating status
	module_stat.module_status = 0;
	zcm.publish("STATUS4",&module_stat);
	std::cout << "autopilot module exiting..." << std::endl;
	#ifdef TEST 
	logfile_ap_test.close();
	#endif

	// Stop ZCM
	zcm.stop();

	return 0;
}