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

#ifdef TEST
#include <stdlib.h>
#include <time.h>
#endif

// Message types
#include "autopilot_t.hpp"
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
double output_scaling(double in_val, double s_min, double s_max, double in_min, double in_max)
{
	return (s_min + (s_max-s_min)/(in_max-in_min)*(in_val-in_min));
}

#ifdef TEST

	// FE steps states based on dynamics, inputs, and current states
	void step_states(double** A, double** B, double* true_state_error, double* input_delta, double delta_t)
	{
		double Axi = 0.0;
		double Bui = 0.0;
		double new_state_errors[9];
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 9; j++)
			{
				Axi += A[i][j] * true_state_error[j];
			}
			for(int j = 0; j < 11; j++)
			{
				Bui += B[i][j] * input_delta[j];
			}
			new_state_errors[i] = true_state_error[i] + delta_t * (Axi + Bui);
			Axi = 0.0;
			Bui = 0.0;
		}
		for (int i = 0; i < 9; i++)
		{
			true_state_error[i] = new_state_errors[i];
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
	// ******************************************************************************************************** CALIBRATION DATA ******************************************************************************************************** //
	// Conversion constants for the adc inputs
	double adc_pres_consts[5][2] = { { 4.8261e-06, -0.19188 },
					 { 4.8269e-06, -0.19142 },
					 { 4.8141e-06, -0.19211 },
					 { 4.8152e-06, -0.19283 },
					 { 4.8028e-06, -0.19231 } };

	// Conversion constants for deflection commands to PWM commands
	double ail_PWM_consts[2] = { -19.9874, 1499.3311 };           // From deflection in degrees to PWM CMD
	double ele_PWM_consts[3] = { 0.027712, 10.6043, 1517.7811 };  // From deflection in degrees to PWM CMD
	double rud_PWM_consts[2] = { -12.3453, 1550.7985 };           // From deflection in degrees to PWM CMD

	// Conversion constants to convert adc raw data to state data
	double AoA_consts[15] = { 0.14417, -14.0401, -0.48222, -0.82991, -0.39334, -0.065129, 0.29331, 0.10864, 0.57212, 0.12463, 0.022992, 0.029209, 0.089836, 0.057237, 0.016901 };
	double bet_consts[15] = { -0.045676, 0.090171, 13.8048, 0.002027, 0.94476, 0.29254, 0.12192, -0.66955, -0.020875, -0.33687, 0.023934, -0.10505, -0.019041, -0.054968, -0.018293 };
	double coeff_ps_consts[21] = { 0.59235, -0.0032055, -0.0045759, -0.098045, 0.010147, -0.0977, -0.015851, -0.0024914, -0.022443, -0.0037574, -0.0042317, -0.0039405, 0.01926, -0.0014971, -0.0014967, -0.0007027, -0.0010546, 0.0041895, 6.6051e-05, 0.00048148, -0.00022731 };
	double coeff_pt_consts[15] = { -0.21483, 0.036139, 0.0037369, -0.12377, -0.034201, -0.11844, 0.0022027, 0.0040131, 0.0047189, 0.0026645, 0.00010707, 0.0023433, 0.0079094, 0.0034925, -0.001166 };
		
	// Conversion constants to convert state data to adc raw data
	# ifdef TEST
		double coeff_AoA_consts[15] = { -0.0016889, -0.067901, -0.0021985, 1.2588e-05, -6.8573e-05, 8.3843e-05, -2.0922e-05, 1.4181e-05, -3.8919e-05, 3.8844e-06, -7.4686e-07, 7.7933e-07, -1.5425e-06, 5.3011e-07, -8.3722e-08 };
		double coeff_bet_consts[15] = { -0.0045874, 0.0034246, 0.068988, 5.9615e-05, -0.00012652, 4.2608e-05, 1.0995e-06, 5.2217e-05, -1.5044e-05, 1.7282e-05, 2.712e-09, 1.1887e-06, -9.8476e-07, 1.3198e-06, -2.7764e-07 };
		double back_calculated_coeff_AoA = 0.0;
		double back_calculated_coeff_bet = 0.0;
		double back_calculated_coeff_pt = 0.0;
		double back_calculated_coeff_ps = 0.0;
		double back_calculated_total_pressure = 0.0;
		double back_calculated_static_pressure = 0.0;
		double back_calculated_adc_pres[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
		double back_calculated_adc_raw[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	#endif

	// ******************************************************************************************************** AUTOPILOT CONFIG DATA ******************************************************************************************************** //
	// Autopilot type name
	string ap_type;
	
	// Engine that is set to fail
	string engine_fail_number;
	
	// Atmospheric conditions
	double rho; // kg/m^3
	
	// Input PWM limits
	double input_pwm_limits[11][2];  // Ele, ail, rud, thr1-8 PWM

	// State trim conditions
	double trim_state[9];  // vel, aoa, wyy,   pit, bet, wxx,   wzz,   rol, yaw
			       // m/s, rad, rad/s, rad, rad, rad/s, rad/s, rad, rad
	
	// Yaw trim rate parameters
	double max_yaw_trim_rate;  // rad/s

	// State limits. Any higher or lower values are considered faulty readings.
	double state_limits[9][2];  // vel, aoa, wyy,   pit, bet, wxx,   wzz,   rol, yaw
	                            // m/s, rad, rad/s, rad, rad, rad/s, rad/s, rad, rad

	// Config loading variables
	string dump;
	std::ifstream ap_config_stream;

	// Load config data
	ap_config_stream.open("/home/pi/adept-fc/config_files/autopilot_main.config");
	ap_config_stream >> dump >> ap_type;
	ap_config_stream >> dump >> engine_fail_number;
	ap_config_stream >> dump >> rho;
	ap_config_stream >> dump >> max_yaw_trim_rate;
	
	#ifdef TEST
		cout << "AUTOPILOT CONFIG DATA:" << endl;
		cout << ap_type << endl;
		cout << rho << endl;
		cout << max_yaw_trim_rate << endl;
	#endif
	
	#ifdef TEST
		cout << "\nINPUT PWM LIMITS:" << endl;
	#endif
	for (int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			ap_config_stream >> dump >> input_pwm_limits[i][j];
			
			#ifdef TEST
				cout << input_pwm_limits[i][j] << endl;
			#endif
		}
	}
	for (int i = 4; i < 11; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			input_pwm_limits[i][j] = input_pwm_limits[3][j];
				
			#ifdef TEST
				cout << input_pwm_limits[i][j] << endl;
			#endif
		}
	}
	#ifdef TEST
		cout << "\nTRIM STATE:" << endl;
	#endif
	for (int i = 0; i < 9; i++)
	{
		ap_config_stream >> dump >> trim_state[i];
		
		#ifdef TEST
			cout << trim_state[i] << endl;
		#endif
	}
	#ifdef TEST
		cout << "\nSTATE LIMITS:" << endl;
	#endif
	for (int i = 0; i < 9; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			ap_config_stream >> dump >> state_limits[i][j];
			
			#ifdef TEST
				cout << state_limits[i][j] << endl;
			#endif
		}
	}
	
	#ifdef TEST
		cout << endl;
	#endif
	
	ap_config_stream.close();

	// ******************************************************************************************************** GAIN DATA ******************************************************************************************************** //
	// Controller constants
	double k[11][12];
	
	// Load config data
	ifstream gain_stream;
	string ap_gains_load_path = "/home/pi/adept-fc/config_files/autopilot_gains/" + ap_type + ".dat";
	cout << "AP Gains @ " << ap_gains_load_path << endl;
	gain_stream.open(ap_gains_load_path);
	if(!gain_stream.good())
	{
		std::cout << "WARNING: COULD NOT FIND AP GAINS." << endl;
		return 1;
	}
	for (int i = 0; i < 11; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			gain_stream >> k[i][j];
			
			#ifdef TEST
				cout << k[i][j] << " ";
			#endif
		}
		
		#ifdef TEST
			cout << endl;
		#endif
	}
	
	#ifdef TEST
		cout << endl;
	#endif
	
	gain_stream.close();
	
	// ******************************************************************************************************** SEF GAIN DATA ******************************************************************************************************** //
	// SEF controller constants
	double k_fail[11][12];
	ap_gains_load_path = "/home/pi/adept-fc/config_files/autopilot_gains/SEF_int.dat";
	cout << "AP SEF Gains @ " << ap_gains_load_path << endl;
	gain_stream.open(ap_gains_load_path);
	if(!gain_stream.good())
	{
		std::cout << "WARNING: COULD NOT FIND AP SEF GAINS." << endl;
		return 1;
	}
	for (int i = 0; i < 11; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			gain_stream >> k_fail[i][j];
			
			#ifdef TEST
				cout << k[i][j] << " ";
			#endif
		}
		
		#ifdef TEST
			cout << endl;
		#endif
	}
	#ifdef TEST
		cout << endl;
	#endif
	gain_stream.close();

	
	// Failed throttle trims
	double sef_thr_delta[8];
	sef_thr_delta_load_path = "/home/pi/adept-fc/config_files/SEF_trims/SEF_" + engine_fail_number + ".dat";
	cout << "AP SEF Thr Cmds @ " << sef_thr_delta_load_path << endl;
	gain_stream.open(sef_thr_delta_load_path);
	if(!gain_stream.good())
	{
		std::cout << "WARNING: COULD NOT FIND AP SEF THR CMDS." << endl;
		return 1;
	}
	for (int i = 0; i < 8; i++)
	{
		gain_stream >> dump >> sef_thr_delta[i];
		
		#ifdef TEST
			cout << sef_thr_delta[i] << endl;
		#endif
	}
	#ifdef TEST
		cout << endl;
	#endif
	gain_stream.close();

	// ******************************************************************************************************** AUTOPILOT TEST PARAMETERS ******************************************************************************************************** //
	#ifdef TEST
		
		// State noise
		srand (time(NULL));
		double state_noise[9] = { 9.00e-1,  8.73e-3,  5.24e-4,  5.24e-4,  8.73e-3,  5.24e-4,  3.49e-3,  5.24e-4,  3.49e-3 }; 
					//vel m/s,  AoA rad,  wyy 1/s,  pit rad,  bet rad,  wxx 1/s,  wzz 1/s,  rol rad,  yaw rad
		
		// Initialize simulated state arrays
		double true_state[9];
		double* true_state_error = new double[9];
		for (int i = 0; i < 9; i++)
		{
			true_state[i] = trim_state[i];
			true_state_error[i] = 0.0;
		}

		// Test parameters
		int num_tests=5;
		int curr_test_number = 0;
		bool initial_state_set = false;
		
		// Set initial states
		double initial_states[num_tests][9];
		for(int i = 0; i < num_tests; i++)
		{
			for(int j = 0; j < 9; j++)
			{
				initial_states[i][j] = trim_state[j];
			}
		}
		
		// Velocity up test
		initial_states[1][0] += 0.25*(state_limits[0][1] - state_limits[0][0]);  
		
		// Pitch up test
		initial_states[2][3] += 0.25* state_limits[3][1];  
	
		// Roll right test
		initial_states[3][7] += 0.25* state_limits[7][1];  
		
		// Velocity up, pitch up, roll right test
		initial_states[4][0] += 0.25*(state_limits[0][1] - state_limits[0][0]);  
		initial_states[4][3] += 0.25* state_limits[3][1];
		initial_states[4][7] += 0.25* state_limits[7][1];

	// ******************************************************************************************************** SYSTEM DYNAMICS ******************************************************************************************************* //
		// Allocate space for system dynamics matrices
		double** A = new double*[9];
		double** B = new double*[9];
		for(int i = 0; i < 9; i++)
		{
			A[i] = new double[9];
			B[i] = new double[11];
		}

		// Load A matrix
		ifstream dynamics_stream;
		dynamics_stream.open("/home/pi/adept-fc/config_files/matrices.dat");
		cout << "A MATRIX:" << endl;
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 9; j++)
			{
				dynamics_stream >> A[i][j];
				cout << A[i][j] << " ";
			}
			cout << endl;
		}
		cout << endl;
		
		// Load B matrix
		cout << "B MATRIX:" << endl;
		for(int i = 0; i < 9; i++)
		{
			for(int j = 0; j < 11; j++)
			{
				dynamics_stream >> B[i][j];
				cout << B[i][j] << " ";
				
			}
			cout << endl;
		}
		cout << endl;
		dynamics_stream.close();
		
	#endif

	// ******************************************************************************************************** INPUT TRIM DATA ******************************************************************************************************** //
	// Declare AP RC channels and cutoffs
	ifstream rc_config_stream;
	int rc_dump;
	double rc_min;
	double rc_max;
	int ap_arm_chan;
	int ap_arm_cutoff;
	int ap_engage_chan;
	int ap_engage_cutoff;
	int sef_arm_chan;
	int sef_arm_cutoff;

	// Load rc configuration variables (Ignore the unnecessary ones)
	rc_config_stream.open("/home/pi/adept-fc/config_files/pwm_out.config");
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> rc_min;
	rc_config_stream >> dump >> rc_max;
	rc_config_stream >> dump >> ap_arm_chan;
	rc_config_stream >> dump >> ap_arm_cutoff;
	rc_config_stream >> dump >> rc_dump;
	rc_config_stream >> dump >> ap_engage_chan;
	rc_config_stream >> dump >> ap_engage_cutoff;
	rc_config_stream >> dump >> sef_arm_chan;
	rc_config_stream >> dump >> sef_arm_cutoff;
	rc_config_stream.close();
	
	// Scream into the void if testing
	#ifdef TEST
		cout << "INPUT TRIM DATA:" << endl;
		cout << rc_min << endl;
		cout << rc_max << endl;
		cout << ap_arm_chan << endl;
		cout << ap_arm_cutoff << endl;
		cout << ap_engage_chan << endl;
		cout << ap_engage_cutoff << endl;
		cout << sef_arm_chan << endl;
		cout << sef_arm_cutoff << endl << endl;
	#endif

	// RC mode settings
	bool trim_values_set = false;
	bool AP_armed_engaged = false;
	bool sef_mode = false;
	bool sef_msg_thrown = false;
	double rc_rud_trim;
	double rc_rud_delta;
	double rc_thr_trim = 0.0;
	double rc_thr_delta;
	
	// Input trim values
	int rc_to_input_mapping[11] = { 1, 0, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
	double input_pmw_trim[11];
	double input_trim[11];
	double filtered_input_trim[11];
	for (int i = 0; i < 11; i++)
	{
		input_pmw_trim[i] = 0.0;
		input_trim[i] = 0.0;
		filtered_input_trim[i] = 0.0;
	}
	
	// Time constant variables
	double delta_t = 0.01;    // seconds
	double time_const = 0.1;  // seconds
	double alpha = 1.0 - exp(-delta_t / time_const);
	

	// ******************************************************************************************************** OTHER VARIABLES ******************************************************************************************************** //
	// Terms used in 5 hole probe to state conversion
	double adc_pres[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	double p_bar = 0.0;
	double coeff_AoA = 0.0;
	double coeff_bet = 0.0;
	double coeff_pt = 0.0;
	double coeff_ps = 0.0;
	double total_pressure = 0.0;
	double static_pressure = 0.0;
	
	// State variables
	double measured_state[9];
	double filtered_state[9];
	double prev_state[9];
	for (int i = 0; i < 9; i++)
	{
		measured_state[i] = trim_state[i];
		filtered_state[i] = trim_state[i];
		prev_state[i] = trim_state[i];
	}
	double filtered_state_error[12];
	for (int i = 0; i < 12; i++)
	{
		filtered_state_error[i] = 0.0;
	}
	
	// Input variables
	double* input_delta = new double[11];
	double input_cmd[11];
	double input_pwm_cmd[11];
	for (int i = 0; i < 11; i++) 
	{
		input_delta[i] = 0.0;
		input_cmd[i] = filtered_input_trim[i];
		input_pwm_cmd[i] = input_pmw_trim[i];
	}

	// ******************************************************************************************************** ZCM AND UI ******************************************************************************************************** //
	// Initialize zcm and message objects
	zcm::ZCM zcm{ "ipc" };

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
	
	// For publishing all other data from this module
	actuators_t acts;
	memset(&acts, 0, sizeof(acts));
	autopilot_t autopilot;
	memset(&autopilot, 0, sizeof(autopilot));
    

	// Run zcm as a separate thread
	zcm.start();
	#ifdef TEST
		cout << "WARNING autopilot started in test mode. DO NOT FLY." << endl;
	#else
		cout << "autopilot started" << endl;
	#endif

	// ******************************************************************************************************** AP MAIN LOOP ******************************************************************************************************** //
	while (!handlerObject.stat.should_exit)
	{
		// Publish the status of this module
		zcm.publish("STATUS4", &module_stat);

		// Every loop if the trim values are not already set, update the input trim, yaw trim, and rc_rud trim. Reset the integrals
		if(!trim_values_set)
		{
			// Gather input PWM commands from RC
			for (int i = 0; i < 11; i++)
			{
				input_pmw_trim[i] = output_scaling((double)handlerObject.rc_in.rc_chan[rc_to_input_mapping[i]], input_pwm_limits[i][0], input_pwm_limits[i][1], rc_min, rc_max);
				
				input_pmw_trim[i] = input_pmw_trim[i] < input_pwm_limits[i][0] ? input_pwm_limits[i][0] : input_pmw_trim[i];
				input_pmw_trim[i] = input_pmw_trim[i] > input_pwm_limits[i][1] ? input_pwm_limits[i][1] : input_pmw_trim[i];
			}

			// Convert to rad and percent throttle commands
			input_trim[0] = 0.01745*((-ele_PWM_consts[1]+sqrt(ele_PWM_consts[1]*ele_PWM_consts[1]-4.0*ele_PWM_consts[0]*(ele_PWM_consts[2]-input_pmw_trim[0])))/(2.0*ele_PWM_consts[0])); // rad
			input_trim[1] = 0.01745*((input_pmw_trim[1] - ail_PWM_consts[1])/ail_PWM_consts[0]); // rad
			input_trim[2] = 0.01745*((input_pmw_trim[2] - rud_PWM_consts[1])/rud_PWM_consts[0]); // rad
			for (int i = 3; i < 11; i++)
			{
				input_trim[i] = ((input_pmw_trim[i] - input_pwm_limits[i][0]) / (input_pwm_limits[i][1] - input_pwm_limits[i][0])); // decimal percent throttle
			}
			
			// Check unfiltered cmds are valid and SPLP filter
			for (int i = 0; i < 11; i++)
			{
				input_trim[i] = isnan(input_trim[i]) ? filtered_input_trim[i] : input_trim[i];
				
				filtered_input_trim[i] += alpha * (input_trim[i] - filtered_input_trim[i]);
			}
			
			// Set the yaw trim value to current heading
			trim_state[8] = 0.017453 * handlerObject.vnins.yaw;   // in rad
			rc_rud_trim = (double)handlerObject.rc_in.rc_chan[2]; // in rc
			
			// Reset the integration to prevent windup while AP is disarmed or disengaged
			for(int i = 9; i < 12; i++)
			{
				filtered_state_error[i] = 0.0;
			}

		}	

		// Check if the AP is both armed and engaged (to prevent integral windup when AP is not running)
		AP_armed_engaged = (handlerObject.rc_in.rc_chan[ap_engage_chan]>=ap_engage_cutoff) && (handlerObject.rc_in.rc_chan[ap_arm_chan]>=ap_arm_cutoff);

		// On AP arm and engage rising edge, lock trim values
		if(AP_armed_engaged && !trim_values_set)
		{
			// Wait for PWM module to complete its messeges
			usleep(5000);	

			// Send trim value message to user
			cout<< "\nElevator trim: " << filtered_input_trim[0]*180.0/3.14159 << " deg" << endl; 
			cout<< "Aileron trim: " << filtered_input_trim[1]*180.0/3.14159 << " deg" <<  endl;
			cout<< "Rudder trim: " << filtered_input_trim[2]*180.0/3.14159 << " deg" <<  endl;
			for(int i = 3; i < 11; i++)
			{
				cout<< "Throttle trim: " << 100.0*filtered_input_trim[3] << " %" <<  endl;
			}
			cout<< "***********************************" <<  endl;
			cout<< "Heading Lock: " << trim_state[8]*180.0/3.14159 << " deg" <<  endl;
			cout<< "RC Rud Trim: " << rc_min << " - " << rc_rud_trim << " - " << rc_max << endl;
			cout<< "RC Thr Trim: " << rc_min << " - " << rc_thr_trim << " - " << rc_max << endl;

			// Lock trim values
			trim_values_set = true;
		}

		// On AP arm and engage falling edge, unlock trim values
		else if(!AP_armed_engaged && trim_values_set)
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
				cout << "Autopilot test " << curr_test_number+1 << " / " << num_tests << "..." << endl;

				// Set initial true state
				for (int i = 0; i < 9; i++)
				{
					true_state[i] = initial_states[curr_test_number][i];
				}
				
				// Set the yaw trim value
				trim_state[8] = true_state[8];

				// Reset state and inputs
				for (int i = 0; i < 9; i++)
				{
					filtered_state[i] = trim_state[i];
					filtered_state_error[i] = 0.0;
					prev_state[i] = trim_state[i];
				}
				for (int i = 0; i < 11; i++) 
				{
					input_delta[i] = 0.0;
					input_cmd[i] = filtered_input_trim[i];
					input_pwm_cmd[i] = input_pmw_trim[i];
				}
		

				// Update test number and initial state set conditions
				curr_test_number++;
				initial_state_set = true;
			}
			
			// If the initial states are already set and a test is running, update the states based on the linear dynamics of the system
			else if(trim_values_set && initial_state_set && curr_test_number<=num_tests)
			{
				
				// Calculate true state error
				for (int i = 0; i < 9; i++) 
				{
					true_state_error[i] = true_state[i] - trim_state[i];
				}
				true_state_error[8] = true_state_error[8] > 3.1416 ? true_state_error[8] - 6.2832 : true_state_error[8];
				true_state_error[8] = true_state_error[8] < -3.1416 ? true_state_error[8] + 6.2832 : true_state_error[8];
				
				// Step the ground truth state errors based on the linear system dynamics
				step_states( A, B, true_state_error, input_delta, delta_t);
				
				// Update the ground truth from the ground truth state errors
				for (int i = 0; i < 9; i++) 
				{
					true_state[i] = true_state_error[i] + trim_state[i];
				}
				true_state[8] = true_state[8] > 3.1416 ? true_state[8] - 6.2832 : true_state[8];
				true_state[8] = true_state[8] < -3.1416 ? true_state[8] + 6.2832 : true_state[8];
			}
			
			// Measure the true states
			for(int i = 0; i < 9; i++)
			{
				measured_state[i] = true_state[i] + get_rand()*state_noise[i];
			}
			measured_state[8] = measured_state[8] > 3.1416 ? measured_state[8] - 6.2832 : measured_state[8];
			measured_state[8] = measured_state[8] < -3.1416 ? measured_state[8] + 6.2832 : measured_state[8];
			
			// Calculate static and total pressure based on state
			back_calculated_total_pressure = (0.5 * rho * measured_state[0]*measured_state[0] + 101325.0) * 0.000145038;  // in PSIA
			back_calculated_static_pressure = 14.69597;  // in PSIA
			
			// Convert vel, aoa, and bet measured state data to ADC calibration consts
			back_calculated_coeff_AoA = eval_2D_poly(coeff_AoA_consts, 15, 57.29578 * measured_state[1], 57.29578 * measured_state[4]);    // unitless
			back_calculated_coeff_bet = eval_2D_poly(coeff_bet_consts, 15, 57.29578 * measured_state[1], 57.29578 * measured_state[4]);    // unitless
			back_calculated_coeff_pt = eval_2D_poly(coeff_pt_consts, 15, back_calculated_coeff_AoA, back_calculated_coeff_bet);    // unitless
			back_calculated_coeff_ps = eval_2D_poly(coeff_ps_consts, 21, back_calculated_coeff_AoA, back_calculated_coeff_bet);    // unitless
			
			// Convert back calculated calibration consts to 5 hole probe differential pressures
			back_calculated_adc_pres[0] = ( (1.0+back_calculated_coeff_ps) * (back_calculated_static_pressure-back_calculated_total_pressure) ) / (1.0+back_calculated_coeff_ps-back_calculated_coeff_pt); // in dPSI
			back_calculated_adc_pres[1] = 0.0; // in dPSI
			back_calculated_adc_pres[2] = ( back_calculated_coeff_bet * (back_calculated_static_pressure-back_calculated_total_pressure) ) / (1.0+back_calculated_coeff_ps-back_calculated_coeff_pt); // in dPSI
			back_calculated_adc_pres[3] = ( (4.0*back_calculated_coeff_ps+back_calculated_coeff_AoA-back_calculated_coeff_bet) * (back_calculated_static_pressure-back_calculated_total_pressure) ) / (2.0 * (1.0+back_calculated_coeff_ps-back_calculated_coeff_pt)); // in dPSI
			back_calculated_adc_pres[4] = ( (4.0*back_calculated_coeff_ps-back_calculated_coeff_AoA-back_calculated_coeff_bet) * (back_calculated_static_pressure-back_calculated_total_pressure) ) / (2.0 * (1.0+back_calculated_coeff_ps-back_calculated_coeff_pt)); // in dPSI
			
			// Convert back calculated 5 hole probe pressures to ADC raw values
			for( int i = 0; i < 5; i++ )
			{
				back_calculated_adc_raw[i] = round( (back_calculated_adc_pres[i] - adc_pres_consts[i][1]) / adc_pres_consts[i][0] );
			}
			
			// Gather raw data from backwards ADC and convert it to dPSI valuesa
			for (int i = 0; i < 5; i++)
			{
				adc_pres[i] = eval_1D_poly(adc_pres_consts[i], 2, back_calculated_adc_raw[i] );  // in dPSI
			}
			
		#else
			// Gather INS data
			measured_state[3] = 0.017453 * handlerObject.vnins.pitch; // in rad
			measured_state[7] = 0.017453 * handlerObject.vnins.roll;  // in rad
			measured_state[8] = 0.017453 * handlerObject.vnins.yaw;   // in rad
			measured_state[2] = handlerObject.vnins.wy;    // in rad/s (pitch rate)
			measured_state[5] = handlerObject.vnins.wx;    // in rad/s (roll rate)
			measured_state[6] = handlerObject.vnins.wz;    // in rad/s (yaw rate)
			
			// Gather raw data from ADC and convert it to dPSI values
			for (int i = 0; i < 5; i++)
			{
				adc_pres[i] = eval_1D_poly(adc_pres_consts[i], 2, (double)handlerObject.adc.data[i] );  // in dPSI
			}
			
		#endif
			
		// Calculate 5 hole probe coefficients
		p_bar = (adc_pres[1] + adc_pres[2] + adc_pres[3] + adc_pres[4])* 0.25; // in dPSI
		coeff_AoA = (adc_pres[3] - adc_pres[4]) / (adc_pres[0] - p_bar);       // unitless
		coeff_bet = (adc_pres[2] - adc_pres[1]) / (adc_pres[0] - p_bar);       // unitless
		coeff_pt = eval_2D_poly(coeff_pt_consts, 15, coeff_AoA, coeff_bet);    // unitless
		coeff_ps = eval_2D_poly(coeff_ps_consts, 21, coeff_AoA, coeff_bet);    // unitless

		// Use pressure coefficient data to calculate AoA, beta
		measured_state[1] = 0.01745 * eval_2D_poly(AoA_consts, 15, coeff_AoA, coeff_bet);    // in rad
		measured_state[4] = 0.01745 * eval_2D_poly(bet_consts, 15, coeff_AoA, coeff_bet);    // in rad

		// If the pressure readings indicate imaginary velocity, assign the velocity as the previous good value
		total_pressure = abs(6894.76*(adc_pres[0] - coeff_pt * (adc_pres[0] - p_bar)) - 101325.0);  // in PaA
		static_pressure = abs(6894.76*(p_bar - coeff_ps * (adc_pres[0] - p_bar)) - 101325.0); // in PaA
		if ( total_pressure <= 0.0 || static_pressure <= 0.0 || (total_pressure - static_pressure) <= 0.0 )
		{
			measured_state[0] = prev_state[0]; // in m/s
		}
		else
		{
			measured_state[0] = sqrt(2.0 * (total_pressure - static_pressure) / rho); // in m/s
		}

		// Bad state rejection, collect previous state values, filter 5-hole probe data
		for (int i = 0 ; i < 9; i++)
		{
			measured_state[i] = ( measured_state[i] < state_limits[i][0] || measured_state[i] > state_limits[i][1] ) ? prev_state[i] : measured_state[i];
			prev_state[i] = measured_state[i];
			
			if ( i==0 || i==1 || i==4 )
			{
				filtered_state[i] = filtered_state[i] + alpha * (measured_state[i] - filtered_state[i]);
			}
			else
			{
				filtered_state[i] = measured_state[i];
			}
		}

		// Calculate the pilot commanded yaw delta
		rc_rud_delta =  2.0 * ((double)handlerObject.rc_in.rc_chan[2] - rc_rud_trim) / (rc_max - rc_min);
		rc_rud_delta = rc_rud_delta > 1.0 ? 1.0 : rc_rud_delta;
		rc_rud_delta = rc_rud_delta < -1.0 ? -1.0 : rc_rud_delta;
		rc_rud_delta = abs(rc_rud_delta) < 0.10 ? 0.0 : rc_rud_delta;
		
		// Calculate the pilot commanded throttle delta
		rc_thr_delta = ((double)handlerObject.rc_in.rc_chan[3] - rc_thr_trim) / (rc_min - rc_thr_trim);
		rc_thr_delta = rc_thr_delta < 0.0 ? 0.0 : rc_thr_delta;
		rc_thr_delta = rc_rud_delta > 1.0 ? 1.0 : rc_thr_delta;
		
		// Update the yaw trim value based on pilot RC inputs
		trim_state[8] += (max_yaw_trim_rate * rc_rud_delta) * delta_t;
		trim_state[8] = trim_state[8] > 3.1416 ? trim_state[8] - 6.2832 : trim_state[8];
		trim_state[8] = trim_state[8] < -3.1416 ? trim_state[8] + 6.2832 : trim_state[8];
		
		// Calculate the yaw integral error ensuring that it's magnitude is always less than pi
		double yaw_int_error = ( trim_state[8] - filtered_state[4] ) - filtered_state[8];
		yaw_int_error = yaw_int_error > 3.1416 ? yaw_int_error - 6.2832 : yaw_int_error;
		yaw_int_error = yaw_int_error < -3.1416 ? yaw_int_error + 6.2832 : yaw_int_error;
		
		// Integrate vel, pitch, and yaw
		filtered_state_error[9] += (trim_state[0] - filtered_state[0]) * delta_t;
		filtered_state_error[10] += (filtered_state[1] - filtered_state[3]) * delta_t;
		filtered_state_error[11] += yaw_int_error * delta_t;

		// Calculate the yaw error ensuring that its magnitude is always less than pi
		double yaw_error = filtered_state[8] - ( trim_state[8] - filtered_state[4] );
		yaw_error = yaw_error > 3.1416 ? yaw_error - 6.2832 : yaw_error;
		yaw_error = yaw_error < -3.1416 ? yaw_error + 6.2832 : yaw_error;

		// Calculate state errors
		for (int i = 0; i < 8; i++)
		{
			filtered_state_error[i] = filtered_state[i] - trim_state[i];
		}
		filtered_state_error[8] = yaw_error;

		// Single engine failure mode calculation
		sef_mode = handlerObject.rc_in.rc_chan[sef_arm_chan]>=sef_arm_cutoff;
		if(sef_mode)
		{
			if (!sef_msg_thrown)
			{
				std::cout << "SEF MODE ARMED." << std::endl;
				sef_msg_thrown = true;
			}
		}
		else
		{
			rc_thr_trim = (double)handlerObject.rc_in.rc_chan[3];
			sef_msg_thrown = false;
		}

		// Calculate input deltas and input commands
		if (!sef_mode)
		{
			for (int i = 0; i < 11; i++)
			{
				input_delta[i] = 0.0;
				for (int j = 0; j < 12; j++)
				{
					input_delta[i] += -1.0*k[i][j]*filtered_state_error[j];
				}
				
				input_cmd[i] = input_delta[i] + filtered_input_trim[i];
			}
		}
		
		// Calculate input deltas and input commands IN SEF MODE
		else
		{
			for (int i = 0; i < 3; i++)
			{
				input_delta[i] = 0.0;
				for (int j = 0; j < 12; j++)
				{
					input_delta[i] += -1.0*k_fail[i][j]*filtered_state_error[j];
				}
				
				input_cmd[i] = input_delta[i] + filtered_input_trim[i];
			}
			for (int i = 3; i < 11; i++)
			{
				input_delta[i] = sef_thr_delta[i];
				input_cmd[i] = (rc_thr_delta)*(input_delta[i] + 0.4919) + (1.0-rc_thr_delta)*0.4919;
			}	
		}


		// Convert input commands to PWM
		input_pwm_cmd[0] = eval_1D_poly(ele_PWM_consts, 3, 57.29578 * input_cmd[0]); // in PWM
		input_pwm_cmd[1] = eval_1D_poly(ail_PWM_consts, 2, 57.29578 * input_cmd[1]); // in PWM
		input_pwm_cmd[2] = eval_1D_poly(rud_PWM_consts, 2, 57.29578 * input_cmd[2]); // in PWM
		for (int i = 3; i < 11; i++)
		{
			input_pwm_cmd[i] = input_pwm_limits[i][0] + (input_pwm_limits[i][1] - input_pwm_limits[i][0]) * (input_cmd[i]);
		}
		
		// Ensure PWM commands are within safe limits
		for (int i = 0; i < 11; i++)
		{
				input_pwm_cmd[i] = input_pwm_cmd[i] < input_pwm_limits[i][0] ? input_pwm_limits[i][0] : input_pwm_cmd[i];
				input_pwm_cmd[i] = input_pwm_cmd[i] > input_pwm_limits[i][1] ? input_pwm_limits[i][1] : input_pwm_cmd[i];
		}

		// Send input PWM commands to PWM module
		acts.de = input_pwm_cmd[0];
		acts.da = input_pwm_cmd[1];
		acts.dr = input_pwm_cmd[2];
		for (int i = 0; i < 8; i++)
		{
			acts.dt[i] = input_pwm_cmd[i+3];
		}

		// Timestamp the actuator values
		acts.time_gps = get_gps_time(&handlerObject);
		
		// Gather input trim data
		autopilot.ele_trim = filtered_input_trim[0] * 57.29577951;  // In deg
		autopilot.ail_trim = filtered_input_trim[1] * 57.29577951;  // In deg
		autopilot.rud_trim = filtered_input_trim[2] * 57.29577951;  // In deg
		for ( int i = 0; i < 8; i++)
		{
			autopilot.thr_trim[i] = filtered_input_trim[i] * 100.0;  // In percent
		}
		
		// Gather yaw rate data
		autopilot.yaw_trim_rate = max_yaw_trim_rate * rc_rud_delta * 57.29577951;  // In deg/s
		
		// Gather AP module status
		autopilot.ap_armed_and_engaged = AP_armed_engaged ? 1 : 0;
		
		// Gather AP module sef status
		autopilot.sef_mode = sef_mode ? 1 : 0;
		
		// Gather ADC conversion data
		#ifdef TEST
			for ( int i = 0; i < 5; i++)
			{
				autopilot.adc_pres_raw[i] = back_calculated_adc_raw[i];
				autopilot.adc_pres_dPSI[i] = adc_pres[i];
			}
		#else
			for ( int i = 0; i < 5; i++)
			{
				autopilot.adc_pres_raw[i] = (double)handlerObject.adc.data[i];
				autopilot.adc_pres_dPSI[i] = adc_pres[i];
			}
		#endif
		autopilot.p_bar_dPSI = p_bar;
		autopilot.Ca = coeff_AoA;
		autopilot.Cb = coeff_bet;
		autopilot.Cpt = coeff_pt;
		autopilot.Cps = coeff_ps;
		autopilot.Pt_Pa = total_pressure;
		autopilot.Ps_Pa = static_pressure;
		autopilot.rho = rho;
		
		// Gather state data
		for ( int i = 0; i < 9; i++ )
		{
			if ( i == 0 )
			{
				autopilot.state[i] = filtered_state[i];              // in m/s
				autopilot.state_error[i] = filtered_state_error[i];  // in m/s
			}
			else
			{
				autopilot.state[i] = filtered_state[i] * 57.29577951;              // in deg or deg/s
				autopilot.state_error[i] = filtered_state_error[i] * 57.29577951;  // in deg or deg/s
			}
		}
		autopilot.integral_ref[0] = trim_state[0];  // in m/s
		autopilot.integral_ref[1] = filtered_state[1] * 57.29577951;  // in deg
		autopilot.integral_ref[2] = (trim_state[8] - filtered_state[4]) * 57.29577951;  // in deg
		autopilot.integral[0] = filtered_state_error[9];   // in m
		autopilot.integral[1] = filtered_state_error[10] * 57.29577951;  // in deg-s
		autopilot.integral[2] = filtered_state_error[11] * 57.29577951;  // in deg-s
		
		// Gather input command
		for ( int i = 0; i < 11; i ++ )
		{
			if ( i == 0 || i == 1 || i == 2 )
			{
				autopilot.input_cmd[i] = 57.29578 * input_cmd[i];  // in deg
			}
			else
			{
				autopilot.input_cmd[i] = 100.0 * input_cmd[i];  // in percent
			}
		}
		
		// Gather the true states if running in test mode
		#ifdef TEST
		for ( int i = 0; i < 9; i++ )
		{
			if ( i == 0 )
			{
				autopilot.true_state[i] = true_state[i];  // in m/s
			}
			else
			{
				autopilot.true_state[i] = true_state[i] * 57.29577951;  // in deg or deg/s
			}
			
		}
		#endif
		
		// Time stamp data
		autopilot.time_gps = get_gps_time(&handlerObject);

		// Publish the actuator and sutopilot values
		zcm.publish("ACTUATORS", &acts);
		zcm.publish("AUTOPILOT", &autopilot);

		// Sleep for 10 ms to remove CPU stress
		usleep(10000);				

	}

	// Publish terminating status
	module_stat.module_status = 0;
	zcm.publish("STATUS4",&module_stat);
	cout << "autopilot module exiting..." << endl;

	// Stop ZCM
	zcm.stop();

	// Clear up memory
	#ifdef TEST
		delete[] true_state_error;
		
		for(int i = 0; i != 9; ++i)
		{
			delete[] A[i];
			delete[] B[i];
		}
		delete[] A;
		delete[] B;
	#endif
	delete[] input_delta;

	return 0;
}
