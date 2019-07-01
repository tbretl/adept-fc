//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <zcm/zcm-cpp.hpp>
#include "vnins_data_t.hpp"
#include "adc_data_t.hpp"

class VNINSHandler {
    public:
        ~VNINSHandler() {}

        vnins_data_t data = {};
	int count = 0;

        void read_messages(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const vnins_data_t *msg) {
            data = *msg;
            this->count++;
        }

        bool tracking() {
	    return (data.tracking == 1);
	}

	bool gpsfix() {
	    return (data.gpsfix == 1);
	}

	bool error() {
	    return (data.error == 1);
	}

        void print() {
	    std::cout << "messages received: " << this->count << std::endl;
	    std::cout << "most recent message: " << std::endl;
	    std::cout << " week: " << data.week << ", time: " << std::setprecision(14) << data.time << std::setprecision(6) << std::endl;
	    std::cout << " tracking: " << this->tracking();
	    std::cout << ", gpsfix: " << this->gpsfix();
	    std::cout << ", error: " << this->error() << std::endl;
	    std::cout << " (yaw, pitch, roll) = (" << data.yaw << ", " << data.pitch << ", " << data.roll << ")" << std::endl;
	    std::cout << " (lat, lon, alt) = (" << data.latitude << ", " << data.longitude << ", " << data.altitude << ")" << std::endl;
	    std::cout << " (vx, vy, vz) = (" << data.vx << ", " << data.vy << ", " << data.vz << ")" << std::endl;
	    std::cout << " (au, pu, vu) = (" << data.attuncertainty << ", " << data.posuncertainty << ", " << data.veluncertainty << ")" << std::endl;
	    std::cout << " time_gpspps (microseconds): " << data.time_gpspps << std::endl;
	    this->count = 0;
	}
};

class ADCHandler {
    public:
        ~ADCHandler() {}

        adc_data_t data = {};
	int count = 0;

        void read_messages(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const adc_data_t *msg) {
            data = *msg;
            this->count++;
        }

        void print(double vnins_gps_time, long long vnins_time_gpspps) {
	    double lastppstime = std::floor(vnins_gps_time);
	    double adc_time_gpspps = ((double) data.time_gpspps) / 1000000;
	    double adc_gps_time = 0;
	    if (std::abs(vnins_time_gpspps - data.time_gpspps) < 500000) {
		adc_gps_time = lastppstime + adc_time_gpspps;
	    } else if (std::abs(vnins_time_gpspps - data.time_gpspps) < 1500000) {
		adc_gps_time = lastppstime + adc_time_gpspps - 1;
	    } else {
		std::cout << "WARNING: error in computing adc_gps_time" << std::endl;
	    }

	    std::cout << "messages received: " << this->count << std::endl;
	    std::cout << "most recent message: " << std::endl;
	    std::cout << " time (computed from VNINS data): " << std::setprecision(14) << adc_gps_time << std::setprecision(6) << std::endl;
	    std::cout << " time (computed from VNINS data in ADC module): " << std::setprecision(14) << data.time_gps << std::setprecision(6) << std::endl;
	    std::cout << " time_gpspps (microseconds): " << data.time_gpspps << std::endl;
	    for (int i=0; i<16; ++i) {
		std::cout << "  data[" << i << "] = " << data.data[i] << std::endl;
	    }
	    this->count = 0;
	}
};

int main(int argc, char *argv[]) {
    // initialize zcm
    zcm::ZCM zcm {"ipc"};

    // subscribe to incoming channels:
    VNINSHandler vninsHandler;
    zcm.subscribe("VNINS_DATA", &VNINSHandler::read_messages, &vninsHandler);
    ADCHandler adcHandler;
    zcm.subscribe("ADC_DATA", &ADCHandler::read_messages, &adcHandler);

    // start zcm as a separate thread:
    zcm.start();

    //control loop:
    while (1) {

        usleep(1000000);
        vninsHandler.print();
	adcHandler.print(vninsHandler.data.time, vninsHandler.data.time_gpspps);

    }

	// stop zcm
    zcm.stop();

	// exit with no error
    return 0;
}
