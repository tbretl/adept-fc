//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <unistd.h>
#include <string.h>
#include <iostream>
#include <zcm/zcm-cpp.hpp>
#include "adc_data_t.hpp"

using namespace std;
using std::string;

int count = 0;

class Handler
{
    public:
        ~Handler() {}

        adc_data_t data = {};

        void read_messages(const zcm::ReceiveBuffer* rbuf, const string& chan, const adc_data_t *msg) {
            data = *msg;
            count++;
        }

        void print() {
			cout << " time_gpspps (microseconds): " << data.time_gpspps << endl;
			for (int i=0; i<16; ++i) {
				cout << "  data[" << i << "] = " << data.data[i] << endl;
			}
		}
};

int main(int argc, char *argv[]) {
    // initialize zcm
    zcm::ZCM zcm {"ipc"};

    // subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("ADC_DATA", &Handler::read_messages, &handlerObject);

    // start zcm as a separate thread:
    zcm.start();

    //control loop:
    while (1) {

        usleep(1000000);
        cout << "messages received in 1 second: " << count << endl;
        cout << "most recent message received: " << endl;
        handlerObject.print();
        count = 0;

    }

	// stop zcm
    zcm.stop();

	// exit with no error
    return 0;
}
