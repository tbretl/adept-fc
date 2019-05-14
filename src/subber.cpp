#include <unistd.h>
#include <string.h>
#include <iostream>
#include <zcm/zcm-cpp.hpp>
#include "vnins_data_t.hpp"

using namespace std;
using std::string;

int count = 0;

class Handler
{
    public:
        ~Handler() {}

        vnins_data_t data = {};

        void read_messages(const zcm::ReceiveBuffer* rbuf, const string& chan, const vnins_data_t *msg) {
            data = *msg;
            count++;
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
			cout << " week: " << data.week << ", time: " << data.time << endl;
			cout << " tracking: " << this->tracking();
			cout << ", gpsfix: " << this->gpsfix();
			cout << ", error: " << this->error() << endl;
			cout << " (yaw, pitch, roll) = (" << data.yaw << ", " << data.pitch << ", " << data.roll << ")" << endl;
			cout << " (lat, lon, alt) = (" << data.latitude << ", " << data.longitude << ", " << data.altitude << ")" << endl;
			cout << " (vx, vy, vz) = (" << data.vx << ", " << data.vy << ", " << data.vz << ")" << endl;
			cout << " (au, pu, vu) = (" << data.attuncertainty << ", " << data.posuncertainty << ", " << data.veluncertainty << ")" << endl;
			cout << " time_gpspps (microseconds): " << data.time_gpspps << endl;
		}
};

int main(int argc, char *argv[]) {
    // initialize zcm
    zcm::ZCM zcm {"ipc"};

    // subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("VNINS_DATA", &Handler::read_messages, &handlerObject);

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
