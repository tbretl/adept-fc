//This module is designed to interact with a simulation
//running on a separate computer on the same network with UDP.
//Aaron Perry 5/30/2019
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <zcm/zcm-cpp.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
//message types:
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"
#include "actuators_t.hpp"
#include "status_t.hpp"

#define UDP_PORT 1337
#define UDP_OUT "1338"

using std::string;
using boost::asio::ip::udp;
using boost::asio::ip::address;

//object class to handle incoming messages
class Handler
{
    public:


        //zcm
        actuators_t acts;
        status_t stat;
        //UDP
        boost::asio::io_service io_service;
        udp::socket socket{io_service};
        udp::endpoint endpoint;

        Handler(const string their_IP)
            : socket(io_service, udp::endpoint(udp::v4(), 0))
        {
            memset(&acts, 0, sizeof(acts));
            memset(&stat, 0, sizeof(stat));
            //socket.open(udp::v4());
            //socket.bind(udp::endpoint(address::from_string(their_IP), UDP_PORT));
            udp::resolver resolver(io_service);
            udp::resolver::query query(udp::v4(), their_IP, UDP_OUT);
            udp::resolver::iterator iter = resolver.resolve(query);
            endpoint = *iter;
        }

        ~Handler() {
            socket.close();
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            acts = *msg;
            string out_msg = "working!";
            socket.send_to(boost::asio::buffer(out_msg, out_msg.size()), endpoint);
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

class Client {

public:
    //udp
    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    std::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;
    //zcm
    adc_data_t adc_msg;
    vnins_data_t vnins_msg;
    status_t module_stat;
    zcm::ZCM zcm {"ipc"};
    Handler handlerObject,act_handler;

    Client(const string my_IP,const string their_IP)
        : handlerObject(their_IP), act_handler(their_IP)
    {
        //zcm
        memset(&adc_msg, 0, sizeof(adc_msg));
        memset(&vnins_msg, 0, sizeof(vnins_msg));
        memset(&module_stat,0,sizeof(module_stat));

        zcm.subscribe("ACTUATORS",&Handler::read_acts,&act_handler);
        zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
        module_stat.module_status = 1;//module running
        zcm.start();
        zcm.publish("STATUS3",&module_stat);
        //udp
        socket.open(udp::v4());
        socket.bind(udp::endpoint(address::from_string(my_IP), UDP_PORT));
        start_receive();
        io_service.run();
    }

    ~Client() {
        zcm.stop();
        socket.close();
    }

    void start_receive() {
        socket.async_receive_from(boost::asio::buffer(recv_buffer),
            remote_endpoint,
            boost::bind(&Client::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

            //add a message time_out here
    }

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cout << "Receive failed: " << error.message() << "\n";
            return;
        }
        //std::cout << "Received: '" << std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred) << "' (" << error.message() << ")\n";
        //do the zcm publishing here;
        adc_msg.time_gps = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        vnins_msg.time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        //publish values which came in from simulation
        zcm.publish("ADC_DATA", &adc_msg);
        zcm.publish("VNINS_DATA",&vnins_msg);
        zcm.publish("STATUS3",&module_stat);

        usleep(5000);
        if (!handlerObject.stat.should_exit){
            start_receive();
        }else {
            module_stat.module_status = 0;
            zcm.publish("STATUS3",&module_stat);
        }
    }

};


int main()
{
    //read in config variables
    string rpi_IP = "192.168.0.35";
    string pc_IP = "192.168.0.11";


    std::cout << "hitl started" << std::endl;

    Client udp_zcm_interface(rpi_IP,pc_IP);


    std::cout << "HITL interface module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)
    return 0;
}
