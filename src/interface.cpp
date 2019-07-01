//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

//This module is designed to interact with a simulation
//running on a separate computer on the same network with UDP.

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
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

//object class to handle ZCM messages
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
            std::ostringstream data_out;
            data_out << "!," << msg->da << "," << msg->de << "," << msg->dr << ",";
            for (int i=0;i<8;i++)
            {
                data_out << msg->dt[i] << ",";
            }
            data_out << "!";
            string out_msg = data_out.str();
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
    }

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cout << "Receive failed: " << error.message() << "\n";
            return;
        }
        std::string incoming_msg = std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred);
        //parse the message:
        parse_message(incoming_msg,&adc_msg,&vnins_msg);
        //time stamp:
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

    void parse_message(const std::string udp_message,adc_data_t* adc, vnins_data_t* vn200){
        std::string dump;
        std::string in_data[20];
        std::istringstream ss(udp_message);
        int i = 0;
        while(std::getline(ss, dump, ',')) {
            in_data[i] = dump;
            i++;
        }
        if (!in_data[0].compare("!") && !in_data[19].compare("!")){
            //fill structure fields:
            vn200->roll = (float) std::stod(in_data[16],nullptr);
            vn200->pitch = (float) std::stod(in_data[17],nullptr);
            vn200->yaw = (float) std::stod(in_data[18],nullptr);
            vn200->latitude = std::stod(in_data[10],nullptr);
            vn200->longitude = std::stod(in_data[11],nullptr);
            vn200->altitude = std::stod(in_data[12],nullptr);
            vn200->vx = (float) std::stod(in_data[4],nullptr);
            vn200->vy = (float) std::stod(in_data[5],nullptr);
            vn200->vz = (float) std::stod(in_data[6],nullptr);
        } else {
            std::cout << "ERROR: bad UDP message received [hitl]." << std::endl;
        }
        return;
    }

};


int main()
{
    //read in config variables
    string rpi_IP;
    string pc_IP;
    string dump;
    std::ifstream config_stream;

    config_stream.open("config_files/network_params.config");
    config_stream >> dump >> rpi_IP;
    config_stream >> dump >> pc_IP;
    config_stream.close();

    std::cout << "hitl started" << std::endl;

    Client udp_zcm_interface(rpi_IP,pc_IP);

    std::cout << "hitl module exiting..." << std::endl;
    return 0;
}
