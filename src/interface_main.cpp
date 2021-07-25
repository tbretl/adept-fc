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
#include "pwm_t.hpp"
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
        pwm_t pwm_out;
        status_t stat;
        //UDP
        boost::asio::io_service io_service;
        udp::socket socket{io_service};
        udp::endpoint endpoint;

        Handler(const string their_IP)
            : socket(io_service, udp::endpoint(udp::v4(), 0))
        {
            memset(&pwm_out, 0, sizeof(pwm_out));
            memset(&stat, 0, sizeof(stat));
            udp::resolver resolver(io_service);
            udp::resolver::query query(udp::v4(), their_IP, UDP_OUT);
            udp::resolver::iterator iter = resolver.resolve(query);
            endpoint = *iter;
        }

        ~Handler() {
            socket.close();
        }

        void read_pwm(const zcm::ReceiveBuffer* rbuf,const string& chan,const pwm_t *msg)
        {
            int num_outputs = 11; // TODO: Read this from a common file
            pwm_out = *msg;
            std::ostringstream data_out;
            data_out << "!PWM,";
            for (int i=0; i < num_outputs; i++)
            {
                data_out << msg->pwm_out[i] << ",";
            }
            data_out << "PWM!";
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
    Handler handlerObject,pwm_handler;

    Client(const string my_IP,const string their_IP)
        : handlerObject(their_IP), pwm_handler(their_IP)
    {
        //zcm
        memset(&adc_msg, 0, sizeof(adc_msg));
        memset(&vnins_msg, 0, sizeof(vnins_msg));
        memset(&module_stat,0,sizeof(module_stat));

        zcm.subscribe("PWM_OUT",&Handler::read_pwm,&pwm_handler);
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
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " Receive failed: " << error.message() << "\n";
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

    void parse_message(const std::string udp_message, adc_data_t* adc, vnins_data_t* vn200){
        std::string dump;
        std::istringstream ss(udp_message);
        int i = 0;
        std::getline(ss, dump, ',');
        
        if (!strcmp(dump.c_str(), "!VNINS"))
        {
            std::string in_data[21]; // 21 VNINS fields
            
            while(std::getline(ss, dump, ','))
            {
                if (!strcmp(dump.c_str(), "VNINS!"))
                {
                    // fill VN200 structure fields:
                    vn200->time = std::stod(in_data[0], nullptr);
                    vn200->tracking = (bool)std::stoi(in_data[1], nullptr); // assumes 0 or 1
                    vn200->gpsfix = (bool)std::stoi(in_data[2], nullptr); // assumes 0 or 1
                    vn200->error = (bool)std::stoi(in_data[3], nullptr); // assumes 0 or 1
                    vn200->yaw = std::stof(in_data[4], nullptr);
                    vn200->pitch = std::stof(in_data[5], nullptr);
                    vn200->roll = std::stof(in_data[6], nullptr);
                    vn200->wx = std::stof(in_data[7], nullptr);
                    vn200->wy = std::stof(in_data[8], nullptr);
                    vn200->wz = std::stof(in_data[9], nullptr);
                    vn200->latitude = std::stod(in_data[10], nullptr);
                    vn200->longitude = std::stod(in_data[11], nullptr);
                    vn200->altitude = std::stod(in_data[12], nullptr);
                    vn200->vn = std::stof(in_data[13], nullptr);
                    vn200->ve = std::stof(in_data[14], nullptr);
                    vn200->vd = std::stof(in_data[15], nullptr);
                    vn200->ax = std::stof(in_data[16], nullptr);
                    vn200->ay = std::stof(in_data[17], nullptr);
                    vn200->az = std::stof(in_data[18], nullptr);
                    vn200->time_gpspps = std::stol(in_data[19], nullptr); // 64 bit
                    vn200->time_rpi = std::stol(in_data[20], nullptr); // 64 bit
                    return;
                }
                
                in_data[i] = dump;
                i++;

                if ( i > 25 )
                {
                    std::cout << "ERROR No VNINS End Identifier\n";

                    return;
                }
            }
        }
        
		if (!strcmp(dump.c_str(), "!ADC" ))
        {
            std::string in_data[19];  // 19 ADC fields
            
            while(std::getline(ss, dump, ','))
            {
                if (!strcmp(dump.c_str(), "ADC!"))
                {
                    // ADC fill structure fields:
                    for (int i_data = 0; i_data < 16; i_data++)
                    {
                        adc->data[i_data] = std::stoi(in_data[i_data], nullptr); // 32 bit
                    }
                    
                    adc->time_gpspps = std::stol(in_data[16], nullptr); // 64 bit
                    adc->time_rpi = std::stol(in_data[17], nullptr); // 64 bit
                    adc->time_gps = std::stod(in_data[18], nullptr);
                    return;
                }
                
                in_data[i] = dump;
                i++;

                if ( i > 25 )
                {
                    std::cout << "ERROR No ADC End Identifier\n";

                    return;
                }
            }
        }

		// Message error if this point is reached 
		std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
		" ERROR: bad UDP message received [hitl]." << std::endl;
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
