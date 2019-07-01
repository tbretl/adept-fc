//
// Use of this file is governed by the GPL License - see adept_fc/LICENSE_GPL
//
// It is a modified version of original source taken from:
// https://gist.github.com/kaimallea/e112f5c22fe8ca6dc627
//

#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//#define IPADDRESS "192.168.0.11"
#define UDP_PORT 1338

using boost::asio::ip::udp;
using boost::asio::ip::address;

class Client {

public:

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    std::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;



    Client(const std::string my_IP)
    {
        std::cout << my_IP << std::endl;
        socket.open(udp::v4());
        socket.bind(udp::endpoint(address::from_string(my_IP), UDP_PORT));
        start_receive();
        io_service.run();
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

        std::cout << "Received: '" << std::string(recv_buffer.begin(), recv_buffer.begin()+bytes_transferred) << "' (" << error.message() << ")\n";

        start_receive();
    }



};


int main(int argc, char *argv[]) {

	if (argc < 2) {
		std::cout << "missing argument: ip address" << std::endl;
		return 1;
	}

    Client receiver(argv[1]);
    return 0;
}
