//Aaron Perry, 5/30/19
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define IPADDRESS "192.168.0.35"
#define UDP_PORT 1337

using boost::asio::ip::udp;
using boost::asio::ip::address;

class Client {

public:

    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    std::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;



    Client()
    {
        socket.open(udp::v4());
        socket.bind(udp::endpoint(address::from_string(IPADDRESS), UDP_PORT));
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


int main() {

    Client receiver;
    return 0;
}
