//
// Use of this file is governed by the GPL License - see adept_fc/LICENSE_GPL
//
// It is a modified version of original source taken from:
// https://gist.github.com/kaimallea/e112f5c22fe8ca6dc627
//

//simple client to send test UDP messages

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define UDP_PORT "1337"

using boost::asio::ip::udp;

class UDPClient
{
public:
	UDPClient(
		boost::asio::io_service& io_service,
		const std::string& host,
		const std::string& port
	) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
		udp::resolver resolver(io_service_);
		udp::resolver::query query(udp::v4(), host, port);
		udp::resolver::iterator iter = resolver.resolve(query);
		endpoint_ = *iter;
	}

	~UDPClient()
	{
		socket_.close();
	}

	void send(const std::string& msg) {
		socket_.send_to(boost::asio::buffer(msg, msg.size()), endpoint_);
	}

private:
	boost::asio::io_service& io_service_;
	udp::socket socket_;
	udp::endpoint endpoint_;
};

int main(int argc, char *argv[])
{
	if (argc < 2) {
		std::cout << "missing argument: ip address" << std::endl;
		return 1;
	}
	boost::asio::io_service io_service;
	UDPClient client(io_service, argv[1], UDP_PORT);
	//!,ax,ay,az,u,v,w,V,alpha,beta,lat,lon,alt,p,q,r,phi,theta,psi

while(1){
	client.send("!,0.1,0.2,0.3,10,20,30,60,5,6,5200,4300,100,0.05,0.06,0.07,5,10,15,!");
    usleep(5000);
}

}
