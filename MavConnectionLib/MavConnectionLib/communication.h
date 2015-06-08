#pragma once
#include "ICommunication.h"
#include <string>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

using namespace::std;
using boost::asio::ip::udp;

#undef SendMessage
namespace SerialCommunicationNamespace
{

	class SerialCommunication : public ICommunication {
	private:
		const std::string serial_device;
		int serial_device_file_descriptor;
		int serial_device_baud_rate;
		udp::endpoint local_endpoint;
		boost::asio::io_service io_service;
	//	udp::endpoint sender_endpoint;
		udp::socket * socket;
		ComunicationInterfaceState state;
	public:
		size_t SendMessage(uint8_t* const buffer, size_t buffer_length);
		size_t ReceiveMessage(uint8_t* const buffer, size_t buffer_length);
		ComunicationInterfaceState getInterfaceState() const;
		
		SerialCommunication(std::string device_or_ip = "10.1.55.232", int baud_or_port=14550);
		
		~SerialCommunication();


	};
}