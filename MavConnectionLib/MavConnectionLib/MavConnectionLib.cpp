// MavConnectionLib.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "MavCommunication.h"
#include "Hardware.h"
#include <iostream>
//#include <boost/array.hpp>
//#include <boost/asio.hpp>
//#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
//using boost::asio::ip::udp;

#pragma comment(lib, R"(C:\Boost\lib\libboost_system-vc140-mt-gd-1_58.lib)")
#pragma comment(lib, R"(C:\Boost\lib\libboost_thread-vc140-mt-gd-1_58.lib)")

//int main2()
//{
//	try
//	{
//		/*if (argc != 3)
//		{
//			std::cerr << "Usage: client <host> <port>" << std::endl;
//			return 1;
//		}
//*/
//		boost::asio::io_service io_service;
//		
//		std::string ip="127.0.0.1", port="14550";
//		
//		udp::endpoint local_endpoint = boost::asio::ip::udp::endpoint(
//			//boost::asio::ip::address::from_string(argv[1]), boost::lexical_cast<int>(argv[2]));
//			boost::asio::ip::address::from_string(ip), boost::lexical_cast<int>(port));
//		
//		std::cout << "Local bind " << local_endpoint << std::endl;
//
//		// MODE 1: WORKS  
//		//    udp::socket socket(io_service, local_endpoint);
//
//		// MODE 2: WORKS
//		//    udp::socket socket(io_service, udp::endpoint(udp::v4(), boost::lexical_cast<int>(argv[2]) )),
//
//		// MODE 3: WORKS
//		udp::socket socket(io_service);
//		socket.open(udp::v4());
//		socket.bind(local_endpoint);
//		///////////////
//
//		boost::array<char, 128> recv_buf;
//
//		udp::endpoint sender_endpoint;
//		size_t len = socket.receive_from(
//			boost::asio::buffer(recv_buf), sender_endpoint);
//
//		std::cout.write(recv_buf.data(), len);
//	}
//	catch (std::exception& e)
//	{
//		std::cerr << e.what() << std::endl;
//	}
//
//	return 0;
//}
//






#define WIN32_LEAN_AND_MEAN

#include "windows.h"

using namespace Mav;

int _tmain(int argc, _TCHAR* argv[])
{


	

	printf("my thread is %d\n, for main", boost::this_thread::get_id());
//	main2();
	//return 0;
	FlightParameters parameters = {};
	Hardware::HardwareData hardware{};
	MavCommunication communication_interface(hardware);


	


	bool b = communication_interface.isMavConnectionValid();
	Sleep(5);

	communication_interface.getFlightParameters(parameters);


	while (1)
	{
		printf("boop\n");
		Sleep(1);

	}


	return 0;
}

