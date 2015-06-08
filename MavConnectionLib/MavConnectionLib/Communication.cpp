#include "communication.h"

#include <iostream>

using namespace SerialCommunicationNamespace;

#if 0
class client
{
public:
	client(const udp::endpoint& listen_endpoint)
		: socket_(io_service_, listen_endpoint),
		deadline_(io_service_)
	{
		// No deadline is required until the first socket operation is started. We
		// set the deadline to positive infinity so that the actor takes no action
		// until a specific deadline is set.
		deadline_.expires_at(boost::posix_time::pos_infin);

		// Start the persistent actor that checks for deadline expiry.
		check_deadline();
	}

	std::size_t receive(const boost::asio::mutable_buffer& buffer,
		boost::posix_time::time_duration timeout, boost::system::error_code& ec)
	{
		// Set a deadline for the asynchronous operation.
		deadline_.expires_from_now(timeout);

		// Set up the variables that receive the result of the asynchronous
		// operation. The error code is set to would_block to signal that the
		// operation is incomplete. Asio guarantees that its asynchronous
		// operations will never fail with would_block, so any other value in
		// ec indicates completion.
		ec = boost::asio::error::would_block;
		std::size_t length = 0;

		// Start the asynchronous operation itself. The handle_receive function
		// used as a callback will update the ec and length variables.
		socket_.async_receive(boost::asio::buffer(buffer),
			boost::bind(&client::handle_receive, _1, _2, &ec, &length));

		// Block until the asynchronous operation has completed.
		do io_service_.run_one(); while (ec == boost::asio::error::would_block);

		return length;
	}

private:
	void check_deadline()
	{
		// Check whether the deadline has passed. We compare the deadline against
		// the current time since a new asynchronous operation may have moved the
		// deadline before this actor had a chance to run.
		if (deadline_.expires_at() <= deadline_timer::traits_type::now())
		{
			// The deadline has passed. The outstanding asynchronous operation needs
			// to be cancelled so that the blocked receive() function will return.
			//
			// Please note that cancel() has portability issues on some versions of
			// Microsoft Windows, and it may be necessary to use close() instead.
			// Consult the documentation for cancel() for further information.
			socket_.cancel();

			// There is no longer an active deadline. The expiry is set to positive
			// infinity so that the actor takes no action until a new deadline is set.
			deadline_.expires_at(boost::posix_time::pos_infin);
		}

		// Put the actor back to sleep.
		deadline_.async_wait(boost::bind(&client::check_deadline, this));
	}

	static void handle_receive(
		const boost::system::error_code& ec, std::size_t length,
		boost::system::error_code* out_ec, std::size_t* out_length)
	{
		*out_ec = ec;
		*out_length = length;
	}

private:
	boost::asio::io_service io_service_;
	udp::socket socket_;
	deadline_timer deadline_;
};

#endif








size_t SerialCommunication::SendMessage(uint8_t* const buffer, size_t buffer_length) 
{
	
	auto bf = boost::asio::buffer(buffer, buffer_length);

	size_t sent=socket->send_to(bf, local_endpoint);

	return sent; 
}

size_t SerialCommunication::ReceiveMessage(uint8_t* const buffer, size_t buffer_length) 
{
	// boost::asio::read(stream, boost::asio::buffer(data, size));
	//async_receive
	boost::array<char, 512> recv_buf;
	size_t len = socket->receive_from(	boost::asio::buffer(recv_buf), local_endpoint);

	
	printf("SIZE:%d, INLEN:%d\t", len, buffer_length);


	for (int i=0; i < len;i++)
	{

		buffer[i] = recv_buf[i];
		printf("%.2x ", buffer[i]);

	}

	printf("\n");

//	size_t len = socket->receive(boost::asio::buffer(recv_buf),local_endpoint);
	//std::cout.write(recv_buf.data(), len);

	return len;

}


SerialCommunication::ComunicationInterfaceState SerialCommunication::getInterfaceState() const
{
	return this->state; 
}

SerialCommunication::SerialCommunication(std::string device_or_ip , int baud_or_port)
{

	try
	{
		/*if (argc != 3)
		{
		std::cerr << "Usage: client <host> <port>" << std::endl;
		return 1;
		}
		*/


		std::string ip = device_or_ip, port = boost::lexical_cast<std::string>(baud_or_port);

		local_endpoint = boost::asio::ip::udp::endpoint(
			//boost::asio::ip::address::from_string(argv[1]), boost::lexical_cast<int>(argv[2]));
			boost::asio::ip::address::from_string(ip), boost::lexical_cast<int>(port));

		std::cout << "Local bind " << local_endpoint << std::endl;

		// MODE 1: WORKS  
		//    udp::socket socket(io_service, local_endpoint);

		// MODE 2: WORKS
		//    udp::socket socket(io_service, udp::endpoint(udp::v4(), boost::lexical_cast<int>(argv[2]) )),

		// MODE 3: WORKS
		socket = new udp::socket(io_service);
		socket->open(udp::v4());
		socket->bind(local_endpoint);

		
		///////////////
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}


		
	
	state = VALID;
}

SerialCommunication::~SerialCommunication() 
{



}


