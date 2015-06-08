/*
 * SerialCommunication.h
 *
 *  Created on: Mar 27, 2014
 *      Author: pneves
 */

#ifndef SERIALCOMMUNICATION_H_
#define SERIALCOMMUNICATION_H_

#include <termios.h>
#include <fcntl.h>	// File control
#include <unistd.h>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <process.h>


#include "ICommunication.h"

using namespace std;

namespace SerialCommunicationNamespace {
	enum BaudRate {
		BAUD50 = B50,
		BAUD75 = B75,
		BAUD110 = B110,
		BAUD134 = B134,
		BAUD150 = B150,
		BAUD200 = B200,
		BAUD300 = B300,
		BAUD600 = B600,
		BAUD1200 = B1200,
		BAUD1800 = B1800,
		BAUD2400 = B2400,
		BAUD4800 = B4800,
		BAUD9600 = B9600,
		BAUD19200 = B19200,
		BAUD38400 = B38400,
		BAUD57600 = B57600,
		BAUD115200 = B115200,
		BAUD230400 = B230400,
		BAUD460800 = B460800,
		BAUD500000 = B500000,
		BAUD576000 = B576000,
		BAUD921600 = B921600,
		BAUD1000000 = B1000000,
		BAUD1152000 = B1152000,
		BAUD1500000 = B1500000,
		BAUD2000000 = B2000000,
		BAUD2500000 = B2500000,
		BAUD3000000 = B3000000,
		BAUD3500000 = B3500000,
		BAUD4000000 = B4000000
	};

	//http://www.cmrr.umn.edu/~strupp/serial.html#3_1
	class SerialCommunication : public ICommunication {
	private:
		const string serial_device;
		int serial_device_file_descriptor;
		BaudRate serial_device_baud_rate;
		termios old_terminal_io_setting;
		termios terminal_io_setting;
		ComunicationInterfaceState state;
	public:
		size_t SendMessage(uint8_t* const buffer, size_t buffer_length);
		size_t ReceiveMessage(uint8_t* const buffer, size_t buffer_length);
		ComunicationInterfaceState getInterfaceState() const;
		SerialCommunication(string ardupilot_serial_device = "/dev/ttyACM0", BaudRate baud_rate = BAUD115200);
		~SerialCommunication();
	};
}

#endif /* SERIALCOMMUNICATION_H_ */
