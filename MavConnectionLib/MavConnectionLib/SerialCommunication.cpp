/*
 * SerialCommunication.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: pneves
 */

#include "SerialCommunication.h"

namespace SerialCommunicationNamespace {

	size_t SerialCommunicationNamespace::SerialCommunication::SendMessage(uint8_t* const buffer,
			size_t buffer_length) {
//		size_t result = write(serial_device_file_descriptor, buffer, buffer_length);

//		return result;
		return 0;
	}

	size_t SerialCommunication::ReceiveMessage(uint8_t* const buffer,
			size_t buffer_length) {

//		size_t result = read(serial_device_file_descriptor, buffer, buffer_length);
//		return result;
		return 0;
	}

	SerialCommunication::SerialCommunication(
			string ardupilot_serial_device, BaudRate baud_rate) :
					serial_device(ardupilot_serial_device), serial_device_baud_rate(baud_rate),
					old_terminal_io_setting({}), terminal_io_setting({}), state(UNITIALIZED_INVALID){

//		serial_device_file_descriptor = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		//fcntl(serial_device_file_descriptor, F_SETFL, FNDELAY);

		if (serial_device_file_descriptor < 0)
			state = MODEM_INVALID;
		else {
			if (tcgetattr(serial_device_file_descriptor, &terminal_io_setting) < 0) {
				state = MODEM_INVALID;
				return;
			}
			// Input flags - Turn off input processing
			// convert break to null byte, no CR to NL translation,
			// no NL to CR translation, don't mark parity errors or breaks
			// no input parity check, don't strip high bit off,
			// no XON/XOFF software flow control
			//
			terminal_io_setting.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

			//
			// Output flags - Turn off output processing
			// no CR to NL translation, no NL to CR-NL translation,
			// no NL to CR translation, no column 0 CR suppression,
			// no Ctrl-D suppression, no fill characters, no case mapping,
			// no local output processing
			terminal_io_setting.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
			//
			// No line processing:
			// echo off, echo newline off, canonical mode off,
			// extended input processing off, signal chars off
			//
			terminal_io_setting.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
			//
			// Turn off character processing
			// clear current char size mask, no parity checking,
			// no output processing, force 8 bit input
			//
			terminal_io_setting.c_cflag &= ~(CSIZE | PARENB);
			terminal_io_setting.c_cflag |= CS8;
			//
			// One input byte is enough to return from read()
			// Inter-character timer off
			//
			terminal_io_setting.c_cc[VMIN] = 1;
			terminal_io_setting.c_cc[VTIME] = 10; // was 0

			if (cfsetispeed(&terminal_io_setting, serial_device_baud_rate) < 0 ||
					cfsetospeed(&terminal_io_setting, serial_device_baud_rate) < 0)	{
//				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", serial_device_baud_rate);
				state = MODEM_INVALID;
				return;
			}

			/* commit the serial port settings */
			if (tcsetattr(serial_device_file_descriptor, TCSANOW, &terminal_io_setting) < 0) {
				state = MODEM_INVALID;
				return;
			}

			if (state == UNITIALIZED_INVALID)
				state = VALID;
		}
	}

	SerialCommunication::~SerialCommunication() {
		;//tcsetattr(serial_device_file_descriptor, TCSANOW, &old_terminal_io_setting);
	}

	ICommunication::ComunicationInterfaceState SerialCommunication::getInterfaceState() const {
		return state;
	}
}
