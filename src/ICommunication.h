#ifndef ICOMMUNICATION_H_
#define ICOMMUNICATION_H_

//#include "opencv2/opencv.hpp"
#include <stdint.h>
class ICommunication {
public:
	enum ComunicationInterfaceState {
		UNITIALIZED_INVALID,
		MODEM_INVALID,
		VALID
	};
	virtual ~ICommunication() {}
	virtual size_t SendMessage(uint8_t* const buffer, size_t buffer_length) = 0;
	virtual size_t ReceiveMessage(uint8_t* const buffer, size_t buffer_length) = 0;
	virtual ComunicationInterfaceState getInterfaceState() const = 0;

private:
	ICommunication& operator=(const ICommunication&);
};


#endif /* ICOMMUNICATION_H_ */
