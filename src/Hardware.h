#ifndef HARDWARE_H
#define HARDWARE_H


#include "mavlink.h" //"mavlink/common/mavlink.h"

#define DO_PRAGMA(x) _Pragma (#x)
#define TODO(x) DO_PRAGMA(message ("TODO - " #x))

#define HOME_ALTITUDE_MM 600
#define HOME_LAT -35.362938
#define HOME_LONG 149.165085
#define HOME_YAW 0.0
#define RESOLUTION_X 640
#define RESOLUTION_Y 640
#define FIELD_OF_VIEW 1.2566370614359172

//http://stackoverflow.com/questions/14294267/class-template-for-numeric-types
//template< typename T,
//typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>

/// http://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
int mapValues(float input, float input_start, float input_end,
		float output_start, float output_end);


namespace Hardware {


	struct FlightParameters {
		float roll; //radians
		float pitch; //radians
		float yaw; //radians
		uint16_t throttle; //0-100%
		float x; //mm
		float y; //mm
		float z; //mm
		uint32_t time_from_boot;
	};


	class HardwareData {
	public:
		/*Hardware physical characteristics */
		HardwareData();
		const MAV_TYPE vehicle_type;
		const MAV_AUTOPILOT autopilot_type;
		const uint8_t component_id;
		const uint8_t system_id;

	
	};
}

#endif
