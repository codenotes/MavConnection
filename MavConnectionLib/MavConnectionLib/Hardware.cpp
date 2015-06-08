/*
 * Hardware.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: pneves
 */

#include <stdlib.h>
#include <assert.h>
#include "Hardware.h"

//http://stackoverflow.com/questions/14294267/class-template-for-numeric-types
//template< typename T,
//typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>

/// http://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
int mapValues(float input, float input_start, float input_end,
		float output_start, float output_end) {

	//printf("i %f, istart %f, iend%f\n", input, input_start, input_end);

//	assert(input <= abs(input_end -input_start) && input >= -abs(input_end -input_start));
	int input_range = input_end - input_start;
	int output_range = output_end - output_start;

	return (input - input_start)*output_range / input_range + output_start;
}

namespace Hardware {


/*Hardware physical characteristics */
HardwareData::HardwareData() :
		vehicle_type(MAV_TYPE_QUADROTOR),
		autopilot_type(	MAV_AUTOPILOT_ARDUPILOTMEGA),
		component_id(1),
		system_id(1)
		{
		}

/*Hardware phisical characteristics */

}

