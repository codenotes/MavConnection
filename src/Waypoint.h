/*
 * Waypoint.h
 *
 *  Created on: Apr 24, 2014
 *      Author: pneves
 */

#ifndef WAYPOINT_H_
#define WAYPOINT_H_

/*
Taken and modified from QGroundControl Waypoint.h file.
QGroundControl license 
*/
#include "Hardware.h"

#define LATITUDE_TO_MILLIMETERS (11.13195f)
#define DEG_TO_RAD (0.017453292519943295769236907684886f)
#define RAD_TO_DEG (57.295779513082320876798154814105f)
class Waypoint {
public:

	/*
	 * 1.113195 centi degrees lat to cm comes from ardupilot code
	 * AP_MAth.h:57
	 */
	static double convertLatitudeToMillimeter(const double& new_latitude) {
		return 1E7 *LATITUDE_TO_MILLIMETERS * new_latitude;
	}

	/*
	 * location.cpp:46
	 */
	static double convertLongitudeToMillimeter(const double& new_longitude,
			const double& new_latitude) {
		return 1E7 * LATITUDE_TO_MILLIMETERS * new_longitude * cos(new_latitude * DEG_TO_RAD);
	}


	//Changing the order of the constructor changes the emplace order in MavCommunication.cpp, becareful!
    Waypoint(uint16_t new_id = 0,
    		float new_latitude = 0,
    		float new_longitude = 0,
    		float new_z = 0,
    		bool new_autocontinue = false,
    		bool new_current = false,
    		float new_param4 = 0,
    		MAV_FRAME new_frame=MAV_FRAME_GLOBAL,
    		float new_param1 = 0,
    		float new_param2 = 0) :
             _id(new_id),
             longitude(new_longitude),
             latitude(new_latitude),
             altitude(new_z),
             autocontinue(new_autocontinue),
             current(new_current),
             yaw(new_param4),
             frame(new_frame),
			 param1(new_param1),
			 param2(new_param2),
			 reached(false) {}

    uint16_t getId() const {
        return _id;
    }
    double getX() const {
        return Waypoint::convertLongitudeToMillimeter(longitude, latitude);
    }
    double getY() const {
        return Waypoint::convertLatitudeToMillimeter(latitude);
    }
    float getZ() const {
        return altitude;
    }
    float getLatitude() const {
        return latitude;
    }
    float getLongitude() const {
        return longitude;
    }
    float getAltitude() const {
        return altitude;
    }
    float getYaw() const {
        return yaw;
    }
    bool getAutoContinue() const {
        return autocontinue;
    }
    bool getCurrent() const {
        return current;
    }
    float getAcceptanceRadius() const {
        return param2;
    }
    float getHoldTime() const {
        return param1;
    }
    float getParam1() const {
        return param1;
    }
    float getParam2() const {
        return param2;
    }
//    float getParam3() const {
//        return _orbit;
//    }
    float getParam4() const {
        return yaw;
    }
    float getParam5() const {
        return longitude;
    }
    float getParam6() const {
        return latitude;
    }
    float getParam7() const {
        return altitude;
    }
    MAV_FRAME getFrame() const {
        return frame;
    }

    /* TODO: Check for a better function. Maybe one day station input is verified on input not
     * in use.
     */
    uint32_t calculateWaypointDistance(const Waypoint& other_waypoint) const {
    	return (unsigned int)sqrt(pow(getX() - other_waypoint.getX(), 2) +
    			pow(getY() - other_waypoint.getY(), 2) +
    	    			pow(altitude - other_waypoint.getZ(), 2));
    }

    double calculateDistanceToPosition(
    		const Hardware::FlightParameters& current_position) const {
    	printf("%f, %f, %f - %f, %f, %f\n", getY(),getX(), getZ() * 1000,
    			current_position.y, current_position.x, current_position.z);
		return sqrt(pow(getX() - current_position.x, 2) +
				pow(getY() - current_position.y, 2) +
				pow(getAltitude() *1000 - current_position.z, 2));
	}


private:
    uint16_t _id;
    float longitude;
    float latitude;
    float altitude;
    bool autocontinue;
    bool current;
    float yaw;
    MAV_FRAME frame;
    float param1;
    float param2;
    bool reached;


};
/** @brief Returns true if same position. We ignore other stuff
 * TODO Correct this to fabs*/
    inline bool operator==(const Waypoint& lhs, const Waypoint& rhs){
    	return (lhs.getLongitude() == rhs.getLongitude() &&
    			lhs.getLatitude() == rhs.getLatitude() &&
    			lhs.getZ() == rhs.getZ()) ? true : false;
    }

#endif /* WAYPOINT_H_ */
