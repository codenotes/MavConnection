/*
 * MavCommunicationTest.cpp
 *
 *  Created on: Apr 23, 2014
 *      Author: pneves
 */

#ifndef MAVCOMMUNICATIONTEST_H_
#define MAVCOMMUNICATIONTEST_H_

#include "gtest/gtest.h"
#include "ProcessSpawner.h"
#include "../src/MavCommunication.h"
#include "../src/Waypoint.h"
#include "../src/Hardware.h"
#include <stdio.h>

using namespace Mav;
namespace {
	class MavCommunicationTest : public ::testing::Test {

	};

	TEST_F(MavCommunicationTest, PUBLICCLASSTEST) {
		ProcessSpawner * const process_spawner_pointer = new ProcessSpawner();
		EXPECT_TRUE(ProcessSpawner::process_spawner_pointer != nullptr);
		ProcessSpawner::process_spawner_pointer->spawnProcess(ProcessSpawner::ARDUPILOT);
		sleep(1);
		ProcessSpawner::process_spawner_pointer->spawnProcess(ProcessSpawner::SITL); //SILT must be after Ardupilot
		sleep(1);
		ProcessSpawner::process_spawner_pointer->spawnProcess(ProcessSpawner::SERIAL_EMULATOR);
		Hardware::HardwareData hardware{};
		MavCommunication *communication_interface = new MavCommunication(hardware);
		EXPECT_TRUE(communication_interface->isMavConnectionValid());
		sleep(5);
		FlightParameters parameters = {};
		communication_interface->getFlightParameters(parameters);
		EXPECT_TRUE(parameters.time_from_boot > 0);
		EXPECT_TRUE(communication_interface->isMavConnectionValid());


		vector<Waypoint> waypoints_to_send{};
		waypoints_to_send.emplace_back(0, 0, 0, 0.0, false); //The first waypoint is the home and will be overwritten by the APM itself
		waypoints_to_send.emplace_back(1, 47, 0.5, 20.0, false, true);
		waypoints_to_send.emplace_back(2, 47, 0.9, 30.0, false, true);
		waypoints_to_send.emplace_back(3, 47, 0.9, 40.0, false);
		printf("x:%f y:%f, z:%f\n", waypoints_to_send[1].getLongitude(), waypoints_to_send[1].getLatitude(), waypoints_to_send[1].getZ());
		EXPECT_TRUE(waypoints_to_send[1].getLatitude() == 47);
		communication_interface->setupRC();
		float parameter = 0.0;

		if (parameter != 1.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("ARMING_CHECK", 1.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("FRAME", parameter));
		if (parameter != 0.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("FRAME", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("MAG_ENABLE", parameter));
		if (parameter != 1.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("MAG_ENABLE", 1.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("FS_THR_ENABLE", parameter));
		if (parameter != 1.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("FS_THR_ENABLE", 1.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("COMPASS_LEARN", parameter));
		if (parameter != 0.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("COMPASS_LEARN", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("CH7_OPT", parameter));
		if (parameter != 7.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("CH7_OPT", 7.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("BATT_MONITOR", parameter));
		if (parameter != 4.0)
			EXPECT_TRUE(communication_interface->setEEPROMParameter("BATT_MONITOR", 4.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("COMPASS_OFS_X", parameter));
		if (parameter != 5.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("COMPASS_OFS_X", 5.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("COMPASS_OFS_Y", parameter));
		if (parameter != 13.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("COMPASS_OFS_Y", 13.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("COMPASS_OFS_Z", parameter));
		if (parameter != -18.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("COMPASS_OFS_Z", -18.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC1_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC1_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC1_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC1_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC1_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC1_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC2_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC2_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC2_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC2_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC2_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC2_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC3_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC3_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC3_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC3_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC3_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC3_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC4_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC4_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC4_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC4_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC4_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC4_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC5_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC5_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC5_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC5_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC5_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC5_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC6_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC6_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC6_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC6_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC6_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC6_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC7_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC7_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC7_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC7_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC7_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC7_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC8_MAX", parameter));
		if (parameter != 2000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC8_MAX", 2000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC8_MIN", parameter));
		if (parameter != 1000.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC8_MIN", 1000.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("RC8_TRIM", parameter));
		if (parameter != 1500.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("RC8_TRIM", 1500.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_GPS_DELAY", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_GPS_DELAY", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_ACC_RND", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_ACC_RND", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_GYR_RND", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_GYR_RND", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_WIND_SPD", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_WIND_SPD", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_WIND_TURB", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_WIND_TURB", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_BARO_RND", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_BARO_RND", 0.0));

		EXPECT_TRUE(communication_interface->getEEPROMParameter("SIM_MAG_RND", parameter));
		if (parameter != 0.0)
		EXPECT_TRUE(communication_interface->setEEPROMParameter("SIM_MAG_RND", 0.0));

//		EXPECT_FALSE(communication_interface->getEEPROMParameter("INVALID_STRIN", parameter));

		printf("Connection Valid, Boot time:%d\n", parameters.time_from_boot);
		sleep(2);
		EXPECT_TRUE(communication_interface->setMotorsArmed(false));
		sleep(1);


		EXPECT_TRUE(communication_interface->sendNewWaypointList(waypoints_to_send));
		sleep(2);
		vector<Waypoint> waypoints = communication_interface->getWaypointList();
		size_t counter = 0;
		for (auto waypoint : waypoints) {
			if (counter == 0) { //home can be anything and is ignored by what we set
				counter++;
				continue;
			}
			printf("Id %d: Id %d\n", waypoint.getId(), waypoints_to_send[counter].getId());
			EXPECT_TRUE(waypoint.getId() == waypoints_to_send[counter].getId());

			printf("x:%f y:%f, z:%f\n", waypoint.getLongitude(), waypoint.getLatitude(), waypoint.getZ());
			printf("x:%f y:%f, z:%f\n", waypoints_to_send[counter].getLongitude(), waypoints_to_send[counter].getLatitude(), waypoints_to_send[counter].getZ());
			EXPECT_TRUE(waypoint.getLongitude() == waypoints_to_send[counter].getLongitude());
			EXPECT_TRUE(waypoint.getLatitude() == waypoints_to_send[counter].getLatitude());
			EXPECT_TRUE(waypoint.getZ() == waypoints_to_send[counter].getZ());

			//autocontinue is ignored in APM GCS_Mavlink.pde and defaults to false
			printf("auto-continue sent %d received %d\n", waypoints_to_send[counter].getAutoContinue(),
					waypoint.getAutoContinue());
			EXPECT_TRUE(waypoint.getAutoContinue() == true);

			//When a flight plan is parsed in APM the current parameter is not read, so commander
			//has to explicitly set the current waypoint like below. Also found in GCS_Mavlink.pde
			printf("current sent %d received %d\n", waypoints_to_send[counter].getCurrent(), waypoint.getCurrent());
			EXPECT_TRUE(waypoint.getCurrent() == false/*waypoints_to_send[counter].getCurrent()*/); //


			counter++;

		}
		printf("Waypoint Sent count %lu, Received Count %lu\n", waypoints_to_send.size(), waypoints.size());
		EXPECT_TRUE(waypoints.size() == waypoints_to_send.size());
		EXPECT_TRUE(communication_interface->isWaypointOnMAV(waypoints_to_send[1], false));

		EXPECT_TRUE(communication_interface->setCurrentWaypoint(2));
		sleep(1);
		EXPECT_TRUE(communication_interface->setCurrentWaypoint(1));
		sleep(1); //sleep because the datastream is refreshed not so often

		EXPECT_TRUE(communication_interface->requestLevelCalibration());
		sleep(3);
		EXPECT_TRUE(communication_interface->setStabilizeMode());
		sleep(1);

		EXPECT_TRUE(communication_interface->setMotorsArmed(true));// Lots of problems
		communication_interface->setupRC();

		sleep(2);
		EXPECT_TRUE(communication_interface->isInStabilizeMode());
		sleep(1);
		communication_interface->setRCChannels(true, 1700, 0, 0, 0, 0);
		FlightParameters parameters_new  = {};
		do {
			communication_interface->getFlightParameters(parameters_new);

		} while(parameters_new.z < 1000);

		delete communication_interface;
		ProcessSpawner::process_spawner_pointer->killProcesses();
		delete process_spawner_pointer;
	}
}
#endif
