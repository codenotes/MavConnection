/*
 * ProcessSpawner.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: pneves
 */

#include "ProcessSpawner.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "../src/Hardware.h"
#include <stdexcept>

//http://www.guyrutenberg.com/2008/12/20/expanding-macros-into-string-constants-in-c/
#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)
#define HOMESTR "--home=" STR(HOME_LAT) "," STR(HOME_LONG) "," STR(HOME_ALTITUDE_MM) "," STR(HOME_YAW)
#define FOVSTR "--fov=" STR(FIELD_OF_VIEW)
#define RESOLUTIONSTR "--resolution=" STR(RESOLUTION_X) "," STR(RESOLUTION_Y)

const char *COMMANDS_PLATFORM_PICTURE[] = {"gst-launch-1.0", "filesrc", "location=assets/g2997.png",
		"!", "pngdec", "!","videoconvert", "!", "imagefreeze", "!", "videoscale", "!",
		"video/x-raw,width=640", "!" , "v4l2sink", "device=/dev/video0",
		(const char*)NULL
};


const char *COMMANDS_ARDUPILOT[] = {
		"Simulator/ardupilot/tmp/ArduCopter.build/ArduCopter.elf", (const char*)NULL
};

/*
 * To not require super user permissions the /dev directory must have rw permissions from the user
 * executing the below command. This because to create /dev/ttyACM0 you need permissions on /dev
 * which by default is owned by root.
 * If you dont like it change the link to a local owned directory.
 * You must then also change the SerialCommunication default address. Also when you do this change
 * be mindful that you have to change it back to connect to the real APM.
 */
const char *COMMANDS_SERIAL_EMULATOR[] = {
		"socat", "-d", "-d", "TCP4:127.0.0.1:5760", "PTY,link=/dev/ttyACM0,raw,echo=0",
		(const char*)NULL
};

const char *COMMANDS_SITL[] = {
//		"python", "-m", "cProfile", "-o", "Simulator/Simulator.prof",
		"Simulator/Simulator.py",
		"--frame=+", HOMESTR, FOVSTR, RESOLUTIONSTR,
		(const char*)NULL
};

const char *COMMANDS_RENDER_CALLIBRATE[] = {
		"Simulator/Simulator.py",
		"--frame=+", HOMESTR, "--render=True", FOVSTR, RESOLUTIONSTR,
		"--image=chessboard.png",
		(const char*)NULL
};

//http://upload.wikimedia.org/wikipedia/commons/d/d5/Chess_Board.svg
const char *COMMANDS_CAMERA_CALIBRATE[] = {
		"Simulator/CameraCalibration",
		"-w", "6", "-h", "6", "-s", "0.125", "-o", "camera.yml", "-op", "-oe", //:"su", "-n", "25",
		(const char*)NULL
};

ProcessSpawner * ProcessSpawner::process_spawner_pointer = nullptr;

ProcessSpawner::ProcessSpawner() :
	child_pid{0},
	current_children{PLATFORM_PICTURE},
	COMMANDS{COMMANDS_PLATFORM_PICTURE,
		COMMANDS_ARDUPILOT,
		COMMANDS_SERIAL_EMULATOR,
		COMMANDS_SITL,
		COMMANDS_RENDER_CALLIBRATE,
		COMMANDS_CAMERA_CALIBRATE } {
	if (process_spawner_pointer == nullptr)
		process_spawner_pointer = this;
	else
		throw(std::runtime_error("Singleton cant have another pointer\n"));
}

void ProcessSpawner::spawnProcess(ChildrenProcessesToExecute process) {
	if (process != MAX_CHILDREN) {
		current_children = process;
		printf("Spawning %s\n", COMMANDS[process][0]);
		child_pid[current_children] = fork();
		isValidPIDOrDie();

		if (child_pid[current_children] == 0) { //child
			execvp(COMMANDS[process][0], (char* const*)COMMANDS[process]); //cast is acceptable
			exit(0);
		}
		sleep(3);
	}
}

void ProcessSpawner::killProcesses() {
	for (int i = 0; i <MAX_CHILDREN ; i++) {
		killProcess(static_cast<ChildrenProcessesToExecute>(i));
	}
	sleep(4); //give time to settle down
}

void ProcessSpawner::killProcess(const ChildrenProcessesToExecute& process) {
	if (child_pid[process] != 0) {
		kill(child_pid[process], SIGTERM);
	}
}

void ProcessSpawner::isValidPIDOrDie() {
	if (child_pid[current_children] < 0) {
		std::cout << "Failed to fork GUI process...Exiting" << std::endl;
		exit (-1);
	}
}
