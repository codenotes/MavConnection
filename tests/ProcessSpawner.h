/*
 * ProcessSpawner.h
 *
 *  Created on: Jun 2, 2014
 *      Author: pneves
 */

#ifndef PROCESSSPAWNER_H_
#define PROCESSSPAWNER_H_

#include <sys/types.h>


class ProcessSpawner {
public:
	enum ChildrenProcessesToExecute {
		PLATFORM_PICTURE,
		ARDUPILOT,
		SERIAL_EMULATOR,
		SITL,
		RENDER_CALIBRATE,
		COMMAND_CAMERA_CALIBRATE,
		MAX_CHILDREN
	};
private:
	pid_t child_pid[MAX_CHILDREN];
	ChildrenProcessesToExecute current_children;
	const char **COMMANDS[MAX_CHILDREN];
public:
	static ProcessSpawner * process_spawner_pointer;
	ProcessSpawner();
	void spawnProcess(ChildrenProcessesToExecute process);
	void killProcesses();
	void isValidPIDOrDie();
	void killProcess(const ChildrenProcessesToExecute& process);
};

#endif /* PROCESSSPAWNER_H_ */
