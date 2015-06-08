/*
 * ActionRequestState.cpp
 *
 *  Created on: Jun 7, 2014
 *      Author: pneves
 */

#include "ActionRequestState.h"
#include <chrono>
#include <thread>
#include <iostream>
using namespace std::chrono;

namespace Mav {

	void ActionRequestState::waitForReset() {
		auto timeout_start =
				std::chrono::steady_clock::now();
		while(isInProgress()) {
			auto current_time = std::chrono::steady_clock::now();
			auto duration = duration_cast<milliseconds>(
					current_time - timeout_start );
			if (duration.count() < 4000) {
				std::this_thread::yield();
			}
			else {
				printf("Time out occured on action state %d\n", getState());
				action_request_state = TIMED_OUT;
				throw std::runtime_error("Timed Out occured\n");
			}
		}
	}
}
