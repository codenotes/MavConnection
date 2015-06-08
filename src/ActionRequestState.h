/*
 * ActionRequestState.h
 *
 *  Created on: Jun 7, 2014
 *      Author: pneves
 */

#ifndef ACTIONREQUESTSTATE_H_
#define ACTIONREQUESTSTATE_H_

#include <assert.h>
#include <atomic>
#include <exception>

namespace Mav {
	class ActionRequestState {
	public:
		enum ActionRequestStateEnum {
			OVERRIDE_RC_CHANNEL,
			SEND_LONG_COMMAND,
			SET_MODE,
			SET_NEW_WAYPOINTS,
			SET_PARTIAL_WAYPOINT_LIST,
			REQUEST_WAYPOINT_LIST,
			GET_PARAMETER,
			SET_PARAMETER,
			HALT_DATASTREAM,
			RESUME_DATASTREAM,
			SET_CURRENT_WAYPOINT,
			TIMED_OUT,
			NOTHING_TO_DO
		};
	private:
		std::atomic<ActionRequestStateEnum> action_request_state;
	public:
		ActionRequestState() :
			action_request_state(NOTHING_TO_DO) {
	}

		inline void setState(ActionRequestStateEnum new_action_request_state) {
			assert(new_action_request_state != NOTHING_TO_DO);//it breaks the action executed if allowed
			action_request_state.store(new_action_request_state);
		}

		inline ActionRequestStateEnum getState() const {
			return action_request_state.load();
		}

		inline bool isInProgress() const { return (getState() != NOTHING_TO_DO) ? true : false; }

		/* So that an action state is not reseted while actions are still progressing
		 * except of course if the parsing has been canceled at all, in which case
		 * all running actions are invalid */
		inline void resetState() {
			//assert(waypoint_send_state.isInProgress() && message_parser_thread == PARSING);
			//requestDatastream(true);
			action_request_state.store(NOTHING_TO_DO);
		}


		void waitForReset();

	};
}
#endif /* ACTIONREQUESTSTATE_H_ */
