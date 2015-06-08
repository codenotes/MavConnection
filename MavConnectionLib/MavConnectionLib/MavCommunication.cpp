/*
 * MavCommander.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: pneves
 */

#define WIN32_LEAN_AND_MEAN


#include "windows.h"

#undef SendMessage

#include <vector>
#include "MavCommunication.h"
//#include "mavlink/ardupilotmega/ardupilotmega.h"
#include "mavlink.h" //"mavlink/common/mavlink.h"
#include <stdio.h>

#include "communication.h"
#include <boost/thread.hpp>

void usleep(DWORD waitTime) {
	LARGE_INTEGER perfCnt, start, now;

	QueryPerformanceFrequency(&perfCnt);
	QueryPerformanceCounter(&start);

	do {
		QueryPerformanceCounter((LARGE_INTEGER*)&now);
	} while ((now.QuadPart - start.QuadPart) / float(perfCnt.QuadPart) * 1000 * 1000 < waitTime);
}


#define sleep(X) Sleep(X);

using namespace::std;


#define PROTOCOL_DELAY_US 20000

#define MAVLINK_PACK_ARGUMENTS_WRAPPER mav_component_parameters.system_id, \
					mav_component_parameters.component_id, \
					&message, \
					hardware_data.system_id, \
					hardware_data.component_id



namespace Mav {



	//Whatever mode we pick up in the heartbeat
	//is going to be set right away.
	MavCommunication::ModeState::ModeState()
		: state(MODE_OK), current_mode(NUM_MODES) {

	}

	void MavCommunication::ModeState::setState(ModeStateEnum new_state) {
		state = new_state;
	}

	MavCommunication::ModeState::ModeStateEnum MavCommunication::ModeState::getState() const {
		return state;
	}

	void MavCommunication::ModeState::setMode(ModeEnum new_mode) {
		current_mode = new_mode;

//		if (getState() == AWAITING_MODE && requested_mode == new_mode) {
//			printf("new_mode %d, requested mode %d\n", requested_mode.load(), new_mode);
//			state = MODE_OK;
//		}
	}

	MavCommunication::ModeState::ModeEnum MavCommunication::ModeState::getMode() const {
		return current_mode;
	}

	void MavCommunication::ModeState::requestNewMode(ModeEnum new_requested_mode) {
		requested_mode = new_requested_mode;
		setState(SET_STATE);
	}

	MavCommunication::ModeState::ModeEnum MavCommunication::ModeState::getRequestedMode() const {
		return requested_mode;
	}

	void MavCommunication::ModeState::resetState() {
		requested_mode = getMode();
		setState(MODE_OK);
	}

	void MavCommunication::parseMode(const mavlink_heartbeat_t& heartbeat) {
		uint32_t custom_mode = heartbeat.custom_mode;
		if (custom_mode < ModeState::NUM_MODES) {
			//Cast from uint32_t to enum should offer no problems because the APM
			//allowed modes have low count and fall on int type range
			mode_state.setMode((ModeState::ModeEnum)custom_mode);
		}
	}


	void MavCommunication::setMAVMode() {
		if (mode_state.getState() == ModeState::SET_STATE) {
			//APM asks for MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
			mavlink_msg_set_mode_pack(mav_component_parameters.system_id,
					mav_component_parameters.component_id,
					&message,
					hardware_data.system_id,
					MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					mode_state.getRequestedMode());
			printf("Sent message to change mode to %d\n", mode_state.getRequestedMode());
			uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_len);
			mode_state.setState(ModeState::AWAITING_MODE);
		}
		else if (mode_state.getState() == ModeState::MODE_OK)
			action_request_state.resetState();
	}

	bool MavCommunication::setMode(ModeState::ModeEnum new_mode) {
		 uint8_t attempts_left = NUMBER_OF_RETRIES;
		 do {
			 try {
				action_request_state.waitForReset();
				mode_state.requestNewMode(new_mode);
				action_request_state.setState(action_request_state.SET_MODE);
				action_request_state.waitForReset();
				if (mode_state.getState() == ModeState::MODE_SET_SUCCESS)
					return true; //because if we dont time out it means we did it :D
				else if (mode_state.getState() == ModeState::MODE_SET_FAILED)
					throw std::exception();
				else
					assert(1);
			 }
			 catch(...) {
				 attempts_left--;
				 mode_state.resetState();
				 action_request_state.resetState();
			 }
		 } while (attempts_left > 0 );

		 return false;
	}

	bool MavCommunication::setStabilizeMode() {
		return setMode(ModeState::ModeEnum::STABILIZE);
	}

	bool MavCommunication::setLoiterMode() {
		return setMode(ModeState::ModeEnum::LOITER);
	}

	bool MavCommunication::setAltitudeHoldMode() {
		return setMode(ModeState::ALT_HOLD);
	}

	bool MavCommunication::setAutoMode() {
		return setMode(ModeState::AUTO);
	}

	bool MavCommunication::isInStabilizeMode() const {
		return (mode_state.getMode() == ModeState::STABILIZE);
	}

	bool MavCommunication::isInLoiterMode() const {
		return (mode_state.getMode() == ModeState::LOITER);
	}

	MavCommunication::WaypointReadState::WaypointReadState() :
			state(START), current_seq(0), waypoint_count(0) {
	}

	void MavCommunication::WaypointReadState::setState(
			WaypointReadStateEnum new_state) {
		//cout << "Read State " << new_state << endl;
		state.store(new_state);
	}

	void MavCommunication::WaypointReadState::setCurrentSeq(uint16_t seq) {
		current_seq = seq;
	}

	void MavCommunication::WaypointReadState::setWaypointCount(uint16_t count) {
		waypoint_count = count;
	}

	MavCommunication::WaypointReadState::WaypointReadStateEnum MavCommunication::WaypointReadState::getState() const {
		return state.load();
	}

	uint16_t MavCommunication::WaypointReadState::getCurrentSeq() const {
		return current_seq;
	}

	void MavCommunication::WaypointReadState::incrementCurrentSeq() {
		current_seq++;
	}

	uint16_t MavCommunication::WaypointReadState::getWaypointCount() const {
		return waypoint_count;
	}

	bool MavCommunication::WaypointReadState::isInProgress() const {
		return (getState() != START) ? true : false;
	}

	void MavCommunication::WaypointReadState::resetState() {
		current_seq = 0;
		waypoint_count = 0;
		setState(START);
	}

	void MavCommunication::getMAVWaypointList() {
		if (waypoint_read_state.getState() == waypoint_read_state.START) {
			requestWaypointListFromAPM();
			waypoint_read_state.setState(WaypointReadState::REQUEST_WAYPOINT_LIST);
		}
		else if(waypoint_read_state.getState() == WaypointReadState::READ_WAYPOINT_COUNT ||
				waypoint_read_state.getState() == WaypointReadState::RECEIVED_WAYPOINT) {
			if (waypoint_read_state.getCurrentSeq() < waypoint_read_state.getWaypointCount()) {
				requestToReadWaypoint();
				waypoint_read_state.setState(WaypointReadState::REQUEST_READ_WAYPOINT);
			}
			else {
				waypoint_read_state.setState(WaypointReadState::WAYPOINT_RECEIVE_ACK);
				sendWaypointAck();
				action_request_state.resetState();
			}
		}
	}

	void MavCommunication::parseWaypointCount() {
		mavlink_mission_count_t mission_count = {};
		mavlink_msg_mission_count_decode(&message, &mission_count);
		waypoint_read_state.setCurrentSeq(0);
		waypoint_read_state.setWaypointCount(mission_count.count);
		waypoint_list.clear();
		//waypoint_list.resize(mission_count.count);
		printf("Received count %d\n", mission_count.count);
		waypoint_read_state.setState(waypoint_read_state.READ_WAYPOINT_COUNT);
	}

	void MavCommunication::parseReceivedWaypoint() {
//		uint16_t current_seq = waypoint_read_state.getCurrentSeq();
//		if (current_seq == 0) {
//			waypoint_list.clear();
//		}
		mavlink_mission_item_t waypoint = {};
		mavlink_msg_mission_item_decode(&message, &waypoint);
		if (waypoint.seq < waypoint_read_state.getWaypointCount()) {
			printf("Seq %d/%lud\n", waypoint.seq, waypoint_list.size());
			waypoint_list.emplace(waypoint_list.begin() + waypoint.seq, waypoint.seq,
					waypoint.x, waypoint.y, waypoint.z, //x is latitude!!
					waypoint.autocontinue, waypoint.current,
					waypoint.param4, (MAV_FRAME)waypoint.frame,
					waypoint.param1, waypoint.param2);
			waypoint_read_state.setCurrentSeq(waypoint.seq+1);
			waypoint_read_state.setState(waypoint_read_state.RECEIVED_WAYPOINT);
			printf("Received valid waypoint %d\n", waypoint.seq);
		}
		else
			printf("Received invalid waypoint\n");
	}

	void MavCommunication::requestToReadWaypoint() {
		mavlink_msg_mission_request_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
				waypoint_read_state.getCurrentSeq());
		uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
		getCommunicationInterface()->SendMessage(buffer, message_length);
	}

	void MavCommunication::requestWaypointListFromAPM() {
		mavlink_msg_mission_request_list_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER);
		uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
		getCommunicationInterface()->SendMessage(buffer, message_length);

	}

	MavCommunication::WaypointSendState::WaypointSendState() :
		state{START}, current_seq{0}, waypoints_to_send(0) {}

	void MavCommunication::WaypointSendState::setState(WaypointSendStateEnum new_state) {
		//cout << "Send State " << new_state << endl;
		state.store(new_state);
	}
	void MavCommunication::WaypointSendState::setCurrentSeq(uint16_t seq) {
		//assert(current_seq == UINT16_MAX);
		if (current_seq == UINT16_MAX)
			current_seq = 0;
		else {
			assert(current_seq <= waypoints_to_send.size());
			current_seq = seq;
		}
	}

	MavCommunication::WaypointSendState::WaypointSendStateEnum MavCommunication::WaypointSendState::getState() const {
		return state.load();
	}

	uint16_t MavCommunication::WaypointSendState::getCurrentSeq() const {
		return current_seq;
	}

	bool MavCommunication::WaypointSendState::isInProgress() const {
		return (getState() != START) ? true : false;
	}

	void MavCommunication::WaypointSendState::resetState() {
		waypoints_to_send.clear();
		current_seq = 0;
		setState(START);
	}

	void MavCommunication::WaypointSendState::setWaypointsToSend(const vector<Waypoint>& waypoints) {
		waypoints_to_send = waypoints;
	}

	const Waypoint& MavCommunication::WaypointSendState::getNextWaypoint() {
		assert(!waypoints_to_send.empty());
		return waypoints_to_send[current_seq];
	}

	const uint16_t MavCommunication::WaypointSendState::getWaypointCount() {
		return waypoints_to_send.size();
	}

	MavCommunication::CurrentWaypointSetState::CurrentWaypointSetState(mavlink_message_t& new_message,
			ActionRequestState& new_mav_requests_state) :
		state(START),
		current_waypoint(0),
		message(new_message),
		mav_request_state(new_mav_requests_state) {
	}

	uint16_t MavCommunication::CurrentWaypointSetState::getCurrentWayPoint() {
		return current_waypoint;
	}

	void MavCommunication::CurrentWaypointSetState::setCurrentWaypoint(uint16_t new_current_waypoint) {
		current_waypoint = new_current_waypoint;
	}

	void MavCommunication::CurrentWaypointSetState::setState(CurrentWaypointSetStateEnum new_state) {
		state = new_state;
	}

	MavCommunication::CurrentWaypointSetState::CurrentWaypointSetStateEnum
		MavCommunication::CurrentWaypointSetState::getState() {
		return state;
	}

	void MavCommunication::CurrentWaypointSetState::resetState() {
		state = START;
		current_waypoint = 0;
	}

	void MavCommunication::parseReceivedWaypointCurrent() {
		if (action_request_state.getState() == ActionRequestState::SET_CURRENT_WAYPOINT) {
			mavlink_mission_current_t current_waypoint_response = {};
			mavlink_msg_mission_current_decode(&message, &current_waypoint_response);
			printf("Expected %d, Received %d\n",
					current_waypoint_set_state.getCurrentWayPoint(), current_waypoint_response.seq);

			if (current_waypoint_response.seq != current_waypoint_set_state.getCurrentWayPoint()) {
				current_waypoint_set_state.setState(CurrentWaypointSetState::CURRENT_WAYPOINT_SET_FAILED);
			}
			else
				current_waypoint_set_state.setState(CurrentWaypointSetState::CURRENT_WAYPOINT_ANSWER);
		}
	}

	void MavCommunication::setMAVCurrentWaypoint() {
		if (action_request_state.getState() == ActionRequestState::SET_CURRENT_WAYPOINT) {
			if (current_waypoint_set_state.getState() == CurrentWaypointSetState::START) {
				mavlink_msg_mission_set_current_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
						current_waypoint_set_state.getCurrentWayPoint());
				uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
				getCommunicationInterface()->SendMessage(buffer, message_len);
			}
			else if (current_waypoint_set_state.getState() == CurrentWaypointSetState::CURRENT_WAYPOINT_ANSWER) {
				//current_waypoint_set_state.resetState();
				action_request_state.resetState();
			}
			else if (current_waypoint_set_state.getState() == CurrentWaypointSetState::CURRENT_WAYPOINT_SET_FAILED) {
				action_request_state.resetState();
			}
			else //Unreachable
				assert(0);
		}
	}

	ICommunication* const MavCommunication::getCommunicationInterface() {
		while(message_parser_thread.load() == INITIALIZING)
			;
		assert(communication_interface != NULL);
		return communication_interface;
	}

	void MavCommunication::parseBatteryLevel() {
		mavlink_msg_battery_status_decode(&message, &flight_parameters.battery_status);
	}

	void MavCommunication::parseHeartBeat() {
		mavlink_heartbeat_t heartbeat_message = {};
		mavlink_msg_heartbeat_decode(&message, &heartbeat_message);
		parseMode(heartbeat_message);
		parseMotorState(heartbeat_message);
		sendHeartBeat();
		if (ready_to_fly == false)
			requestDatastream(true);
		printf("Autopilot %d, Base: %d, Custom: %d\n", heartbeat_message.autopilot, heartbeat_message.base_mode, heartbeat_message.custom_mode);
	}

	void MavCommunication::parseTime() {
		mavlink_system_time_t system_time;
		mavlink_msg_system_time_decode(&message, &system_time);
		if (system_time.time_boot_ms > 0)
			ready_to_fly = true;
		//printf("Time since boot: %d\n", system_time.time_boot_ms);
	}

	void MavCommunication::parseGlobalPosition() {
		mavlink_global_position_int_t global_position = {};
		mavlink_msg_global_position_int_decode(&message, &global_position);
		flight_parameters.global_position = global_position;
		//printf("Latitude %f, Longitude %f, Altitude %f, Heading %d\n", global_position.lat * 1.0e-7, global_position.lon *1.0e-7, global_position.hdg, (float)global_position.relative_alt * 0.01);
	}

	void MavCommunication::parseReceivedParameters() {
		mavlink_param_value_t received_parameter = {};
		mavlink_msg_param_value_decode(&message, &received_parameter);
		printf("Value:%f, Param Count%d, Param index:%d, Param Id:%s", received_parameter.param_value, received_parameter.param_count,
				received_parameter.param_index, received_parameter.param_id);
	}

	void MavCommunication::parseAttitude() {
		mavlink_attitude_t attitude = {};
		mavlink_msg_attitude_decode(&message, &attitude);
		flight_parameters.attitude = attitude;
		//printf("Pitch %f, Roll %f, Yaw %f\n", attitude.pitch, attitude.roll, attitude.yaw);
	}

	void MavCommunication::requestParameter(uint16_t parameter_id) {
		mavlink_param_request_read_t param_request = {};
		message = {};

		param_request.param_index = parameter_id;
		param_request.target_component = hardware_data.component_id;
		param_request.target_system = hardware_data.system_id;
		mavlink_msg_param_request_read_encode(mav_component_parameters.system_id, mav_component_parameters.component_id,
				&message, &param_request);
		uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
		assert(getCommunicationInterface()->SendMessage(buffer, message_len) == message_len);
	}

	void MavCommunication::parseWaypointRequest() {
		mavlink_mission_request_t request = {};
		mavlink_msg_mission_request_decode(&message, &request);
		waypoint_send_state.setCurrentSeq(request.seq);
		waypoint_send_state.setState(waypoint_send_state.WAYPOINT_REQUEST);
	}

	void MavCommunication::parseWaypointAck() {
		mavlink_mission_ack_t acknowledgment = {};
		mavlink_msg_mission_ack_decode(&message, &acknowledgment);
		if (acknowledgment.type == MAV_MISSION_ERROR) {
			waypoint_send_state.setState(waypoint_send_state.WAYPOINT_SEND_FAILED);
		}
		else {
			waypoint_send_state.setState(waypoint_send_state.WAYPOINT_SENT_ACK);
		}
	}

	void MavCommunication::parseTextMessage() {
		mavlink_msg_statustext_get_text(&message, text_message);
		if (memcmp(text_message, "PreArm: Alt disparity", strlen("PreArm: Alt disparity")))
			printf("Received Text: %s\n", text_message);
		else {
			//dont even ack it is urgent
			mavlink_msg_command_long_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					MAV_CMD_PREFLIGHT_CALIBRATION, 0,
					1, 0, 1, 1, 0, 0, 0);
			uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_length);
			command_state.setState(CommandState::WAITING_ACK);
			printf("Sent urgent Calibration request\n");
		}
	}

	void MavCommunication::parseVfrHUD() {
		mavlink_vfr_hud_t vfr_hud = {};
		mavlink_msg_vfr_hud_decode(&message, &vfr_hud);
		flight_parameters.vfr_data = vfr_hud;
		//printf("Throttle %d, Altitude %f, Heading %d\n", vfr_hud.throttle,
		//		vfr_hud.alt, vfr_hud.heading);
	}

	void MavCommunication::parseRC() {
		mavlink_rc_channels_raw_t rc_channel = {0};
		mavlink_msg_rc_channels_raw_decode(&message, &rc_channel);
		//printf("RC setpoint T:%d R:%d, Y:%d, P:%d\n", rc_channel.chan3_raw, rc_channel.chan1_raw, rc_channel.chan2_raw, rc_channel.chan4_raw);
	}

	void MavCommunication::parseServoOutput() {
		mavlink_servo_output_raw_t rc_output = {0};
		mavlink_msg_servo_output_raw_decode(&message, &rc_output);
		//printf("Output T:%d R:%d, Y:%d, P:%d\n", rc_output.servo3_raw, rc_output.servo1_raw, rc_output.servo2_raw, rc_output.servo4_raw);
	}

	void MavCommunication::parseReceivedMessages() {
		if (connection_type == SERIAL && communication_interface == NULL) {
			communication_interface = new SerialCommunicationNamespace::SerialCommunication();
			if (isCommunicationInterfaceValid() &&
					message_parser_thread.load() == INITIALIZING)
			{
				printf("storing PARSING in message parser thread\n");
				message_parser_thread.store(PARSING);
			}
			else {
				message_parser_thread = STOPPED;
				return;
			}
		}
		else
			return;
		sleep(4);
		sendHeartBeat();




		while(message_parser_thread.load() == PARSING &&
				isCommunicationInterfaceValid()) {

			ActionParserExecutive();

//			printf("parse loop thread id is %d\n", boost::this_thread::get_id());


			int result = getCommunicationInterface()->ReceiveMessage(buffer,
					sizeof(uint8_t) * MAVLINK_MAX_PACKET_LEN);
			if (result > 0) {
				for (int16_t i = 0; i < result; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
						switch(message.msgid) {
						case MAVLINK_MSG_ID_HEARTBEAT:
							parseHeartBeat();
							break;
						case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
							parseGlobalPosition();
							break;
						case MAVLINK_MSG_ID_SYSTEM_TIME:
							parseTime();
							break;
						case MAVLINK_MSG_ID_ATTITUDE:
							parseAttitude();
							break;
						case MAVLINK_MSG_ID_MISSION_COUNT:
							parseWaypointCount();
							break;
						case MAVLINK_MSG_ID_MISSION_REQUEST:
							parseWaypointRequest();
							break;
						case MAVLINK_MSG_ID_MISSION_ACK:
							parseWaypointAck();
							break;
						case MAVLINK_MSG_ID_MISSION_ITEM:
							parseReceivedWaypoint();
							break;
						case MAVLINK_MSG_ID_MISSION_CURRENT:
							parseReceivedWaypointCurrent();
							break;
						case MAVLINK_MSG_ID_STATUSTEXT:
							parseTextMessage();
							break;
						case MAVLINK_MSG_ID_COMMAND_ACK:
							parseCommandAck();
							break;
						case MAVLINK_MSG_ID_VFR_HUD:
							parseVfrHUD();
							break;
						case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
							parseRC();
							break;
						case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
							parseServoOutput();
							break;
						case MAVLINK_MSG_ID_PARAM_VALUE:
							parseMAVParameterValue();
							break;
						default:
							//printf("Unhandled Received message with ID %d, sequence: %d from component %d of system %d\n", message.msgid, message.seq, message.compid, message.sysid);
							break;

						ActionParserExecutive();
						}
						std::this_thread::yield();
					}
				}
			}
		}
		requestDatastream(false);
		action_request_state.resetState();
		delete communication_interface;
		communication_interface = NULL;
		message_parser_thread = STOPPED;
	}

	inline void MavCommunication::haltDatastream() {
		requestDatastream(false);
		action_request_state.resetState();
	}
	inline void MavCommunication::resumeDatastream() {
		requestDatastream(true);
		action_request_state.resetState();
	}

	void MavCommunication::ActionParserExecutive() {


		usleep(PROTOCOL_DELAY_US);
		//No messages no actions
		switch (action_request_state.getState()) {
		case ActionRequestState::OVERRIDE_RC_CHANNEL:
			setMAVRC();
			break;
		case ActionRequestState::SEND_LONG_COMMAND:
			setMAVLongCommand();
			break;
		case ActionRequestState::SET_MODE:
			setMAVMode();
			break;
		case ActionRequestState::REQUEST_WAYPOINT_LIST:
			getMAVWaypointList();
			break;
		case ActionRequestState::SET_NEW_WAYPOINTS:
			setMAVWaypoints();
			break;
		case ActionRequestState::HALT_DATASTREAM:
			haltDatastream();
			break;
		case ActionRequestState::RESUME_DATASTREAM:
			resumeDatastream();
			break;
		case ActionRequestState::SET_CURRENT_WAYPOINT:
			setMAVCurrentWaypoint();
			break;
		case ActionRequestState::SET_PARAMETER:
			setMAVParameterValue();
			break;
		case ActionRequestState::GET_PARAMETER:
			getMAVParameterValue();
			break;

		case ActionRequestState::NOTHING_TO_DO:
		default:
			break;
		}
	}

	void MavCommunication::requestDatastream(bool start_stop) {
		mavlink_msg_request_data_stream_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
				MAV_DATA_STREAM_ALL, 3, start_stop);
		uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
		assert(getCommunicationInterface()->SendMessage(buffer, message_len) == message_len);

	}
	bool MavCommunication::isCommunicationInterfaceValid() const {
		return (communication_interface != NULL &&
				communication_interface->getInterfaceState() == ICommunication::VALID) ? true : false;
	}

	bool MavCommunication::isMavConnectionValid() const {
		while (message_parser_thread == INITIALIZING || ready_to_fly == false)
		{
//			printf("NP--");
		}
 		return (message_parser_thread.load() == PARSING) ? true : false;
	}


	/*http://code.google.com/p/ardupilot-mega/wiki/MAVParam */
	MavCommunication::MavCommunication(const HardwareData& hardware) 
		:
			ready_to_fly(false),
			connection_type(SERIAL),
			buffer{0},
			message{0},
			status{0},
			communication_interface(NULL),
			hardware_data(hardware),
			instance_thread(&MavCommunication::parseReceivedMessages, this),
			message_parser_thread(INITIALIZING),
			waypoint_list(),
			mode_state(),
			action_request_state(),
			waypoint_send_state(),
			waypoint_read_state(),
			current_waypoint_set_state(message, action_request_state)		,
			command_state(),
			rc_override_state(),
			parameter_handler_state(),
			instance_mutex(),
			NUMBER_OF_RETRIES{3},
			
			flight_parameters{0}, //Greg

			mav_component_parameters(MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
				MAV_STATE_ACTIVE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 255, 0)
			
	{
		//text_message = NULL; 
//		memset(&flight_parameters, 0, sizeof(FlightParameters));
		text_message = new char[50];
	}

	void MavCommunication::getFlightParameters(Hardware::FlightParameters& buffer) {
		getMavFlightParameters(flight_parameters);
		buffer.pitch = flight_parameters.attitude.pitch;
		buffer.roll = flight_parameters.attitude.roll;
		buffer.yaw = flight_parameters.attitude.pitch;
		buffer.throttle = flight_parameters.vfr_data.throttle;
		buffer.x = Waypoint::convertLongitudeToMillimeter(flight_parameters.global_position.lon * 1.0e-7,
				flight_parameters.global_position.lat * 1.0e-7);
		buffer.y = Waypoint::convertLatitudeToMillimeter(flight_parameters.global_position.lat * 1.0e-7);
		buffer.z = flight_parameters.global_position.alt;
		buffer.time_from_boot = flight_parameters.global_position.time_boot_ms;
	}

	void MavCommunication::getMavFlightParameters(MAVFlightParameters& buffer) {
		instance_mutex.lock();
		buffer = flight_parameters;
		instance_mutex.unlock();
	}

	void MavCommunication::sendHeartBeat() {
		mavlink_msg_heartbeat_pack(mav_component_parameters.system_id,
				mav_component_parameters.component_id,
				&message,
				mav_component_parameters.type,
				mav_component_parameters.autopilot_type,
				mav_component_parameters.commander_mode,
				0, //TODO Check this
				mav_component_parameters.commander_state);
		uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
		getCommunicationInterface()->SendMessage(buffer, message_len);
	}

	void MavCommunication::sendWaypointCount() {
		mavlink_msg_mission_count_pack(
				MAVLINK_PACK_ARGUMENTS_WRAPPER, waypoint_send_state.getWaypointCount());
		uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
		getCommunicationInterface()->SendMessage(buffer, message_len);
	}

	void MavCommunication::setMAVWaypoints() {
		if (action_request_state.getState() == action_request_state.SET_NEW_WAYPOINTS) {
			if (waypoint_send_state.getState() == waypoint_send_state.START) {
				sendWaypointCount();
				waypoint_send_state.setState(waypoint_send_state.SEND_WAYPOINT_COUNT);
			}
			else if (waypoint_send_state.getState() == waypoint_send_state.WAYPOINT_REQUEST) {
				const Waypoint& current_waypoint = waypoint_send_state.getNextWaypoint();
				mavlink_msg_mission_item_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
						waypoint_send_state.getCurrentSeq(),
						current_waypoint.getFrame(),
						MAV_CMD_NAV_WAYPOINT,
						current_waypoint.getCurrent(),
						current_waypoint.getAutoContinue(),
						current_waypoint.getParam1(),
						current_waypoint.getParam2(),
						0,
						current_waypoint.getParam4(),
						current_waypoint.getLatitude(), // Latitude is actually X for ardupilot, who would have thought
						current_waypoint.getLongitude(), // and longitude Y!!!
						current_waypoint.getZ() //meters APMIssion.cpp617
						);

				uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
				getCommunicationInterface()->SendMessage(buffer, message_len);
				waypoint_send_state.setState(waypoint_send_state.SENDING_WAYPOINT);
			}
			else if (waypoint_send_state.getState() == waypoint_send_state.WAYPOINT_SENT_ACK) {
				action_request_state.resetState();
			}
			else if (waypoint_send_state.getState() == waypoint_send_state.WAYPOINT_SEND_FAILED) {
				action_request_state.resetState();
			}
		}
	}

bool MavCommunication::setMotorsArmed(bool arm) {
	/* we could compare the bool directly with the enum but then it would be dependent on
	*  enum order which is nasty for the future me.
	*  And I actually thought in c++ we could not compare enums with other types..but it
	*  seems we can, which is dangerous and error prone.
	*/
	CommandState::CommandStateEnum requested_arm = arm ?
			CommandState::ARMED : CommandState::DISARMED;
	if (requested_arm == command_state.getState()) {
		printf("Nothing to do to motor state %d\n", command_state.getState());
		return true;
	}

	uint8_t attempts_left = NUMBER_OF_RETRIES;
	do {
		try {
			action_request_state.waitForReset();
			command_state.setRequestedArmed(arm);
			command_state.setState(CommandState::REQUEST_ARM_SETTING);
			action_request_state.setState(ActionRequestState::SEND_LONG_COMMAND);
			action_request_state.waitForReset();
			if (command_state.getState() == CommandState::COMMAND_SUCCESS) {
				command_state.resetState();
				return true;
			}
			else {
				throw std::exception();
			}
		}
		catch (...) {
			printf("Motor Arm Timeout?\n");
			attempts_left--;
			command_state.resetState();
			action_request_state.resetState();
		}
	} while (attempts_left > 0 );
	return false;
}

bool MavCommunication::requestLevelCalibration() {
	uint8_t attempts_left = NUMBER_OF_RETRIES;
	do {
		try {
			action_request_state.waitForReset();

			command_state.setState(CommandState::CALIBRATE_LEVEL);
			action_request_state.setState(ActionRequestState::SEND_LONG_COMMAND);

			action_request_state.waitForReset();
			if (command_state.getState() == CommandState::COMMAND_SUCCESS) {
				command_state.resetState();
				return true;
			}
			else {
				throw std::exception();
			}
		}
		catch (...) {
			printf("Calibration Timeout?\n");
			attempts_left--;
			command_state.resetState();
			action_request_state.resetState();
		}
	} while (attempts_left > 0 );
	sleep(2);
	return false;
}


	MavCommunication::~MavCommunication() {
		while (message_parser_thread.load() != REQUEST_INTERRUPT) {
			message_parser_thread.store(REQUEST_INTERRUPT);
		}
		instance_thread.join();
		delete text_message;
	}

	void MavCommunication::sendWaypointAck() {
		mavlink_msg_mission_ack_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
				MAV_MISSION_ACCEPTED); //TODO: Check this
		uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
		getCommunicationInterface()->SendMessage(buffer, message_length);
	}


	vector<Waypoint> MavCommunication::getWaypointList() {
		uint8_t attempts_left = NUMBER_OF_RETRIES;
		do {
			try {
				action_request_state.setState(action_request_state.HALT_DATASTREAM);
				action_request_state.waitForReset();

				waypoint_read_state.resetState();
				waypoint_list.clear();
				action_request_state.setState(action_request_state.REQUEST_WAYPOINT_LIST);
				action_request_state.waitForReset();
				printf("waypoints received\n");

				action_request_state.setState(action_request_state.RESUME_DATASTREAM);
				action_request_state.waitForReset();
				printf("resumed datastream\n");
				if (!waypoint_list.empty() &&
						waypoint_read_state.getState() == WaypointReadState::WAYPOINT_RECEIVE_ACK) {
					waypoint_read_state.resetState();
					return waypoint_list;
				}
				else {
					printf("Ui\n");
					throw std::exception();
				}
			}
			catch (...) {
				printf("Exeception Timout?\n");
				attempts_left--;
				waypoint_read_state.resetState();
				waypoint_list.clear();
				action_request_state.resetState();
			}
		} while (attempts_left > 0 );
		waypoint_list.clear();
		return waypoint_list;
	}

	bool MavCommunication::sendNewWaypointList(const vector<Waypoint>& waypoints_to_send) {
		uint8_t attempts_left = NUMBER_OF_RETRIES;
		do {
			try {
				action_request_state.waitForReset();
				waypoint_send_state.setWaypointsToSend(waypoints_to_send);
				action_request_state.setState(action_request_state.SET_NEW_WAYPOINTS);
				action_request_state.waitForReset();
				if (waypoint_send_state.getState() != waypoint_send_state.WAYPOINT_SEND_FAILED) {
					waypoint_send_state.resetState();
					return true;
				}
				else
					throw std::exception();
			}
			catch(...) {
				attempts_left--;
				waypoint_send_state.resetState();
				action_request_state.resetState();
			}
		} while (attempts_left > 0 );
		return false;
	}

	bool MavCommunication::setCurrentWaypoint(uint16_t waypoint_number) {
		uint8_t attempts_left = NUMBER_OF_RETRIES;
		do {
			try {
				action_request_state.waitForReset();
				current_waypoint_set_state.setCurrentWaypoint(waypoint_number);
				action_request_state.setState(action_request_state.SET_CURRENT_WAYPOINT);
				action_request_state.waitForReset();
				if (current_waypoint_set_state.getState() ==
						CurrentWaypointSetState::CURRENT_WAYPOINT_ANSWER) {
					current_waypoint_set_state.resetState();
					return true;
				}
				else
					throw std::exception();
			}
			catch (...) {
				attempts_left--;
				current_waypoint_set_state.resetState();
				action_request_state.resetState();
			}
		} while (attempts_left > 0 );

		return false;
	}

	bool MavCommunication::isWaypointOnMAV(const Waypoint& waypoint_to_check,
			bool refresh_waypoints, uint16_t id_hint) {

		if (refresh_waypoints == true) {
			getWaypointList();
		}

		if (id_hint != 0 && waypoint_to_check == waypoint_list.at(id_hint))
			return true;
		else {
			for (auto waypoint : waypoint_list) {
				if (waypoint_to_check == waypoint)
					return true;
			}
		}
		return false;
	}

	MavCommunication::CommandState::CommandState() :
			state(DISARMED),
			request_arm(false) {
	}

	MavCommunication::CommandState::CommandStateEnum
			MavCommunication::CommandState::getState() const {
		return state;
	}

	void MavCommunication::CommandState::setState(CommandStateEnum new_state) {
		state = new_state;
	}

	void MavCommunication::CommandState::setRequestedArmed(bool arm) {
		request_arm = arm;
	}
	
	bool MavCommunication::CommandState::getRequestArm() {
		return request_arm;
	}
	
	void MavCommunication::CommandState::resetState() {
		if (getState() == ARMED)
			request_arm = true;
		else
			request_arm = false;
		setState(IDLE);
	}

	void MavCommunication::parseCommandAck() {
		//We currently do no check for the command...:(
		mavlink_command_ack_t received_ack = {};
		mavlink_msg_command_ack_decode(&message, &received_ack);
		printf("Command %d Result %d\n", received_ack.command, received_ack.result);
		if (action_request_state.getState() == ActionRequestState::SEND_LONG_COMMAND &&
				command_state.getState() == CommandState::WAITING_ACK) {
			if (received_ack.result == 0) { //check enum MAV_RESULT
				command_state.setState(CommandState::COMMAND_ACK);
				printf("Ack, result successful\n");
			}
			else {
				command_state.setState(CommandState::COMMAND_FAILED);
				printf("Command setting not successful\n");
			}
		}
		else if (action_request_state.getState() == ActionRequestState::SET_MODE &&
				mode_state.getState() == ModeState::AWAITING_MODE) {
			if (received_ack.result == 0) { //check enum MAV_RESULT
				mode_state.setState(ModeState::MODE_SET_SUCCESS);
				printf("mode set successful\n");
			}
			else {
				mode_state.setState(ModeState::MODE_SET_FAILED);
				printf("mode set NOT successful\n");
			}
			action_request_state.resetState();
		}

	}


/*TODO */
	void MavCommunication::parseMotorState(const mavlink_heartbeat_t& heartbeat) {
			bool apm_armed = heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
			//This is a trick to check for successful initialization, not really if it is armed or not
			if (ready_to_fly == false && apm_armed != 0) {
				ready_to_fly = true;
			}

			/*if (action_request_state.getState() == ActionRequestState::SEND_LONG_COMMAND &&
				command_state.getState() >= CommandState::WAITING_HEARTBEAT_ACK1 &&
				command_state.getState() < CommandState::COMMAND_FAILED) {
				if (apm_armed == command_state.getRequestArm()) {
					command_state.setState(CommandState::COMMAND_ACK);
					action_request_state.resetState();
				}
				else {
					CommandState::CommandStateEnum new_state =
							(CommandState::CommandStateEnum)((int)command_state.getState() + 1);
					command_state.setState(new_state);
					printf("heartbeat didnt receive the exepected, new try is %d\n", new_state);
				}
			}
			else */if (action_request_state.getState() != ActionRequestState::SEND_LONG_COMMAND) {
				command_state.setMotorState(apm_armed);
				//printf("Motors arm state %d\n", apm_armed);
			}
		}

	void MavCommunication::CommandState::setMotorState(bool armed) {
		if (armed == true) {
			setState(ARMED);
		}
		else
			setState(DISARMED);
	}

	void MavCommunication::setMAVLongCommand() {
		if (command_state.getState() == CommandState::REQUEST_ARM_SETTING) {
			float value_to_send = (command_state.getRequestArm()) ? 1.0f : 0.0f;
			mavlink_msg_command_long_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					MAV_CMD_COMPONENT_ARM_DISARM, 1,
					value_to_send, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

			uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_length);
			command_state.setState(CommandState::WAITING_ACK);
			printf("Sent Request change to %f\n", value_to_send);
		}
		else if(command_state.getState() == CommandState::COMMAND_FAILED) {
			action_request_state.resetState();
		}
		else if (command_state.getState() == CommandState::CALIBRATE_LEVEL) {
			mavlink_msg_command_long_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                    1, 0, 1, 1, 0, 0, 0);
			uint16_t message_length = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_length);
			command_state.setState(CommandState::WAITING_ACK);
			printf("Sent Calibration request\n");
		}
		else if (command_state.getState() == CommandState::COMMAND_ACK) {
			command_state.setState(CommandState::COMMAND_SUCCESS);
			action_request_state.resetState();
		}
	}

	MavCommunication::RCOverrideState::RCOverrideState()
		:
			MAX_CHANNEL_VALUE(2000),
			MIN_CHANNEL_VALUE(1000)//,
			/*rc_mapping{MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE,
					MIN_CHANNEL_VALUE},
			state(OVERRIDE_DISARMED)*/
			
			{

			}

//	void MavCommunication::RCOverrideState::setChannel(const RCMappingEnum& channel, const uint16_t& new_value,
//			const bool& enable_override) 
//	{
//		if (enable_override)
//			setState(OVERRIDE_ARMED);
//		else
//			setState(OVERRIDE_DISARMED);
//#ifdef _TEST
//		assert(channel != MAX_MAPPING);
//		assert(new_value == 0 || (new_value >= MIN_CHANNEL_VALUE &&
//				new_value <= MAX_CHANNEL_VALUE));
//#endif
//		rc_mapping[channel] = new_value;
//	}

	void MavCommunication::RCOverrideState::setChannels(const RCMapping new_channel_values, const bool& enable_override) 
	{
		if (enable_override)
			setState(OVERRIDE_ARMED);
		else
			setState(OVERRIDE_DISARMED);

#ifdef _TEST
		/*
		 * PPM ranges from 1000 0% to 2000 - 100%
		 * 0 means override is off
		 */
		for (uint8_t i = 0; i < MAX_MAPPING; i++) {
			assert((new_channel_values[i] == 0 || (new_channel_values[i] >= MIN_CHANNEL_VALUE &&
					new_channel_values[i] <= MAX_CHANNEL_VALUE)) ||
					new_channel_values[i] == UINT16_MAX);
		}
#endif
		memcpy(rc_mapping, new_channel_values, sizeof(RCMapping));
	}

	/*
	 * The return value is value copied so that strange things don't happen in the user code.
	 */
	void MavCommunication::RCOverrideState::getChannels(RCMapping channel_buffer) const 
	{
		/*
		 * A lock must be made. Don't want to pass the atomic array box. Probably weird things inside.
		 * The lock should exist if the class was nested like in C+11, but it isnt! The integration
		 * is weaker because of that.
		 */
//		instance_mutex.lock();
		memcpy(channel_buffer, rc_mapping, sizeof(RCMapping));
//		instance_mutex.unlock();
	}

	/*
	 * Cannot return by reference because we dont want to return the atomic itself
	 * even though it would be constant.
	 */
	const MavCommunication::RCOverrideState::RCOverrideStateEnum		MavCommunication::RCOverrideState::getState() const 
	{

		return state;

	}

	void MavCommunication::RCOverrideState::setState(const RCOverrideStateEnum& new_state) 
	{
		state = new_state;
	}

	void MavCommunication::setRCChannels(bool effective_now, uint16_t throttle, uint16_t roll, uint16_t pitch,  uint16_t yaw, uint16_t mode) {
		clock_t begin = clock();
		action_request_state.waitForReset();
		RCOverrideState::RCMapping channel_buffer = {0};
		channel_buffer[RCOverrideState::ROLL] = roll;
		channel_buffer[RCOverrideState::PITCH] = pitch;
		channel_buffer[RCOverrideState::THROTTLE]  = throttle;
		channel_buffer[RCOverrideState::YAW] = yaw;
		channel_buffer[RCOverrideState::MODE_SWITCH] = mode;
		instance_mutex.lock();
		rc_override_state.setChannels(channel_buffer, effective_now);
		instance_mutex.unlock();
		action_request_state.setState(ActionRequestState::OVERRIDE_RC_CHANNEL);
		action_request_state.waitForReset();
		double elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;
		printf("Set channels elapsed time %f\n", elapsed_secs);
	}

	void MavCommunication::disableOverride() {
		action_request_state.waitForReset();
		rc_override_state.setState(RCOverrideState::OVERRIDE_DISARMED);
		action_request_state.setState(ActionRequestState::OVERRIDE_RC_CHANNEL);
		action_request_state.waitForReset();
	}

	void MavCommunication::setupRC() {
		action_request_state.waitForReset();
		RCOverrideState::RCMapping channel_buffer;
		std::fill_n(channel_buffer,(int) RCOverrideState::MAX_MAPPING, 1500);
		channel_buffer[RCOverrideState::THROTTLE] = 1000;
		instance_mutex.lock();
		rc_override_state.setChannels(channel_buffer, false);
		instance_mutex.unlock();
		action_request_state.waitForReset();
	}

	void MavCommunication::setMAVRC() {
			/*
			 * Unfortunately we have no way of knowing if the command was received hmmm
			 * Hopefully we are fast sending another one.
			 */
			RCOverrideState::RCMapping channel_buffer = {0};
			if (rc_override_state.getState() == RCOverrideState::OVERRIDE_ARMED) {
				rc_override_state.getChannels(channel_buffer);
			}
			/*
			 * else zeroing disables the override, we initialized the buffer with 0s
			 */
			mavlink_msg_rc_channels_override_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					channel_buffer[RCOverrideState::ROLL],
					channel_buffer[RCOverrideState::PITCH],
					channel_buffer[RCOverrideState::THROTTLE],
					channel_buffer[RCOverrideState::YAW],
					/*channel_buffer[RCOverrideState::MODE_SWITCH]*/0,
					/*channel_buffer[RCOverrideState::AUX2]*/0,
					/*channel_buffer[RCOverrideState::AUX3]*/0,
					/*channel_buffer[RCOverrideState::AUX4]*/0);
			uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_len);
			action_request_state.resetState();
	}

	MavCommunication::ParameterHandlerState::ParameterHandlerState() :
		state(IDLE),
		parameter_id{},
		parameter_value(0.0) {

	}

	MavCommunication::ParameterHandlerState::ParameterHandlerStateEnum
		MavCommunication::ParameterHandlerState::getState() const {
		return state;
	}

	void MavCommunication::ParameterHandlerState::setState(
			ParameterHandlerStateEnum new_state) {
		state = new_state;
	}

	void MavCommunication::ParameterHandlerState::resetState() {
		setState(IDLE);
	}

	void MavCommunication::ParameterHandlerState::setParameterIDToSet(
			const ParameterID new_parameter,
			float new_value) {
		setParameterID(new_parameter);
		setParameterValue(new_value);
	}

	void MavCommunication::ParameterHandlerState::setParameterValue(float new_value) {
		parameter_value = new_value;
	}

	void MavCommunication::ParameterHandlerState::setParameterIDToGet(
			const ParameterID new_parameter) {
		setParameterID(new_parameter);
	}

	float MavCommunication::ParameterHandlerState::getParameterValue() const {
		return parameter_value;
	}

	bool MavCommunication::ParameterHandlerState::isReceivedParameterEqual(
			const ParameterID parameter_to_check) const {
		if (strncmp(parameter_to_check, parameter_id, PARAMETER_ID_SIZE) == 0)
			return true;
		else
			return false;
	}

	void MavCommunication::parseMAVParameterValue() {
		mavlink_param_value_t received_parameter_value = {};

		mavlink_msg_param_value_decode(&message, &received_parameter_value);
		printf("Parameter Received %s: %f\n", received_parameter_value.param_id, received_parameter_value.param_value);
		printf("State %d\n", parameter_handler_state.getState());


		assert(parameter_handler_state.getState() == ParameterHandlerState::PARAMETER_REQUESTED ||
				parameter_handler_state.getState() == ParameterHandlerState::PARAMETER_SENT);
		if (parameter_handler_state.isReceivedParameterEqual(
				received_parameter_value.param_id) == true) {
			if ((parameter_handler_state.getState() == ParameterHandlerState::PARAMETER_SENT &&
					parameter_handler_state.getParameterValue() == received_parameter_value.param_value) ||
					parameter_handler_state.getState() == ParameterHandlerState::PARAMETER_REQUESTED){
				parameter_handler_state.setState(ParameterHandlerState::HANDLER_ACK);
			}
			else
				parameter_handler_state.setState(ParameterHandlerState::HANDLER_FAIL);
		}
		else {
			parameter_handler_state.setState(ParameterHandlerState::HANDLER_FAIL);
		}
		// Even if we would fail to read the the correct value we should not be desynchronized
		parameter_handler_state.setParameterID(received_parameter_value.param_id);
		parameter_handler_state.setParameterValue(received_parameter_value.param_value);

	}

	void MavCommunication::ParameterHandlerState::getCurrentParameterID(
			ParameterID destination_buffer) const {
		memcpy(destination_buffer, parameter_id, sizeof(ParameterID));
	}

	void MavCommunication::setMAVParameterValue() {
		if (parameter_handler_state.getState() == ParameterHandlerState::SEND_PARAMETER) {
			ParameterID parameter_buffer = {};
			parameter_handler_state.getCurrentParameterID(parameter_buffer);
			mavlink_msg_param_set_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					parameter_buffer,
					parameter_handler_state.getParameterValue(),
					MAV_PARAM_TYPE_UINT8); //PARAM_TYPE can actually be other stuff..for ardupilot it is ignored but other APs may not be
			uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_len);
			parameter_handler_state.setState(ParameterHandlerState::PARAMETER_SENT);
		}
		else if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_FAIL ||
				parameter_handler_state.getState() == ParameterHandlerState::HANDLER_ACK) {
			action_request_state.resetState();
		}
		//waiting for answer..
	}

	void MavCommunication::ParameterHandlerState::setParameterID(
			const ParameterID new_parameter_id) {
		memcpy(parameter_id, new_parameter_id, sizeof(ParameterID));
	}

	void MavCommunication::getMAVParameterValue() {
		if (parameter_handler_state.getState() == ParameterHandlerState::REQUEST_PARAMETER) {
			ParameterID parameter_id_buffer = {};

			parameter_handler_state.getCurrentParameterID(parameter_id_buffer);
			mavlink_msg_param_request_read_pack(MAVLINK_PACK_ARGUMENTS_WRAPPER,
					parameter_id_buffer, -1);
			uint16_t message_len = mavlink_msg_to_send_buffer(buffer, &message);
			getCommunicationInterface()->SendMessage(buffer, message_len);
			parameter_handler_state.setState(ParameterHandlerState::PARAMETER_REQUESTED);

		}
		else if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_FAIL ||
				parameter_handler_state.getState() == ParameterHandlerState::HANDLER_ACK) {
			action_request_state.resetState();
		}
		//waiting for answer..
	}

	bool MavCommunication::getEEPROMParameter(const string& parameter, float& buffer) {
		uint8_t attempts_left = NUMBER_OF_RETRIES;
		ParameterID parameter_id = {};
		assert(parameter.length() < PARAMETER_ID_SIZE);
		memcpy(parameter_id, parameter.c_str(), parameter.length());

		do {
			try {
				action_request_state.waitForReset();

				instance_mutex.lock();
				parameter_handler_state.setParameterIDToGet(parameter_id);
				instance_mutex.unlock();

				parameter_handler_state.setState(ParameterHandlerState::REQUEST_PARAMETER);
				action_request_state.setState(ActionRequestState::GET_PARAMETER);

				action_request_state.waitForReset();
				if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_FAIL) {
					printf("Handler Failed\n");
					throw std::exception();
				}
				else if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_ACK) {
					buffer = parameter_handler_state.getParameterValue();
					parameter_handler_state.resetState();
					return true;
				}
				else
					assert(0); //unreachable
			}
			catch (...) {
				attempts_left--;
				parameter_handler_state.resetState();
				action_request_state.resetState();
			}
		} while (attempts_left > 0 );

		return false;
	}

	bool MavCommunication::setEEPROMParameter(const string& parameter, const float& destination_value) {
		uint8_t attempts_left = NUMBER_OF_RETRIES;
		ParameterID parameter_id = {};
		assert(parameter.length() < PARAMETER_ID_SIZE);
		memcpy(parameter_id, parameter.c_str(), parameter.length());

		do {
			try {
				action_request_state.waitForReset();

				instance_mutex.lock();
				parameter_handler_state.setParameterIDToSet(parameter_id, destination_value);
				instance_mutex.unlock();

				parameter_handler_state.setState(ParameterHandlerState::SEND_PARAMETER);
				action_request_state.setState(ActionRequestState::SET_PARAMETER);
				action_request_state.waitForReset();

				if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_FAIL) {
					throw std::exception();
				}
				else if (parameter_handler_state.getState() == ParameterHandlerState::HANDLER_ACK) {
					parameter_handler_state.resetState();
					return true;
				}
				else {
					printf("State %d\n", parameter_handler_state.getState());
					assert(0);
				}
			}
			catch (...) {
				attempts_left--;
				parameter_handler_state.resetState();
				action_request_state.resetState();
			}
		} while (attempts_left > 0 );

		return false;
	}

}
