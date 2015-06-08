/*
 * MavCommander.h
 *
 *  Created on: Mar 27, 2014
 *      Author: pneves
 */

#ifndef MAVCOMMUNICATION_H_
#define MAVCOMMUNICATION_H_

//#include <sys/time.h>

//#include "SerialCommunication.h"
#include "ICommunication.h"
#include "Hardware.h"
#include <assert.h>
#include <thread>
#include <atomic>
#include <mutex>
#include "ActionRequestState.h"
#include "Waypoint.h"
#include "mavlink.h" //"mavlink/ardupilotmega/mavlink.h"



namespace Mav {

using namespace std;

using namespace Hardware;

	enum ConnectionType {
		TCP_IP,
		SERIAL
	};

	struct MAVFlightParameters {
		mavlink_global_position_int_t global_position;
		mavlink_attitude_t attitude;
		mavlink_battery_status_t battery_status;
		mavlink_vfr_hud_t vfr_data;
	};


	class MavCommunication {
	public:
		enum ParameterConstant {
			PARAMETER_ID_SIZE = 15
		};
		typedef char ParameterID[PARAMETER_ID_SIZE];

		struct MAVComponentParameters {
			const MAV_TYPE type;
			const MAV_AUTOPILOT autopilot_type;
			const uint8_t system_id;
			const uint8_t component_id;

			MAV_STATE commander_state;
			MAV_MODE_FLAG commander_mode;

			MAVComponentParameters(const MAV_TYPE _type, const MAV_AUTOPILOT _autopilot_type,
					MAV_STATE _commander_state, MAV_MODE_FLAG _commander_mode, const uint8_t _system_id,
					const uint8_t _component_id) :
						type(_type), autopilot_type(_autopilot_type),
						system_id(_system_id), component_id(_component_id), commander_state(_commander_state),
						commander_mode(_commander_mode) { }

		};
	private:
		enum ThreadState {
			INITIALIZING,
			PARSING,
			REQUEST_INTERRUPT,
			STOPPED
		};

		class ParameterHandlerState {
		public:
			ParameterHandlerState();
			enum ParameterHandlerStateEnum {
				IDLE,
				REQUEST_PARAMETER,
				PARAMETER_REQUESTED,
				SEND_PARAMETER,
				PARAMETER_SENT,
				HANDLER_FAIL,
				HANDLER_ACK,
				MAX_STATE
			};

			ParameterHandlerStateEnum getState() const;
			void setState(ParameterHandlerStateEnum new_state);
			void resetState();
			void setParameterIDToSet(const ParameterID new_parameter_id, float new_value);
			void setParameterValue(float new_value);
			void setParameterID(const ParameterID new_parameter_id);
			void setParameterIDToGet(const ParameterID new_parameter_id);
			bool isReceivedParameterEqual(const ParameterID parameter_to_check) const;
			float getParameterValue() const;
			void getCurrentParameterID(ParameterID destination_buffer) const;
		private:
			atomic<ParameterHandlerStateEnum> state;
			ParameterID parameter_id;
			atomic<float> parameter_value;
		};

		void setMAVParameterValue();
		void getMAVParameterValue();
		void parseMAVParameterValue();

		class ModeState{
		public:
			enum ModeStateEnum {
				MODE_OK,
				SET_STATE,
				AWAITING_MODE,
				MODE_SET_SUCCESS,
				MODE_SET_FAILED,
				MAX_STATE
			};
			//Taken from arducopter/defines.h enums start at 0 in std!
			enum ModeEnum {
				STABILIZE,// hold level position
				ACRO,// rate control
				ALT_HOLD,// AUTO control
				AUTO,// AUTO control
				GUIDED,// AUTO control
				LOITER,// Hold a single location
				RTL,// AUTO control
				CIRCLE,// AUTO control
				POSITION, // AUTO control
				LAND,// AUTO control
				OF_LOITER, // Hold a single location using optical flow sensor
				TOY_A, // THOR Enum for Toy mode
				TOY_M,// THOR Enum for Toy mode
				NUM_MODES
			};
		private:
			atomic<ModeStateEnum> state;
			atomic<ModeEnum> current_mode;
			atomic<ModeEnum> requested_mode;
		public:
			ModeState();
			void setState(ModeStateEnum new_state);
			ModeStateEnum getState() const;
			void setMode(ModeEnum new_mode);
			ModeEnum getMode() const;
			void requestNewMode(ModeEnum new_requested_mode);
			ModeEnum getRequestedMode() const;
			void resetState();
		};

		void parseMode(const mavlink_heartbeat_t& heartbeat);
		void setMAVMode();
		//same name but one is to initiate the transaction and the other is a setter
		// one is bool and other is void...ugly but for now its the convention I follow
		bool setMode(ModeState::ModeEnum new_mode);

		class WaypointReadState {
		public:
			enum WaypointReadStateEnum {
				START,
				REQUEST_WAYPOINT_LIST,
				READ_WAYPOINT_COUNT,
				REQUEST_READ_WAYPOINT,
				RECEIVED_WAYPOINT,
				WAYPOINT_RECEIVE_ACK
			};
		private:
			atomic<WaypointReadStateEnum> state;
			uint16_t current_seq;
			uint16_t waypoint_count;
		public:
			WaypointReadState();
			void setState(WaypointReadStateEnum new_state);
			void setCurrentSeq(uint16_t seq);
			void setWaypointCount(uint16_t count);
			WaypointReadStateEnum getState() const;
			uint16_t getCurrentSeq() const;
			void incrementCurrentSeq();
			uint16_t getWaypointCount() const;
			bool isInProgress() const;
			void resetState();
		};

		class WaypointSendState {
		public:
			enum WaypointSendStateEnum {
				START,
				SEND_WAYPOINT_COUNT,
				WAYPOINT_REQUEST,
				SENDING_WAYPOINT,
				WAYPOINT_SENT_ACK,
				WAYPOINT_SEND_FAILED
			};
		private:
			atomic<WaypointSendStateEnum> state;
			uint16_t current_seq;
			vector<Waypoint> waypoints_to_send;
		public:
			WaypointSendState();
			void setState(WaypointSendStateEnum new_state) ;
			void setCurrentSeq(uint16_t seq);
			WaypointSendStateEnum getState() const;
			uint16_t getCurrentSeq() const;
			bool isInProgress() const;
			void resetState();
			void setWaypointsToSend(const vector<Waypoint>& waypoints);
			const Waypoint& getNextWaypoint();
			const uint16_t getWaypointCount();
		};

		class CurrentWaypointSetState {
		public:
			enum CurrentWaypointSetStateEnum {
				START,
				SET_CURRENT_WAYPOINT,
				CURRENT_WAYPOINT_ANSWER,
				CURRENT_WAYPOINT_SET_FAILED,
				MAX_STATE
			};
		private:
			atomic<CurrentWaypointSetStateEnum> state;
			atomic<uint16_t> current_waypoint;
			mavlink_message_t& message;
			ActionRequestState& mav_request_state;
		public:
			CurrentWaypointSetState(mavlink_message_t& new_message,
					ActionRequestState& new_mav_requests_state);
			uint16_t getCurrentWayPoint();
			void setCurrentWaypoint(uint16_t new_current_waypoint);
			void setState(CurrentWaypointSetStateEnum new_state);
			CurrentWaypointSetStateEnum getState();
			void resetState();

		};
		//Add to CurrentWaypointSetState when g++ corrects the behavior where
		// nested classes can access parent's privates
		//https://gcc.gnu.org/bugzilla/show_bug.cgi?id=59482
		void parseReceivedWaypointCurrent();
		void setMAVCurrentWaypoint();

		class CommandState {
		public:
			enum CommandStateEnum {
				IDLE,
				DISARMED,
				ARMED,
				CALIBRATE_LEVEL,
				REQUEST_ARM_SETTING,
				WAITING_ACK,
				COMMAND_ACK,
				COMMAND_SUCCESS,
				COMMAND_FAILED
			};
			CommandState();
			CommandStateEnum getState() const;
			void setState(CommandStateEnum new_state);
			void setRequestedArmed(bool arm);
			bool getRequestArm();
			void resetState();
			void setMotorState(bool armed);
		private:
			atomic<CommandStateEnum> state;
			atomic<bool> request_arm; //false is disarm
		};

		void setMAVLongCommand();
		void parseMotorState(const mavlink_heartbeat_t& heartbeat);

		void parseCommandAck();

		class RCOverrideState {
		public:
			RCOverrideState();
			/*
			 * RC channel mapping taken from documentation
			 * http://copter.ardupilot.com/wiki/connecting-your-rc-input-and-motors/#Connecting_the_RC_inputs
			 * The way ardupilot handles the overrides in code is found here
			 * https://github.com/diydrones/ardupilot/blob/2874ec67c7a111f09fd57add336a99461e567b73/ArduCopter/GCS_Mavlink.pde#L1876
			 * https://github.com/diydrones/ardupilot/blob/2874ec67c7a111f09fd57add336a99461e567b73/libraries/AP_RCMapper/AP_RCMapper.cpp
			 */
			enum RCMappingEnum {
				ROLL,
				PITCH,
				THROTTLE,
				YAW,
				MODE_SWITCH,
				AUX2,
				AUX3,
				AUX4,
				MAX_MAPPING
			};

			enum RCOverrideStateEnum {
				OVERRIDE_DISARMED,
				OVERRIDE_ARMED
			};

			typedef uint16_t RCMapping[MAX_MAPPING];

			const uint16_t MAX_CHANNEL_VALUE;
			const uint16_t MIN_CHANNEL_VALUE;
			void setChannel(const RCMappingEnum& channel, const uint16_t& value, const bool& enable_override);
			void setChannels(const RCMapping new_channel_values, const bool& enable_override);
			void getChannels(RCMapping channel_buffer) const; //this actually return an array of constant references, type safety and all...
			const RCOverrideStateEnum getState() const;
			void setState(const RCOverrideStateEnum& new_state);
		private:
			RCMapping rc_mapping;
			atomic<RCOverrideStateEnum> state;


		};

		bool ready_to_fly;
		ConnectionType connection_type;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
		mavlink_message_t message;
		mavlink_status_t status;
		ICommunication* communication_interface;
		const HardwareData& hardware_data;
		thread instance_thread;
		atomic<ThreadState> message_parser_thread;

		vector<Waypoint> waypoint_list;

		ModeState mode_state;
		ActionRequestState action_request_state;
		WaypointSendState waypoint_send_state;
		WaypointReadState waypoint_read_state;
		CurrentWaypointSetState current_waypoint_set_state;
		CommandState command_state;
		RCOverrideState rc_override_state;
		ParameterHandlerState parameter_handler_state;
		mutex instance_mutex;
		const uint8_t NUMBER_OF_RETRIES;
		MAVFlightParameters flight_parameters;
		MAVComponentParameters mav_component_parameters;
		char *text_message;

		ICommunication* const getCommunicationInterface();
		void ActionParserExecutive();
		void parseHeartBeat(); /* Todo: The parsing function need to check the destinataries */
		void parseGlobalPosition();
		void parseTime();
		void parseReceivedParameters();
		void parseAttitude();
		void parseBatteryLevel();
		void parseReceivedMessages();
		void parseWaypointCount();
		void parseWaypointRequest();
		void parseWaypointAck();
		void parseReceivedWaypoint();
		void parseTextMessage();
		void parseVfrHUD();
		void parseRC();
		void parseServoOutput();
		void requestParameter(uint16_t parameter_id);
		void requestWaypointListFromAPM();
		void requestDatastream(bool start_stop);
		void setMAVWaypoints();
		void getMAVWaypointList();
		void requestToReadWaypoint();
		void sendHeartBeat();
		void sendWaypointCount();
		void sendWaypointAck();
		bool isCommunicationInterfaceValid() const;
		void haltDatastream();
		void resumeDatastream();
		void getMavFlightParameters(MAVFlightParameters &buffer);
	public:
		void getFlightParameters(Hardware::FlightParameters& buffer);

		MavCommunication(MavCommunication const&) = delete; //because of threads and atomics
		MavCommunication& operator =(MavCommunication const&) = delete;
		MavCommunication(const HardwareData& hardware);
		bool isMavConnectionValid() const;
		bool sendNewWaypointList(const vector<Waypoint>& waypoints_to_send);
		bool setStabilizeMode();
		bool setLoiterMode();
		bool setAltitudeHoldMode();
		bool setAutoMode();
		vector<Waypoint> getWaypointList();
		bool isWaypointOnMAV(const Waypoint& waypoint_to_check, bool refresh_waypoints,
				uint16_t id_hint=0);
		bool isInStabilizeMode() const;
		bool isInLoiterMode() const;
		bool setCurrentWaypoint(uint16_t waypoint_number);
		bool setMotorsArmed(bool armed);
		void setMAVRC();
		void setRCChannels(bool effective_now, uint16_t throttle = 0, uint16_t roll = 0, uint16_t pitch = 0, uint16_t yaw = 0, uint16_t mode = 1815);
		void setupRC();
		void disableOverride();
		bool setEEPROMParameter(const string& parameter, const float& destination_value);
		bool getEEPROMParameter(const string& parameter, float& buffer);
		bool requestLevelCalibration();
		~MavCommunication();
	};
}

#endif /* MAVCOMMUNICATION_H_ */
