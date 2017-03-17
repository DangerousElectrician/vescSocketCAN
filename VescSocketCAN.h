#include <iostream>
//socketcan includes
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>



class Vesc {
	public:
		typedef enum {
			MC_STATE_OFF = 0,
			MC_STATE_DETECTING,
			MC_STATE_RUNNING,
			MC_STATE_FULL_BRAKE,
		} mc_state;
		typedef enum {
			FAULT_CODE_NONE = 0,
			FAULT_CODE_OVER_VOLTAGE,
			FAULT_CODE_UNDER_VOLTAGE,
			FAULT_CODE_DRV8302,
			FAULT_CODE_ABS_OVER_CURRENT,
			FAULT_CODE_OVER_TEMP_FET,
			FAULT_CODE_OVER_TEMP_MOTOR
		} mc_fault_code;

	private:
		struct ifreq ifr;
		struct sockaddr_can addr;
		int s;
		int sbcm;

		uint8_t _enable = 1;

		uint8_t _controllerID;

		int32_t _rpm;
		float _current;
		float _duty_cycle;
		float _position;
		int32_t _tachometer;
		float _wattHours;
		float _vin;
		float _tempMotor;
		float _tempPCB;
		mc_fault_code _fault_code;
		mc_state _state;

		typedef struct VESC_set {
			int set:32;
		} VESC_set;
		typedef struct VESC_status{
			int32_t rpm:32;
			int16_t current:16;
			int16_t duty_cycle:16;
		} VESC_status;
		typedef struct VESC_status1{
			int rpm:32;
			int position:16;
			int motorCurrent:16;
		} VESC_status1;
		// most updated 100hz

		// slightly less updated 50hz
		typedef struct VESC_status2 { // 32bits
			int tachometer:32;
		} VESC_status2;

		// even less updated 10hz
		typedef struct VESC_status3 { // 26bits
			int wattHours:32;
			unsigned voltage:12;
		} VESC_status3;
		typedef struct VESC_status4 { // 25bits
			unsigned tempMotor:12;
			unsigned tempPCB:12;
			unsigned faultCode:3;
			unsigned state:2;
		} VESC_status4;


		void init_socketCAN(char *ifname);
		void processMessages();
	public:
		typedef enum {
			CAN_PACKET_SET_DUTY = 0,
			CAN_PACKET_SET_CURRENT,
			CAN_PACKET_SET_CURRENT_BRAKE,
			CAN_PACKET_SET_RPM,
			CAN_PACKET_SET_POS,
			CAN_PACKET_FILL_RX_BUFFER,
			CAN_PACKET_FILL_RX_BUFFER_LONG,
			CAN_PACKET_PROCESS_RX_BUFFER,
			CAN_PACKET_PROCESS_SHORT_BUFFER,
			CAN_PACKET_STATUS, // 9

			CAN_PACKET_STATUS1,
			CAN_PACKET_STATUS2,
			CAN_PACKET_STATUS3,
			CAN_PACKET_STATUS4

		} CAN_PACKET_ID;

		Vesc(char *interface, uint8_t controllerID);
		void setPoint(CAN_PACKET_ID mode, int setpoint);
		void setDuty(float dutyCycle);
		void setCurrent(float current);
		void setCurrentBrake(float current);
		void setRpm(float rpm);
		void setPos(float pos);

		void enable();
		void disable();

		int getRpm();
		float getCurrent();
		float getDutyCycle();
		float getPosition();
		int getTachometer();
		float getWattHours();
		float getVin();
		float getTempMotor();
		float getTempPCB();
		mc_fault_code getFaultCode();
		mc_state getState();
};
