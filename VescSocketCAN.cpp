#include "VescSocketCAN.h"
#include <byteswap.h>
#include <cmath>

Vesc::Vesc(char *interface, uint8_t controllerID) {
	init_socketCAN(interface);
	_controllerID = controllerID;
}

void Vesc::init_socketCAN(char *ifname) {
	
	s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); //create nonblocking raw can socket
	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr); 
	addr.can_family = AF_CAN; 
	addr.can_ifindex = ifr.ifr_ifindex; 
	
	bind(s, (struct sockaddr *)&addr, sizeof(addr));

	sbcm = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
	connect(sbcm, (struct sockaddr *)&addr, sizeof(addr));
}

//TODO: figure out whether or not a destructor is needed

struct can_msg {
	struct bcm_msg_head msg_head;
	struct can_frame frame[1];
} msg;
void Vesc::setPoint(mc_control_mode mode, float setpoint) {
	if(_enable) {
		custom_control set;
		set.setpointf = setpoint;
		set.control_mode = mode;
		//struct can_frame frame;
		//frame.can_id = mode << 8 | _controllerID | 0x80000000;
		//frame.can_dlc = sizeof(VESC_set);
		//memcpy(frame.data, &set, sizeof(VESC_set));

		//write(s, &frame, sizeof(struct can_frame));


		msg.msg_head.opcode  = TX_SETUP;
		msg.msg_head.can_id  = CAN_PACKET_CONTROL << 8 | _controllerID | 0x80000000;
		msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
		msg.msg_head.nframes = 1;
		msg.msg_head.count = 0;
		msg.msg_head.ival1.tv_sec = 0;
		msg.msg_head.ival1.tv_usec = 0;
		msg.msg_head.ival2.tv_sec = 0;
		msg.msg_head.ival2.tv_usec = 1000*10;
		//msg.frame[0].can_id    = 0x42; /* obsolete when using TX_CP_CAN_ID */
		msg.frame[0].can_dlc   = sizeof(custom_control);
		memcpy(msg.frame[0].data, &set, 8);
		write(sbcm, &msg, sizeof(msg));
	}

}

void Vesc::setDuty(float dutyCycle) {
	setPoint(CONTROL_MODE_DUTY, dutyCycle);
}
void Vesc::setCurrent(float current) {
	setPoint(CONTROL_MODE_CURRENT, current);
}
void Vesc::setCurrentBrake(float current) {
	setPoint(CONTROL_MODE_CURRENT_BRAKE, current);
}
void Vesc::setRpm(float rpm) {
	setPoint(CONTROL_MODE_SPEED, rpm);
}
void Vesc::setPos(float pos) {
	setPoint(CONTROL_MODE_POS, pos);
}
void Vesc::setCustom(float setpoint) {
	setPoint(CONTROL_MODE_CUSTOM, setpoint);
	//if(_enable) {
	//	custom_control set;
	//	set.setpointf = setpoint;
	//	set.control_mode = CONTROL_MODE_CUSTOM;
	//	//struct can_frame frame;
	//	//frame.can_id = mode << 8 | _controllerID | 0x80000000;
	//	//frame.can_dlc = sizeof(VESC_set);
	//	//memcpy(frame.data, &set, sizeof(VESC_set));

	//	//write(s, &frame, sizeof(struct can_frame));


	//	msg.msg_head.opcode  = TX_SETUP;
	//	msg.msg_head.can_id  = CAN_PACKET_CONTROL << 8 | _controllerID | 0x80000000;
	//	msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
	//	msg.msg_head.nframes = 1;
	//	msg.msg_head.count = 0;
	//	msg.msg_head.ival1.tv_sec = 0;
	//	msg.msg_head.ival1.tv_usec = 0;
	//	msg.msg_head.ival2.tv_sec = 0;
	//	msg.msg_head.ival2.tv_usec = 1000*10;
	//	//msg.frame[0].can_id    = 0x42; /* obsolete when using TX_CP_CAN_ID */
	//	msg.frame[0].can_dlc   = sizeof(custom_control);
	//	memcpy(msg.frame[0].data, &set, 8);
	//	write(sbcm, &msg, sizeof(msg));
	//}
}

void Vesc::enable() {
	_enable = 1;
}
void Vesc::disable() {
	setCurrent(0);
	_enable = 0;
}

void Vesc::processMessages() {
	struct can_frame msg;
	while(1) {
		int a = read(s, &msg, sizeof(msg));
		if(a == -1) break;
		switch( (msg.can_id & ~0x80000000 & ~_controllerID) >> 8) {
			case CAN_PACKET_STATUS: //default status message, probably going to be unused but we can handle it if it does appear
				// received data is big endian
				_rpm = __bswap_32((*(VESC_status*) msg.data).rpm); // pointer casting!
				_current = ((int16_t) __bswap_16((*(VESC_status*) msg.data).current)) / 10.0; 
				_duty_cycle = ((int16_t) __bswap_16((*(VESC_status*) msg.data).duty_cycle)) / 1000.0;
				break;
			case CAN_PACKET_STATUS1: //custom status message
				_rpm = (*(VESC_status1*) msg.data).rpm;
				_current = (*(VESC_status1*) msg.data).motorCurrent / 10.0;
				_position = (*(VESC_status1*) msg.data).position / 1000.0;
				break;
			case CAN_PACKET_STATUS2:
				_tachometer = (*(VESC_status2*) msg.data).tachometer;
				break;
			case CAN_PACKET_STATUS3:
				_wattHours = (*(VESC_status3*) msg.data).wattHours;
				_inCurrent = (*(VESC_status3*) msg.data).inCurrent / 100.0;
				_vin = (*(VESC_status3*) msg.data).voltage;
				break;
			case CAN_PACKET_STATUS4:
				_tempMotor = (*(VESC_status4*) msg.data).tempMotor;
				_tempPCB = (*(VESC_status4*) msg.data).tempPCB;
				_fault_code = (mc_fault_code) (*(VESC_status4*) msg.data).faultCode;
				_state = (mc_state) (*(VESC_status4*) msg.data).state;
			default:
				break;
		}
	}
}

int Vesc::getRpm() {
	processMessages();
	return _rpm;
}

float Vesc::getCurrent() {
	processMessages();
	return _current;
}

float Vesc::getDutyCycle() {
	processMessages();
	return _duty_cycle;
}
float Vesc::getPosition() {
	processMessages();
	return _position;
}
int Vesc::getTachometer() {
	processMessages();
	return _tachometer;
}
float Vesc::getWattHours() {
	processMessages();
	return _wattHours;
}
float Vesc::getInCurrent() {
	processMessages();
	return _inCurrent;
}
#define VIN_R1                          39000.0
#define VIN_R2                          2200.0
#define V_REG	3.3
#define GET_INPUT_VOLTAGE(adc_val)     ((V_REG / 4095.0) * (float)adc_val * ((VIN_R1 + VIN_R2) / VIN_R2))
float Vesc::getVin() {
	processMessages();
	return GET_INPUT_VOLTAGE(_vin);
}
#define NTC_RES_GND(adc_val)    (10000.0*adc_val/4095.0)/(1-adc_val/4095.0)
#define NTC_RES(adc_val)        ((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_val)       (1.0 / ((log(NTC_RES(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15) // use when ntc is connected to vcc
#define NTC_TEMP_GND(adc_val)   (1.0 / ((log(NTC_RES_GND(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15) // use when ntc is connected to ground
float Vesc::getTempMotor() {
	processMessages();
	return NTC_TEMP_GND(_tempMotor);
}
float Vesc::getTempPCB() {
	processMessages();
	return NTC_TEMP(_tempPCB);
}
Vesc::mc_fault_code Vesc::getFaultCode() {
	processMessages();
	return _fault_code;
}
Vesc::mc_state Vesc::getState() {
	processMessages();
	return _state;
}
