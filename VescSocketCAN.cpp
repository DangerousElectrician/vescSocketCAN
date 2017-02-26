#include "VescSocketCAN.h"
#include <byteswap.h>

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

void Vesc::setPoint(CAN_PACKET_ID mode, int setpoint) {
	struct can_frame frame;
	frame.can_id = mode << 8 | _controllerID | 0x80000000;
	VESC_set set;
	set.set = setpoint;
	frame.can_dlc = sizeof(VESC_set);
	memcpy(frame.data, &set, sizeof(VESC_set));

	write(s, &frame, sizeof(struct can_frame));
}

void Vesc::setDuty(float dutyCycle) {
	setPoint(CAN_PACKET_SET_DUTY, dutyCycle*100000);
}
void Vesc::setCurrent(float current) {
	setPoint(CAN_PACKET_SET_CURRENT, current*1000);
}
void Vesc::setCurrentBrake(float current) {
	setPoint(CAN_PACKET_SET_CURRENT, current*1000);
}
void Vesc::setRpm(float rpm) {
	setPoint(CAN_PACKET_SET_CURRENT, rpm);
}
void Vesc::setPos(float pos) {
	setPoint(CAN_PACKET_SET_CURRENT, pos*1000000);
}

void Vesc::processMessages() {
	struct can_frame msg;
	while(1) {
		int a = read(s, &msg, sizeof(msg));
		if(a == -1) break;
		VESC_status status;
		switch( (msg.can_id & ~0x80000000 & ~_controllerID) >> 8) {
			case CAN_PACKET_STATUS:
				 status = * (VESC_status*) msg.data; // casting pointers!
				// received data is big endian
				_rpm = __bswap_32(status.rpm);
				_current = ((int16_t) __bswap_16(status.current)) / 10.0; 
				_duty_cycle = ((int16_t) __bswap_16(status.duty_cycle)) / 1000.0;
				break;
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
