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

struct can_msg {
	struct bcm_msg_head msg_head;
	struct can_frame frame[1];
} msg;
void Vesc::setPoint(CAN_PACKET_ID mode, int setpoint) {
	VESC_set set;
	set.set = setpoint;
	//struct can_frame frame;
	//frame.can_id = mode << 8 | _controllerID | 0x80000000;
	//frame.can_dlc = sizeof(VESC_set);
	//memcpy(frame.data, &set, sizeof(VESC_set));

	//write(s, &frame, sizeof(struct can_frame));


	msg.msg_head.opcode  = TX_SETUP;
	msg.msg_head.can_id  = mode << 8 | _controllerID | 0x80000000;
	msg.msg_head.flags   = SETTIMER|STARTTIMER|TX_CP_CAN_ID;
	msg.msg_head.nframes = 1;
	msg.msg_head.count = 0;
	msg.msg_head.ival1.tv_sec = 0;
	msg.msg_head.ival1.tv_usec = 0;
	msg.msg_head.ival2.tv_sec = 0;
	msg.msg_head.ival2.tv_usec = 1000*10;
	//msg.frame[0].can_id    = 0x42; /* obsolete when using TX_CP_CAN_ID */
	msg.frame[0].can_dlc   = sizeof(VESC_set);
	memcpy(msg.frame[0].data, &set, 8);
	write(sbcm, &msg, sizeof(msg));

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
		switch( (msg.can_id & ~0x80000000 & ~_controllerID) >> 8) {
			case CAN_PACKET_STATUS:
				// received data is big endian
				_rpm = __bswap_32((*(VESC_status*) msg.data).rpm); // pointer casting!
				_current = ((int16_t) __bswap_16((*(VESC_status*) msg.data).current)) / 10.0; 
				_duty_cycle = ((int16_t) __bswap_16((*(VESC_status*) msg.data).duty_cycle)) / 1000.0;
				break;
			case CAN_PACKET_STATUS1:
				_rpm = (*(VESC_status1*) msg.data).rpm;
				_current = (*(VESC_status1*) msg.data).motorCurrent / 10.0;
				_position = (*(VESC_status1*) msg.data).position / 1000.0;
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
