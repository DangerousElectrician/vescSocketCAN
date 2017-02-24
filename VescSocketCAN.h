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
		struct ifreq ifr;
		struct sockaddr_can addr;
		int s;
		int sbcm;

		uint8_t _controllerID;

		typedef struct VESC_set {
			int set:32;
		} VESC_set;
		void init_socketCAN(char *ifname);
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
			CAN_PACKET_STATUS
		} CAN_PACKET_ID;
		Vesc(char *interface, uint8_t controllerID);
		void setPoint(CAN_PACKET_ID mode, int setpoint);
		void setDuty(float dutyCycle);
		void setCurrent(float current);
		void setCurrentBrake(float current);
		void setRpm(float rpm);
		void setPos(float pos);
};
