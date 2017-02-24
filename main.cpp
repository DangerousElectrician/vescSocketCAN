#include "VescSocketCAN.h"
#define VESC_ID 0
#define CAN_INTERFACE "can0"

int main(int argc, char **argv) {
	Vesc motor((char*) "can0", VESC_ID); //TODO: this char* thing is a bit iffy, is this the right way to do it
	motor.setCurrent(0.1);
}
