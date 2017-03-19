#include "VescSocketCAN.h"
#include <iomanip>
#define VESC_ID 0
#define CAN_INTERFACE "can0"

int main(int argc, char **argv) {
	Vesc motor((char*) "can0", VESC_ID); //TODO: this char* thing is a bit iffy, is this the right way to do it
	motor.setCustom(0.1);
	//float current = 0;
	while(1) {
		std::cout << std::setprecision(4) << "mcur " << motor.getCurrent() << std::setw(9) << "\tduty " <<  motor.getDutyCycle() << std::setw(9) << "\trpm " << motor.getRpm() << std::setw(9) << "\tpos " <<motor.getPosition() << std::setw(9) << "\ttach " << motor.getTachometer() << std::setw(9) << "\twatth " << motor.getWattHours() << std::setw(9) << "\tincur " << motor.getInCurrent() << std::setw(9) << "\tVin " << motor.getVin() << std::setw(9) << "\ttempM " << motor.getTempMotor() << std::setw(9) << "\ttempP " << motor.getTempPCB() << std::setw(9) << "\tfault  " << motor.getFaultCode() << std::setw(9) << "\tstate " << motor.getState() << std::endl;
		//motor.getCurrent();
		//motor.setCurrent(current);
		//current = current + 0.01;
		//motor.setCustom(3.14159);
		usleep(50000);
	}
	//motor.setCurrent(1);
	//usleep(50000);
	////motor.disable();
	//motor.setCurrent(2);
	//usleep(50000);
	//motor.enable();
	//motor.setCurrent(3);
	//usleep(50000);
}
