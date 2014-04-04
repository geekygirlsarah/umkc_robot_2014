#include "runner.h"

#ifndef ARM_COMMANDS_H
#define ARM_COMMANDS_H
class ArmCommands{
	static int grasp(){return Runner::executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: grasp}'", "");
	static int release() {return Runner::executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: release}'", "");
	static int park() {return Runner::executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: park}'", "");}
	static int carry() {return Runner::executeBinary("rostopic pub -1 /arm/put/point wesley/arm_point '{direct_mode: false, cmd: carry}'", "");}
	static int giveup() {return executeBinary("rostopic pub -1 /arm/put/angle wesley/arm_angle 90 90 180 90 90 120", "");}
};
#endif

