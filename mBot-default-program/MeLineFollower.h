
#ifndef MELINEFOLLOWER_H_
#define MELINEFOLLOWER_H_
#include "MePort.h"

//states of two lineFollower sensors

#define S1_IN_S2_IN         0x00 //sensor1 and sensor2 are both inside of black line
#define S1_IN_S2_OUT        0x01 //sensor1 is inside of black line and sensor is outside of black line
#define S1_OUT_S2_IN        0x02 //sensor1 is outside of black line and sensor is inside of black line 
#define S1_OUT_S2_OUT       0x03 //sensor1 is outside of black line and sensor is outside of black line
///@brief Class for Line Follower Module
class MeLineFollower: public MePort
{
public:
    ///@brief initialize
	MeLineFollower(uint8_t pin1,uint8_t pin2);
    MeLineFollower(MEPORT port);
    ///@brief state of all sensors
    ///@return state of sensors
    uint8_t readSensors();
    ///@brief state of left sensors
    bool readSensor1();
    ///@brief state of right sensors
    bool readSensor2();
private:
	uint8_t _pin1;
	uint8_t _pin2;
};
#endif
