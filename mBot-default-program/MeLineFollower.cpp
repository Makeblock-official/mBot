#include "MeLineFollower.h"
/*             LineFinder              */

MeLineFollower::MeLineFollower(uint8_t pin1,uint8_t pin2)
{
	_pin1 = pin1;
	_pin2 = pin2;
	pinMode(_pin1,INPUT);
	pinMode(_pin2,INPUT);
}

MeLineFollower::MeLineFollower(MEPORT port): MePort(port)
{
	_pin1 = s1;
    _pin2 = s2;
    pinMode(_pin1,INPUT);
	pinMode(_pin2,INPUT);
}
uint8_t MeLineFollower::readSensors()
{
    // uint8_t state = S1_IN_S2_IN;
    uint8_t state = digitalRead(_pin1);
    state = (state << 1) | digitalRead(_pin2);
    // state = ((1 & s1State) << 1) | s2State;
    return state;
}
bool MeLineFollower::readSensor1()
{
    return digitalRead(_pin1);
}
bool MeLineFollower::readSensor2()
{
    return digitalRead(_pin2);
}