#include "MeDCMotor.h"
// MeDCMotor::MeDCMotor(): MePort(0)
// {

// }
MeDCMotor::MeDCMotor(MEPORT port): MePort(port)
{
    _pwmPin = s1;
    _dirPin = s2;   
    pinMode(_dirPin,OUTPUT);
}

MeDCMotor::MeDCMotor(uint8_t pwmPin,uint8_t dirPin)
{  
    _pwmPin = pwmPin;
    _dirPin = dirPin;
    pinMode(_dirPin,OUTPUT);
}
void MeDCMotor::run(int speed)
{
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;
    // constrain(speed,-255,255);
    if(speed >= 0) {
        digitalWrite(_dirPin,HIGH);
        analogWrite(_pwmPin,speed);
        // MePort::dWrite2(HIGH);
        // MePort::aWrite1(speed);
    } else {
        digitalWrite(_dirPin,LOW);
        analogWrite(_pwmPin,-speed);
        // MePort::dWrite2(LOW);
        // MePort::aWrite1(-speed);
    }
}
void MeDCMotor::stop()
{
    MeDCMotor::run(0);
}
