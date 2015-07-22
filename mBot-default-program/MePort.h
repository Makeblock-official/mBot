#ifndef MEPORT_H_
#define MEPORT_H_

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"
#ifndef F_CPU
#define  F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>
#include <SoftwareSerial.h>

typedef enum
{
    MeBaseBoard,
    MakeblockOrion,
    mCore,
    UNOShield
}Board_type;

extern Board_type MakeblockBoard;
typedef struct
{
    uint8_t s1;
    uint8_t s2;
} MePort_Sig;
extern MePort_Sig mePort[11];//mePort[0] is nonsense

#define NC 					-1

// MePort_Sig mePort[11]={{NC, NC}, {11, 12}, {9, 10}, {A2, A3}, {A0, A1},
//     {NC, NC}, {NC, NC}, {NC, NC}, {NC, NC}, {6, 7}, {5, 4}};

// #define PORT_1 				0x01
// #define PORT_2 				0x02
// #define PORT_3 				0x03
// #define PORT_4 				0x04
// #define PORT_5 				0x05
// #define PORT_6 				0x06
// #define PORT_7 				0x07
// #define PORT_8 				0x08
// #define M1     				0x09
// #define M2     				0x0a

typedef enum
{
    PORT_0,
    PORT_1,
    PORT_2,
    PORT_3,
    PORT_4,
    PORT_5,
    PORT_6,
    PORT_7,
    PORT_8,
    M1,
    M2,
}MEPORT;

// #if defined(__AVR_ATmega32U4__) 
// // buzzer 
// #define buzzerOn()  DDRE |= 0x04,PORTE |= B00000100
// #define buzzerOff() DDRE |= 0x04,PORTE &= B11111011
// #else
// #define buzzerOn()  DDRC |= 0x20,PORTC |= B00100000;
// #define buzzerOff() DDRC |= 0x20,PORTC &= B11011111;
// #endif
#define SLOT1 1
#define SLOT2 2
#define SLOT_1 SLOT1
#define SLOT_2 SLOT2

#define FALSE 0
#define TRUE  1

///@brief class of MePort,it contains two pin.
class MePort
{
public:
    MePort();
    ///@brief initialize the Port
    ///@param port port number of device
    MePort(uint8_t port);
    MePort(uint8_t port,uint8_t slot);
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    uint8_t getPort();
    uint8_t getSlot();
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead1();
    ///@return the level of pin 2 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead2();
    ///@brief set the analog value of pin 1 of port
    ///@param value is HIGH or LOW
    void dWrite1(bool value);
    ///@brief set the level of pin 1 of port
    ///@param value is HIGH or LOW
    void dWrite2(bool value);
    ///@return the analog signal of pin 1 of port between 0 to 1023
    int aRead1();
    ///@return the analog signal of pin 2 of port between 0 to 1023
    int aRead2();
    ///@brief set the PWM outpu value of pin 1 of port
    ///@param value between 0 to 255
    void aWrite1(int value);
    ///@brief set the PWM outpu value of pin 2 of port
    ///@param value between 0 to 255
    void aWrite2(int value);
    void reset(uint8_t port);
    void reset(uint8_t port,uint8_t slot);
    uint8_t pin1();
    uint8_t pin2();
    uint8_t pin();
    uint8_t pin(uint8_t port,uint8_t slot);
protected:
    uint8_t s1;
    uint8_t s2;
    uint8_t _port;
    uint8_t _slot;
};


//TWI
void i2c_init(void);
void waitTransmissionI2C();
void i2c_rep_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data );
uint8_t i2c_read(uint8_t ack);
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
int8_t i2c_readBit(uint8_t add, uint8_t reg, uint8_t bitNum);
int8_t i2c_readBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length);
void i2c_writeBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length,uint8_t data);

///@brief class of MeWire
class MeWire: public MePort
{
public:
    MeWire(uint8_t address);
    ///@brief initialize
    ///@param port port number of device
    MeWire(MEPORT port, uint8_t address);
    ///@brief reset start index of i2c slave address.
    void setI2CBaseAddress(uint8_t baseAddress);
    bool isRunning();
    ///@brief Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
    ///@param address the 7-bit slave address (optional); if not specified, join the bus as a master.
    void begin();
    ///@brief send one byte data request for read one byte from slave address.
    byte read(byte dataAddress);
    void read(byte dataAddress, uint8_t *buf, int len);
    ///@brief send one byte data request for write one byte to slave address.
    void write(byte dataAddress, byte data);
    void request(byte *writeData, byte *readData, int wlen, int rlen);
protected:
    int _slaveAddress;
};

///@brief Class for Encoder Motor Driver
class MeEncoderMotor: public MeWire{
    public:
        MeEncoderMotor(uint8_t addr,uint8_t slot);
        void begin();
        boolean Reset();
        boolean Move(float angle, float speed);
        boolean MoveTo(float angle, float speed);
        boolean RunTurns(float turns, float speed);
        boolean RunSpeed(float speed);
        boolean RunSpeedAndTime(float speed, float time);
        float GetCurrentSpeed();
        float GetCurrentPosition();
    private:
        uint8_t _slot;    
};
#endif
