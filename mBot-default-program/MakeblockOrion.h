///@file Makeblock.h head file of Makeblock Library V2.1.0625
///Define the interface of Makeblock Library

#ifndef MakeblockOrion_h
#define MakeblockOrion_h

#include "MePort.h"
#include "MeDCMotor.h"
#include "MeBuzzer.h"
#include "MeTemperature.h"
#include "Me7SegmentDisplay.h"
#include "MeRGBLed.h"
#include "MeUltrasonic.h"
#include "MeInfraredReceiver.h"
#include "MeIR.h"
#include "MeLineFollower.h"
// #include "Wire.h"
Board_type MakeblockBoard = MakeblockOrion;

MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
    {1, 0}, {A2, A3}, {A6, A1}, {A7, A0}, {6, 7}, {5, 4}
};

///@brief Class for MeBoard
// class MeBoard
// {
// public:
//     MeBoard(uint8_t boards);
// };



#endif
