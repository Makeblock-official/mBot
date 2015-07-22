///@file Makeblock.h head file of Makeblock Library V2.1.0625
///Define the interface of Makeblock Library

#ifndef MeBaseBoard_h
#define MeBaseBoard_h

#include "MePort.h"
#include "MeDCMotor.h"
// #include "MeBuzzer.h"
#include "MeTemperature.h"
#include "Me7SegmentDisplay.h"
#include "MeRGBLed.h"
#include "MeUltrasonic.h"
#include "MeInfraredReceiver.h"
#include "MeIR.h"

Board_type MakeblockBoard = MeBaseBoard;

MePort_Sig mePort[11] = {{NC, NC}, {11, A8}, {13, A11}, {A10, A9}, {1, 0},
    {MISO, SCK}, {A0, A1}, {A2, A3}, {A4, A5}, {6, 7}, {5, 4}
};

#endif
