#include "MeInfraredReceiver.h"
MeInfraredReceiver::MeInfraredReceiver():MePort(),SoftwareSerial(NC, NC){

}

MeInfraredReceiver::MeInfraredReceiver(MEPORT port):MePort(port),SoftwareSerial(mePort[port].s2, mePort[port].s1){
	// SoftwareSerial::begin(9600);
	_staPin = s1;
}

MeInfraredReceiver::MeInfraredReceiver(uint8_t staPin,uint8_t datPin):SoftwareSerial(datPin, staPin){
	// SoftwareSerial::begin(9600);
	_staPin = staPin;
    
	// pinMode(s2,INPUT);
}

int MeInfraredReceiver::available(){
	// pinMode(s1, INPUT);
	// pinMode(_staPin, INPUT);
	if(SoftwareSerial::available() || digitalRead(_staPin)==0)return 1;	
	return 0;	
}
		
bool MeInfraredReceiver::buttonState(){
  
  if(getPort()>0){
	return dRead1()==0;
  }
  return 0;
}

int MeInfraredReceiver::read()
{
    uint8_t val;
    uint16_t i;
    do
    {
        i++;
        if(++i > 2000)break;
        if(SoftwareSerial::available())
        val = SoftwareSerial::read();							//Read serial infrared data
        val &= 0xff;
    }
    while(val == 0x0 || val == 0xFF);	//0x0 and 0xff are the user code of BC7210A IC
    delayMicroseconds(10);
    return  val;

}

void MeInfraredReceiver::begin()
{
    SoftwareSerial::begin(9600);
    pinMode(_staPin, INPUT);
}