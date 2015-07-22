#include "MePort.h"

// #if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

// MePort_Sig mePort[11] = {{NC, NC}, {11, A8}, {13, A11}, {10, 9}, {1, 0},
//     {MISO, SCK}, {A0, A1}, {A2, A3}, {A4, A5}, {6, 7}, {5, 4}
// };
// #else // else ATmega328
// MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
//     {NC, NC}, {A2, A3}, {A6, A1}, {A7, A0}, {6, 7}, {5, 4}
// };

// #endif

// #define _usemCore



/*        Port       */
MePort::MePort(){
    s1 = mePort[0].s1;
    s2 = mePort[0].s2;
    _port = 0;
}
MePort::MePort(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
MePort::MePort(uint8_t port,uint8_t slot)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
uint8_t MePort::getPort(){
	return _port;
}
uint8_t MePort::getSlot(){
	return _slot;
}
bool MePort::dRead1()
{
    bool val;
    pinMode(s1, INPUT);
    val = digitalRead(s1);
    return val;
}

bool MePort::dRead2()
{
    bool val;
    pinMode(s2, INPUT);
    val = digitalRead(s2);
    return val;
}

void MePort::dWrite1(bool value)
{
    pinMode(s1, OUTPUT);
    digitalWrite(s1, value);
}

void MePort::dWrite2(bool value)
{
    pinMode(s2, OUTPUT);
    digitalWrite(s2, value);
}

int MePort::aRead1()
{
    int val;
    val = analogRead(s1);
    return val;
}

int MePort::aRead2()
{
    int val;
    val = analogRead(s2);
    return val;
}

void MePort::aWrite1(int value)
{   
    analogWrite(s1, value);  
}

void MePort::aWrite2(int value)
{
    analogWrite(s2, value); 
}
void MePort::reset(uint8_t port){
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
void MePort::reset(uint8_t port,uint8_t slot){
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
uint8_t MePort::pin1(){
    return s1;
}
uint8_t MePort::pin2(){
    return s2;
}

uint8_t MePort::pin(){
    return _slot==SLOT1?s1:s2;
}
uint8_t MePort::pin(uint8_t port,uint8_t slot){
    return slot==SLOT1?mePort[port].s1:mePort[port].s2;
}


/* I2C */
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;
void i2c_init(void) {
  // #if defined(INTERNAL_I2C_PULLUPS)
  //   I2C_PULLUPS_ENABLE
  // #else
  //   I2C_PULLUPS_DISABLE
  // #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / 1000000) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void waitTransmissionI2C() {
  uint16_t count = 512; // change to 512 for lego encoder motor, the timer may overflow when rep-start
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      Serial.println("i2cerr");
      break;
    }
  }
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  // while(!(TWCR & (1<<TWINT)));                // <- can produce a blocking state with some WMP clones
  // TWCR = (1 << TWINT) | (1 << TWEN);
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}
int8_t i2c_readBit(uint8_t add, uint8_t reg, uint8_t bitNum) {
    uint8_t b;
    b = i2c_readReg(add, reg);
    return b & (1 << bitNum);
}
int8_t i2c_readBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length) {
    uint8_t b;
    b = i2c_readReg(dev, reg);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    return b;
}
void i2c_writeBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length,uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    b = i2c_readReg(dev, reg);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    i2c_writeReg(dev, reg, b);
}

//Wire Setup
#define BEGIN_FLAG          0x1E
#define BEGIN_STATE         0x91

/*             Wire               */
MeWire::MeWire(uint8_t address): MePort()
{
    _slaveAddress = address + 1;
}
MeWire::MeWire(MEPORT port, uint8_t address): MePort(port)
{
    _slaveAddress = address + 1;
}
void MeWire::begin()
{
    delay(1000);
    Wire.begin();
    write(BEGIN_FLAG, 0x01);
}
bool MeWire::isRunning()
{
    return read(BEGIN_STATE);
}
void MeWire::setI2CBaseAddress(uint8_t baseAddress)
{
    byte w[2] = {0};
    byte r[4] = {0};
    w[0] = 0x21;
    w[1] = baseAddress;
    request(w, r, 2, 4);
}

byte MeWire::read(byte dataAddress)
{
    byte *b = {0};
    read(dataAddress, b, 1);
    return b[0];
}

void MeWire::read(byte dataAddress, uint8_t *buf, int len)
{
    byte rxByte;
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    Wire.endTransmission(); // stop transmitting
    // delayMicroseconds(1);
    delay(100);
    Wire.requestFrom(_slaveAddress, len); // request 6 bytes from slave device
    int index = 0;
    while(Wire.available()) // slave may send less than requested
    {
        rxByte = Wire.read(); // receive a byte as character
        buf[index] = rxByte;
        index++;
    }
}

void MeWire::write(byte dataAddress, byte data)
{
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    // Wire.endTransmission(); // stop transmitting

    // Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(data); // sends one byte
    Wire.endTransmission(); // stop transmitting
}
void MeWire::request(byte *writeData, byte *readData, int wlen, int rlen)
{

    uint8_t rxByte;
    uint8_t index = 0;
    delay(10);
    Wire.beginTransmission(_slaveAddress); // transmit to device

    Wire.write(writeData, wlen);

    Wire.endTransmission();//delay(10);
    delayMicroseconds(2);
    Wire.requestFrom(_slaveAddress, rlen); // request 6 bytes from slave device
    // delayMicroseconds(2);
    while(Wire.available()) // slave may send less than requested
    {
        rxByte = Wire.read(); // receive a byte as character

        readData[index] = rxByte;
        index++;
    }
}

//  function:       pack data into a package to send
//  param:  buf     buffer to save package
//          bufSize size of buf
//          module  the associated module of package
//          data    the data to pack
//          length  the length(size) of data
//  return: 0       error
//          other   package size
uint32_t MeHost_Pack(uint8_t * buf,
                     uint32_t bufSize, 
                     uint8_t module, 
                     uint8_t * data, 
                     uint32_t length)
{
    uint32_t i = 0;

    //  head: 0xA5
    buf[i++] = 0xA5;
    buf[i++] = module;
    //  pack length
    buf[i++] = *((uint8_t *)&length + 0);
    buf[i++] = *((uint8_t *)&length + 1);
    buf[i++] = *((uint8_t *)&length + 2);
    buf[i++] = *((uint8_t *)&length + 3);
    //  pack data
    for(uint32_t j = 0; j < length; ++j)
    {
        buf[i++] = data[j];
    }

    //  calculate the LRC
    uint8_t check = 0x00;
    for(uint32_t j = 0; j < length; ++j)
    {
        check ^= data[j];
    }
    buf[i++] = check;

    //  tail: 0x5A
    buf[i++] = 0x5A;

    if (i > bufSize)
    {
        return 0;
    }
    else
    {
        return i;
    }
}

#define BUF_SIZE            256
#define MASK                255

class MeHost_Parser
{
public:
    MeHost_Parser();
    ~MeHost_Parser();

    //  push data to buffer
    uint8_t PushStr(uint8_t * str, uint32_t length);
    uint8_t PushByte(uint8_t ch);
    //  run state machine
    uint8_t Run();
    //  get the package ready state
    uint8_t PackageReady();
    //  copy data to user's buffer
    uint8_t GetData(uint8_t *buf, uint32_t size);

    void Print(char *str, uint32_t * cnt);
private:
    int state;
    uint8_t buffer[BUF_SIZE];
    uint32_t in;
    uint32_t out;
    uint8_t packageReady;

    uint8_t module;
    uint32_t length;
    uint8_t *data;
    uint8_t check;

    uint32_t lengthRead;
    uint32_t currentDataPos;

    uint8_t GetByte(uint8_t * ch);
};


#define HEAD    0xA5
#define TAIL    0x5A

//  states
#define ST_WAIT_4_START     0x01
#define ST_HEAD_READ        0x02
#define ST_MODULE_READ      0x03
#define ST_LENGTH_READ      0x04
#define ST_DATA_READ        0x05
#define ST_CHECK_READ       0x06

MeHost_Parser::MeHost_Parser()
{
    state = ST_WAIT_4_START;
    in = 0;
    out = 0;
    packageReady = 0;

    module = 0;
    length = 0;
    data = NULL;
    check = 0;

    lengthRead = 0;
    currentDataPos = 0;
}

MeHost_Parser::~MeHost_Parser()
{
    ;
}

uint8_t MeHost_Parser::PackageReady()
{
    return (1 == packageReady);
}

uint8_t MeHost_Parser::PushStr(uint8_t * str, uint32_t length)
{
    if (length > ((in + BUF_SIZE - out - 1) & MASK))
    {
        return 0;
    }
    else
    {
        for (int i = 0; i < length; ++i)
        {
            PushByte(str[i]);
        }
    }
}

uint8_t MeHost_Parser::PushByte(uint8_t ch)
{
    if (((in + 1) & MASK) != out)
    {
        buffer[in] = ch;
        ++in;
        in &= MASK;
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t MeHost_Parser::GetByte(uint8_t * ch)
{
    if (in != out)
    {
        *ch = buffer[out];
        ++out;
        out &= MASK;
        return 1;
    }
    else
    {
        // Serial.println("GET error!");
        return 0;
    }
}

uint8_t CalculateLRC(uint8_t *data, uint32_t length)
{
    uint8_t LRC = 0;
    for (uint32_t i = 0; i < length; ++i)
    {
        LRC ^= data[i];
    }
    return LRC;
}

uint8_t MeHost_Parser::Run(void)
{
    uint8_t ch = 0;
    while (GetByte(&ch))
    {
        switch (state)
        {
        case ST_WAIT_4_START:
            if (HEAD == ch)
            {
                state = ST_HEAD_READ;
            }
            break;
        case ST_HEAD_READ:
            module = ch;
            state = ST_MODULE_READ;
            break;
        case ST_MODULE_READ:
            //  read 4 bytes as "length"
            *(((uint8_t *)&length) + lengthRead) = ch;
            ++lengthRead;
            if (4 == lengthRead)
            {
                lengthRead = 0;
                state = ST_LENGTH_READ;
            }
            break;
        case ST_LENGTH_READ:
            //  alloc space for data
            if (0 == currentDataPos)
            {
                if (length > 255)
                {
                    state = ST_WAIT_4_START;
                    currentDataPos = 0;
                    lengthRead = 0;
                    length = 0;
                    module = 0;
                    check = 0;
                    break;
                }
                data = (uint8_t *)malloc(length + 1);
                if (NULL == data)
                {
                    state = ST_WAIT_4_START;
                    currentDataPos = 0;
                    lengthRead = 0;
                    length = 0;
                    module = 0;
                    check = 0;
                    break;
                }
            }
            //  read data
            data[currentDataPos] = ch;
            ++currentDataPos;
            if (currentDataPos == length)
            {
                currentDataPos = 0;
                state = ST_DATA_READ;
            }
            break;
        case ST_DATA_READ:
            check = ch;
            if (check != CalculateLRC(data, length))
            {
                state = ST_WAIT_4_START;
                if (NULL != data)
                {
                    free(data);
                    data = NULL;
                }
                currentDataPos = 0;
                lengthRead = 0;
                length = 0;
                module = 0;
                check = 0;
            }
            else
            {
                state = ST_CHECK_READ;
            }
            break;
        case ST_CHECK_READ:
            if (TAIL != ch)
            {
                if (NULL != data)
                {
                    free(data);
                    data = NULL;
                }
                length = 0;
            }
            else
            {
                packageReady = 1;
            }
            state = ST_WAIT_4_START;
            currentDataPos = 0;
            lengthRead = 0;
            module = 0;
            check = 0;
            break;
        default:
            break;
        }
    }
    return state;
}



uint8_t MeHost_Parser::GetData(uint8_t *buf, uint32_t size)
{
    int copySize = (size > length) ? length : size;
    if ((NULL != data) && (NULL != buf))
    {
        memcpy(buf, data, copySize);
        free(data);
        data = NULL;
        length = 0;
        packageReady = 0;

        return copySize;
    }
    else
    {
        return 0;
    }
}

//  frame type
#define ENCODER_MOTOR_GET_PARAM     0x01
#define ENCODER_MOTOR_SAVE_PARAM    0x02
#define ENCODER_MOTOR_TEST_PARAM    0x03
#define ENCODER_MOTOR_SHOW_PARAM    0x04
#define ENCODER_MOTOR_RUN_STOP      0x05
#define ENCODER_MOTOR_GET_DIFF_POS  0x06
#define ENCODER_MOTOR_RESET         0x07
#define ENCODER_MOTOR_SPEED_TIME    0x08
#define ENCODER_MOTOR_GET_SPEED     0x09
#define ENCODER_MOTOR_GET_POS       0x10
#define ENCODER_MOTOR_MOVE          0x11
#define ENCODER_MOTOR_MOVE_TO       0x12
#define ENCODER_MOTOR_DEBUG_STR     0xCC
#define ENCODER_MOTOR_ACKNOWLEDGE   0xFF

MeHost_Parser encoderParser = MeHost_Parser();

/*          EncoderMotor        */
MeEncoderMotor::MeEncoderMotor(uint8_t addr,uint8_t slot):MeWire(addr - 1)
{
    _slot = slot - 1;
}

void MeEncoderMotor::begin()
{
    MeWire::begin();
    Reset();
}

boolean MeEncoderMotor::Reset()
{
    uint8_t w[10] = {0};
    uint8_t r[10] = {0};

    uint8_t data[2] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_RESET;

    MeHost_Pack(w, 10, 0x01, data, 2);
    request(w, r, 10, 10);
    encoderParser.PushStr(r, 10);

    uint8_t ack[2] = {0};
    encoderParser.GetData(ack, 2);
    return ack[1];
}

boolean MeEncoderMotor::MoveTo(float angle, float speed)
{
    uint8_t w[18] = {0};
    uint8_t r[10] = {0};

    uint8_t data[10] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_MOVE_TO;
    *((float *)(data + 2)) = angle;
    *((float *)(data + 6)) = speed;

    MeHost_Pack(w, 18, 0x01, data, 10);
    request(w, r, 18, 10);
    encoderParser.PushStr(r, 10);
    encoderParser.Run();

    uint8_t ack[2] = {0};
    encoderParser.GetData(ack, 2);
    return ack[1];
}

boolean MeEncoderMotor::Move(float angle, float speed)
{
    uint8_t w[18] = {0};
    uint8_t r[10] = {0};

    uint8_t data[10] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_MOVE;
    *((float *)(data + 2)) = angle;
    *((float *)(data + 6)) = speed;

    MeHost_Pack(w, 18, 0x01, data, 10);
    request(w, r, 18, 10);
    encoderParser.PushStr(r, 10);
    encoderParser.Run();

    uint8_t ack[2] = {0};
    encoderParser.GetData(ack, 2);
    return ack[1];
}

boolean MeEncoderMotor::RunTurns(float turns, float speed)
{
    return Move(turns * 360, speed);
}

boolean MeEncoderMotor::RunSpeed(float speed)
{
    uint8_t w[14] = {0};
    uint8_t r[10] = {0};

    uint8_t data[6] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_RUN_STOP;
    *((float *)(data + 2)) = speed;

    MeHost_Pack(w, 14, 0x01, data, 6);
    request(w, r, 14, 10);
    encoderParser.PushStr(r, 10);
    encoderParser.Run();

    // uint8_t ack[2] = {0};
    // encoderParser.GetData(ack, 2);
    // return ack[1];
    return 0;
}

boolean MeEncoderMotor::RunSpeedAndTime(float speed, float time)
{
    uint8_t w[18] = {0};
    uint8_t r[10] = {0};

    uint8_t data[10] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_SPEED_TIME;
    *((float *)(data + 2)) = speed;
    *((float *)(data + 6)) = time;

    MeHost_Pack(w, 18, 0x01, data, 10);
    request(w, r, 18, 10);
    encoderParser.PushStr(r, 10);
    encoderParser.Run();

    // uint8_t ack[2] = {0};
    // encoderParser.GetData(ack, 2);
    // return ack[1];
    return 0;
}

float MeEncoderMotor::GetCurrentSpeed()
{
    uint8_t w[10] = {0};
    uint8_t r[14] = {0};

    uint8_t data[2] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_GET_SPEED;

    MeHost_Pack(w, 10, 0x01, data, 2);
    request(w, r, 10, 14);
    encoderParser.PushStr(r, 14);
    encoderParser.Run();

    uint8_t temp[6] = {0};
    encoderParser.GetData(temp, 6);
    float speed = *((float *)(temp + 2));
    return speed;
}

float MeEncoderMotor::GetCurrentPosition()
{
    uint8_t w[10] = {0};
    uint8_t r[14] = {0};

    uint8_t data[2] = {0};
    data[0] = _slot;
    data[1] = ENCODER_MOTOR_GET_POS;

    MeHost_Pack(w, 10, 0x01, data, 2);
    request(w, r, 10, 14);
    encoderParser.PushStr(r, 14);

    encoderParser.Run();

    uint8_t temp[6] = {0};
    uint8_t size = encoderParser.GetData(temp, 6);
    float pos = *((float *)(temp + 2));
    return pos;
}
