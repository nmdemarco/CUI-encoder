/*
  from https://forum.arduino.cc/index.php?topic=158790.15
*/

//include SPI library
#include <SPI.h>
//this is the serial baud rate for talking to the Arduino
#define baudRate 500000
//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point
//set the chip select pin for the AMT20
const int CS = 53;
double deg;
long distance; //to check if stepper has moved a step, accelstepper doesnt have this function while using .run();

#include <AccelStepper.h>
long positions[1]; // Array of desired stepper positions
AccelStepper stepper1(1, 54, 55);//(1 for A4988, Step pin, Dir pin), im using a 200 step stepper set to microstep size of 2

void setup()
{

  //  //Set I/O mode of all SPI pins.
  //  pinMode(SCK, OUTPUT);
  //  pinMode(MOSI, OUTPUT);
  //  pinMode(MISO, INPUT);
  //  pinMode(CS, OUTPUT);
  //Initialize SPI using the SPISettings(speedMaxium, dataOrder, dataAMode) function
  //For our settings we will use a clock rate of 500kHz, and the standard SPI settings
  //of MSB First and SPI Mode 0
  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  //Using SPI.beginTransaction seems to require explicitly setting the beginning state
  //of the CS pin as opposed to the SPI.begin() function that does this for us.
  //  digitalWrite(CS, HIGH);
  SPI.begin();
  SPIWrite(set_zero_point); //issue set 0 point command
  while (SPIWrite(nop) != 0x80)    //
  {
  }
  SPI.end(); //cause device to power cycle, needs to set new 0 position
  delay(100);


  // Configure each stepper
  stepper1.setEnablePin(38);
  stepper1.setPinsInverted(false, false, true); //invert logic of enable pin
  stepper1.enableOutputs();
  stepper1.setMaxSpeed(50);
  stepper1.setAcceleration(50);

  //Initialize the UART serial connection
  Serial.begin(baudRate);
  Serial.println("starting");
  delay(3000);

  positions[0] = 100;

}

void loop() //moves the motor 90 deg and back twice, reading encoder value after each step, then stops completely
{
  stepper1.move(positions[0]);
  //  long a = millis();
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    if ( distance != stepper1.distanceToGo() ) {
      distance = stepper1.distanceToGo();
      encoderBuff();
    }
  }
  Serial.println("buffer check begin");
  double oldDeg = deg;
  encoderBuff();
  //  dpr(deg);  dpr(oldDeg);
  while (oldDeg != deg)
  {
    oldDeg = deg;
    encoderBuff();
  }
  //  Serial.println(millis() - a); //check how much time it moved
  delay(1000);
  Serial.println("90 deg should be here");
  //  a = millis();
  stepper1.move(-positions[0]);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    if ( distance != stepper1.distanceToGo() ) {
      distance = stepper1.distanceToGo();
      encoderBuff();
    }
  }
  //  Serial.println(millis() - a); //check how much time it moved
  delay(1000);
  Serial.println("end");

  stepper1.move(positions[0]);
  //  long a = millis();
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    if ( distance != stepper1.distanceToGo() ) {
      distance = stepper1.distanceToGo();
      encoderBuff();
    }
  }
  Serial.println("buffer check begin");
  oldDeg = deg;
  encoderBuff();
  //  dpr(deg);  dpr(oldDeg);
  while (oldDeg != deg)
  {
    oldDeg = deg;
    encoderBuff();
  }
  //  Serial.println(millis() - a); //check how much time it moved
  delay(1000);
  Serial.println("90 deg should be here");
  //  a = millis();
  stepper1.move(-positions[0]);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    if ( distance != stepper1.distanceToGo() ) {
      distance = stepper1.distanceToGo();
      encoderBuff();
    }
  }
  //  Serial.println(millis() - a); //check how much time it moved
  delay(1000);
  Serial.println("end");

  while (1);
}



void encoderBuff() //order the encoder to start another position reading right after finishing the current reading
{
  uint8_t recieved;    //just a temp vairable
  uint16_t ABSposition;

  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  recieved = SPIWrite(nop);    //issue NOP to check if encoder is ready to send

  if (recieved == rd_pos)    //check if encoder is ready to send
  {
    //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
    //shift it left 8 bits to make room for the lower byte.
    ABSposition = (SPIWrite(nop) & 0x0F) << 8;

    //OR the next byte with the current position
    ABSposition |= SPIWrite(rd_pos);

    deg = ABSposition * 0.08789;    // aprox 360/4096
    Serial.println(deg, DEC); //current position in decimal
  }
  else
  {
    recieved = SPIWrite(rd_pos);    //cleck again if encoder is still working

    while (recieved != rd_pos)    //sometimes works very slowly
    {
      recieved = SPIWrite(nop);    //cleck again if encoder is still working
    }

    //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
    //shift it left 8 bits to make room for the lower byte.
    ABSposition = (SPIWrite(nop) & 0x0F) << 8;

    //OR the next byte with the current position
    ABSposition |= SPIWrite(rd_pos);

    deg = ABSposition * 0.08789;    // aprox 360/4096
    Serial.println(deg, DEC); //current position in decimal
  }
  SPI.end();

}

void encoderWithoutBuff() //works almost the same as the previous examples in the thread, without the encoder to read position at the end of each reading (so it wouldnt buffer it)
{
  // SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();

  uint8_t data;               //this will hold our returned data from the AMT20
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position

  //send the rd_pos command to have the AMT20 begin obtaining the current position
  data = SPIWrite(rd_pos);

  //we need to send nop commands while the encoder processes the current position. We
  //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
  //    while (data != rd_pos && timeoutCounter++ < timoutLimit)
  while (data != rd_pos)
  {
    data = SPIWrite(nop);
  }

  //We received the rd_pos echo which means the next two bytes are the current encoder position.
  //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.

  //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
  //shift it left 8 bits to make room for the lower byte.
  currentPosition = (SPIWrite(nop) & 0x0F) << 8;

  //OR the next byte with the current position
  currentPosition |= SPIWrite(nop);

  deg = currentPosition * 0.08789; // aprox 360/4096
  Serial.println(deg, DEC); //current position in decimal

  SPI.end();

}


//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(CS, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS, HIGH);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  delayMicroseconds(21);

  return data;
}
