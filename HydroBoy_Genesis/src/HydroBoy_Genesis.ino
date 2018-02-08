// Smart Hydroponics Monitoring System
// 2018.01.29
// Coded by Saeyoung Macx Kim (saeyoung0102@gmail.com)
// Read from 4 different commercial sensors (pH, DO, temp, eC) and publish to ThingSpeak web server

// libraries   ----------------------------------------------------------------------------------------------------------------
#include <LiquidCrystal_I2C_Spark.h>
#include <flashee-eeprom.h>
#include <OneWire.h>
#include <ThingSpeak/ThingSpeak.h>

// PH sensor initialization
/*
#define PHSensorPin A0          //pH meter Analog output to Arduino Analog Input 0
#define PHOffset 0.00           //deviation compensate
#define PHLED 13
#define PHsamplingInterval 20
#define PHprintInterval 800
#define PHArrayLenth  40        //times of collection


int pHArray[PHArrayLenth];      //Store the average value of the sensor feedback
int pHArrayIndex=0;
*/

const int PHSensorPin = A0;
int PHsensorValue = 0;
unsigned long int PHavgValue;
float b;
int PHbuf[10],PHtemp;

// DO sensor initialization   -------------------------------------------------------------------------------------------------
#include <avr/pgmspace.h>

#define DoSensorPin A1     //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 3300          //for arduino uno, the ADC reference is the AVCC, that is 5000 mV(TYP), 3300 mV for Particle Photon
float doValue;             //current dissolved oxygen value, unit: mg/L
float temperature = 25;    //default temperature is 25^C, you can use a temperature sensor to read it

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];    // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT 30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 12          //the address of the Saturation Oxygen voltage stored in the EEPROM
#define SaturationDoTemperatureAddress 16      //the address of the Saturation Oxygen temperature stored in the EEPROM
float SaturationDoVoltage,SaturationDoTemperature;
float averageDoVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

// EC sensor init    ----------------------------------------------------------------------------------------------------------
#define StartConvert 0
#define ReadTemperature 1

const byte numReadings = 20;     //the number of sample times
byte ECsensorPin = A3;  //EC Meter analog output,pin on analog 1
byte DS18B20_Pin = 2; //DS18B20 signal, pin on digital 2
unsigned int ECAnalogSampleInterval=25,ECprintInterval=700,ECtempSampleInterval=850;  //analog sample interval;serial print interval;temperature sample interval
unsigned int readings[numReadings];      // the readings from the analog input
byte ECindex = 0;                  // the ECindex of the current reading
unsigned long ECAnalogValueTotal = 0;                  // the running total
unsigned int ECAnalogAverage = 0,ECaverageVoltage=0;                // the average
unsigned long ECAnalogSampleTime,ECprintTime,ECtempSampleTime;
float ECcurrent;


// declare variables   --------------------------------------------------------------------------------------------------------

// LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display, if in doubt try 0x3F
LiquidCrystal_I2C lcd(0x27,20,4);  //

/*
 * unsigned long int avgPHval, avgTEMPval;
unsigned long PHvoltage, TEMPvoltage, ECvoltage;
float PHval, TEMPval;
*/

// define sensor pins (raw readings)   ----------------------------------------------------------------------------------------
// int PHsensor = A0;      // defined at pH sensor init section
// int DOsensor = A1;      // defined at DO sensor init section
int TEMPsensor = D2;    //DS18S20 signal pin on digital 2

// delay for entire loop
int loopDelay = 1000;

// define counter for sending data to ThingSpeak
int sendCount = 0;

// for temp sensor
OneWire ds(TEMPsensor);

// info for ThingSpeak connection
TCPClient client;

unsigned int myChannelNumber = 413558; // channel # for HydroBoy Genesis Sensor Stream
const char * myWriteAPIKey = "5H5SA6B45N14J3UF"; // write API key for HydroBoy Genesis Sensor Stream

void setup() {

    Serial.begin(115200);
    ThingSpeak.begin(client); // begin ThingSpeak client
    pinMode(D2, INPUT); // set D2 pinmode to INPUT

    lcd.init(); // initialize the 2004 lCD
    lcd.backlight();

    // DO setup    ----------------------------------------------------------------
    pinMode(DoSensorPin,INPUT);
    readDoCharacteristicValues();      //read Characteristic Values calibrated from the EEPROM

    // EC setup    ----------------------------------------------------------------
    for (byte thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }
    getTemp(StartConvert);   //let the DS18B20 start the convert
    ECAnalogSampleTime=millis();
    ECprintTime=millis();
    ECtempSampleTime=millis();

    // HydroBoy Bootup Display

    // Serial.begin(9600);
    Serial.println("HydroBoy Genesis Node");
    lcd.setCursor(0,0);
    lcd.print(F("HB Genesis Node"));
    delay(1000);

    Serial.println("by Kang and Kim");
    lcd.setCursor(0,1);
    lcd.print(F("by Kang and Kim"));
    delay(1000);

    lcd.setCursor(0,2);
    lcd.print(Time.format(Time.now(), "%T"));   // Format from C library: https://www.gnu.org/software/libc/manual/html_node/Low_002dLevel-Time-String-Parsing.html
    lcd.setCursor(0,3);
    lcd.print(Time.format(Time.now(), "%a %b %d,%Y"));  // Format from C library: https://www.gnu.org/software/libc/manual/html_node/Low_002dLevel-Time-String-Parsing.html
    delay(1000);

}

void loop() {

  temperature = getTemp(ReadTemperature);

// PH sample code    -----------------------------------------------------------------------------------------------------
/*
  static unsigned long PHsamplingTime = millis();
  static unsigned long PHprintTime = millis();
  static float pHValue,pHvoltage;

  if(millis()-PHsamplingTime > PHsamplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(PHSensorPin);
      if(pHArrayIndex==PHArrayLenth)pHArrayIndex=0;
      pHvoltage = avergearray(pHArray, PHArrayLenth)*3.3/4096;
      pHValue = 3.5*pHvoltage+PHOffset;
      PHsamplingTime=millis();
  }
  if(millis() - PHprintTime > PHprintInterval)   //Every 800 milliseconds, print a numerical, convert the state of the PHLED indicator
  {
    Serial.print("pH voltage: ");
         Serial.print(pHvoltage,2);
         Serial.print(" pH value: ");
    Serial.print(pHValue,2);
         Serial.print(" pH");
         digitalWrite(PHLED,digitalRead(PHLED)^1);
         PHprintTime=millis();
  }

*/

     for(int i=0;i<10;i++)
     {
      PHbuf[i]=analogRead(PHSensorPin);
      delay(10);
     }
     for(int i=0;i<9;i++)
     {
      for(int j=i+1;j<10;j++)
      {
       if(PHbuf[i]>PHbuf[j])
       {
        PHtemp=PHbuf[i];
        PHbuf[i]=PHbuf[j];
        PHbuf[j]=PHtemp;
       }
      }
     }
     PHavgValue=0;
     for(int i=2;i<8;i++)
     PHavgValue+=PHbuf[i];
     float pHvoltage=(float)PHavgValue*3.3/4096/6;
     float pHValue = -5.70 * pHvoltage + 21.00;

     Serial.print(F("pH Voltage:  "));
     Serial.print(pHvoltage,3);
     Serial.print(F("  pH Value:  "));
     Serial.println(pHValue,2);

// DO sample code   -----------------------------------------------------------------------------------------------------

   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }

   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
   {
      tempSampleTimepoint = millis();
      //temperature = readTemperature();  // add your temperature codes here to read the temperature, unit:^C
   }

   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageDoVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0; // read the value more stable by the median filtering algorithm
      Serial.print(F("Temperature: "));
      Serial.print(temperature,1);
      Serial.print(F("  C"));
      doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageDoVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
      Serial.print(F("  DO Value: "));
      Serial.print(doValue,2);
      Serial.println(F("  mg/L"));
   }

   if(serialDataAvailable() > 0)
   {
      byte modeIndex = uartParse();  //parse the uart command received
      doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
   }

// EC sample code   -----------------------------------------------------------------------------------------------------

  /*
   Every once in a while,sample the analog value and calculate the average.
  */

  if(millis()-ECAnalogSampleTime>=ECAnalogSampleInterval)
  {
    ECAnalogSampleTime=millis();
     // subtract the last reading:
    ECAnalogValueTotal = ECAnalogValueTotal - readings[ECindex];
    // read from the sensor:
    readings[ECindex] = analogRead(ECsensorPin);
    // add the reading to the total:
    ECAnalogValueTotal = ECAnalogValueTotal + readings[ECindex];
    // advance to the next position in the array:
    ECindex = ECindex + 1;
    // if we're at the end of the array...
    if (ECindex >= numReadings)
    // ...wrap around to the beginning:
    ECindex = 0;
    // calculate the average:
    ECAnalogAverage = ECAnalogValueTotal / numReadings;
  }

  /*
   Every once in a while, MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */

   if(millis()-ECtempSampleTime>=ECtempSampleInterval)
   {
    ECtempSampleTime=millis();
    temperature = getTemp(ReadTemperature);  // read the current temperature from the  DS18B20
    getTemp(StartConvert);                   //after the reading,start the convert for next reading
   }

   /*
   Every once in a while,print the information on the serial monitor.
  */

  if(millis()-ECprintTime>=ECprintInterval)
  {
    ECprintTime=millis();
    ECaverageVoltage=ECAnalogAverage*(float)3300/4096;
    Serial.print("EC Analog value: ");
    Serial.print(ECAnalogAverage);   //analog average,from 0 to 4095
    Serial.print("  EC Voltage: ");
    Serial.print(ECaverageVoltage);  //millivolt average,from 0mv to 3300mV
    Serial.print(" mV ");
    Serial.print(" EC temp: ");
    Serial.print(temperature);    //current temperature
    Serial.print(" C");
    Serial.print("  EC: ");

    float TempCoefficient=1.0+0.0185*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVoltage=(float)ECaverageVoltage/TempCoefficient;
    if(CoefficientVoltage<150)Serial.println("No solution!");   //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if(CoefficientVoltage>3300)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else
    {
      if(CoefficientVoltage<=448)ECcurrent=6.84*CoefficientVoltage-64.32;   //1ms/cm<EC<=3ms/cm
      else if(CoefficientVoltage<=1457)ECcurrent=6.98*CoefficientVoltage-127;  //3ms/cm<EC<=10ms/cm
      else ECcurrent=5.3*CoefficientVoltage+2278;                           //10ms/cm<EC<20ms/cm
      ECcurrent/=1000;    //convert us/cm to ms/cm
      Serial.print(ECcurrent,2);  //two decimal
      Serial.println("  mS/cm");
    }
    Serial.println("-------------------------------------------------------------------------");
  }


// write to 2004 LCD  -----------------------------------------------------------------------------------------------------------
    lcd.setCursor(0,0);
    lcd.print(F("  pH: "));
    lcd.print(pHValue, 2);
    lcd.print(F(" pH"));

    lcd.setCursor(0,1);
    lcd.print(F("  DO: "));
    lcd.print(doValue, 2);
    lcd.print(F(" mg/L"));

    lcd.setCursor(0,2);
    lcd.print(F("temp: "));
    lcd.print(temperature, 2);
    lcd.print(F(" C"));

    lcd.setCursor(0,3);
    lcd.print(F("  eC: "));
    lcd.print(ECcurrent, 2);
    lcd.print(F(" mS/cm"));

// send data to ThingSpeak channel every 10 sec --------------------------------------------------------------------------------------------
    sendCount++;
    if(sendCount > 10)
    {
        ThingSpeak.setField(1, pHValue);
        ThingSpeak.setField(2, doValue);
        ThingSpeak.setField(3, temperature);
        ThingSpeak.setField(4, ECcurrent);

        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        sendCount = 0;
    }

    delay(loopDelay);

}

// functions for PH sensor ------------------------------------------------------------------------------------------------------

double avergearray(int* arr, int number)
{
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

// functions for DO sensor ------------------------------------------------------------------------------------------------------

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 )
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
  receivedBufferIndex = 0;
  strupr(receivedBuffer);
  return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL)
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL)
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;

      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;

     case 2:
      if(enterCalibrationFlag)
      {
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageDoVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperature);
         SaturationDoVoltage = averageDoVoltage;
         SaturationDoTemperature = temperature;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)
               Serial.print(F(">>>Calibration Successful"));
            else
              Serial.print(F(">>>Calibration Failed"));
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
    for (i = 0; i < iFilterLen - j - 1; i++)
          {
      if (bTab[i] > bTab[i + 1])
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }
}

// functions for TEMP sensor ----------------------------------------------------------------------------------------------------

/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/

/*
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
*/

float getTemp(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds.reset_search();
              return 0;
          }
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{
          byte present = ds.reset();
          ds.select(addr);
          ds.write(0xBE); // Read Scratchpad
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }
          ds.reset_search();
          byte MSB = data[1];
          byte LSB = data[0];
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
    return TemperatureSum;
}
