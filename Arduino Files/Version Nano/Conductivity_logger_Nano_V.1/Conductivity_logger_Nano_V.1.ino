
// DF robot conductivity logger;  Version 1 made on 5/2024 by A.Rok

// This is for use with the Arduino Nano and DS3231 RTC. 
// This program is intended for the use with the DF robot conductivity sensor, and temperature sensor. 
 
// All software and hardware are open source. Feel free to edit or change anything; I'm sure there are better ways. 

// LIBRARIES
#include <SPI.h>  
#include <SD.h>
#include "RTClib.h"
#include <avr/sleep.h>
#include <OneWire.h>  // for temperature sensor

#include "DFRobot_EC10.h"  // for the DF robot conductivity sensor
#include <EEPROM.h>  // for the DF robot conductivity sensor


#define interruptPin 2 //Pin we are going to use to wake up the Arduino
DFRobot_EC10 ec;









//------------------------------------------------------ VARIABLES ----------------------------------------------------------------

const int time_interval = 1;// THIS SETS THE SAMPLING INTERVAL IN MINUTES!!!!!!!---------------------------------


//---------------------------------------------------------------------------------------------------------------------------------







// storage of sample values

int buf1[10]; // temporary storage of samples for conductivity
int buf2[10];
float conduc_raw; // storage of calculate conductivity
float conduc;
float volt_conduc; // storage of the raw voltage from conductivity
float temp;
int power_pin = 8;
int power_pinT = 5;

int DS18S20_Pin = 6; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2



// Arduino nano additional parameters
RTC_DS3231 rtc; //for real time clock
File mydata; // what you are writing your data too
int chipSelect = 10; // pin of SD card reader
#define DS3231_ADDRESS     0x68   // defining the I2C address of the RTC module. 

//----------------------------------------------------SETUP-----------------------------------------------------------------------

void setup() {
Serial.begin(9600);  //turn on serial port, baud rate is 9600 in this case
pinMode(power_pin,OUTPUT);
pinMode(power_pinT,OUTPUT);

  Serial.print("Initializing SD card..."); // Check to see if SD reader is working and communicating
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");  // if this fails the code loop will stop here. Check for connection issue or power issues with SD reader
    while (1);
  }
  Serial.println("initialization done.");

  if (! rtc.begin()) {                      // Begin communication with RTC
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  rtc.disable32K(); // dont need the 32k pin


  //All of the code below is for the RTC system

    if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

//Set alarms for system:
 pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, FALLING);  // sets the interrrupt pin to wake the system up from sleep
    
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);
    
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);
    
    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(0,0,time_interval,0),  // time span is (Days, Hours, MInutes, Seconds). MOve your Time_interval to appropriate location
            DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set");  
    }


delay(50);
digitalWrite(power_pin, LOW);
digitalWrite(power_pinT,LOW);

Serial.println("Power cycle");// will print when the system cycles power or resets the alarm

}

//-------------------------------------------------------------------------INITIAL LOOP-------------------------------------------------------------
void loop() {

delay(2000); //wait 2 seconds before going to sleep
Going_to_sleep(); // goes to sleep

}

//-----------------------------------------------------------------------SLEEP FUNCTION-------------------------------------------------------------
// function for putting sensor to sleep and resetting the alarms

void Going_to_sleep(){
sleep_enable();
attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp,LOW);//attach an interrupt to pin d2
set_sleep_mode(SLEEP_MODE_PWR_DOWN);// set for full sleep
sleep_cpu();// activating sleep mode
sample_data(); // This is the fuction that will power on and record data from your sensors
// clears the alarm
if(rtc.alarmFired(1)) {  
        rtc.clearAlarm(1);
        Serial.println("Alarm cleared");
    }
// reset the alarm
if(!rtc.setAlarm1( 
            rtc.now() + TimeSpan(0,0,time_interval,0),  // remember to move your time_interval as needed
            DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set");  
    }

}

//-------------------------------------------------------------WAKE UP FUNCTION--------------------------------------------------------------------
// wakes up the system
void wakeUp(){
  Serial.println("Interrrupt Fired");//Print message to serial monitor
   sleep_disable();//Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(interruptPin)); //Removes the interrupt from pin 2;
 
}

//------------------------------------------------------------SAMPLE DATA FUNCTION---------------------------------------------------------------
// takes sample from sensors and prints to the serial port
void sample_data(){  
delay(50);

digitalWrite(power_pin, HIGH);
digitalWrite(power_pinT, HIGH);
// wait 1 second for conductivity to power on 
delay(2000);



// loop through 10 samples with .1 seconds wait between.
for(int i=0;i<10;i++)
{    
    volt_conduc = analogRead(A1); // Read and store 10 measurement of CO2 sensor
    buf1[i] = volt_conduc;
    buf2[i]= getTemp();
    delay(200); // wait .1 sec between each measurment. Overall sampling interaval takes 5 seconds here
}

// recording only the last taken value. But you can record more values if you want a time average or wish to trouble shoot the sensor.
temp = buf2[9];
conduc_raw = buf1[9]/1024.0*5000; // voltage reading
conduc = ec.readEC(conduc_raw,temp);  // calculated conductance



delay(100);
digitalWrite(power_pin, LOW);
digitalWrite(power_pinT,LOW);
delay(100);

write_file(conduc_raw, conduc,temp);//sends data to write file

DateTime now = rtc.now(); 

Serial.print(now.month(), DEC);  // print the date, time, and measurmenets to the serial port
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(' ');
    // Serial.print("ppm1:"); 
    // Serial.print(ppm1);  
    // Serial.print("; ppm2:");
    // Serial.print(ppm2);
    Serial.print("; volts:");
    Serial.print(conduc_raw);
    Serial.print(";  ms/cm:");
    Serial.print(conduc);
    Serial.print(";  DegreeC:");
    Serial.print(temp);
    Serial.println(";");
 

}

//---------------------------------------------------------WRITE FILE FUNCTION-------------------------------------------------------------------
// writes the data to the SD card
void write_file(float conduc_raw,float conduc,float temp){

  DateTime now = rtc.now(); 

  mydata = SD.open("datalog.txt", FILE_WRITE);  // opens file to write to on SD card
  
   // if the file is available, write to it:
  if (mydata) {
    mydata.print(now.month(), DEC);
    mydata.print('/');
    mydata.print(now.day(), DEC);
    mydata.print('/');
    mydata.print(now.year(), DEC);
    mydata.print(';');
    mydata.print(now.hour(), DEC);
    mydata.print(':');
    mydata.print(now.minute(), DEC);
    mydata.print(':');
    mydata.print(now.second(), DEC);
    mydata.print(';');
    // mydata.print(ppm1); 
    // mydata.print(';');
    // mydata.print(ppm2);
    // mydata.print(';');
    mydata.print(conduc_raw);
    mydata.print(conduc);
    mydata.println(temp);
      mydata.close();
   
  }
  
  }


//-------------------------------------------------------------------------------------------------------

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

}