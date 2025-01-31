
// DIY CTD / LTC logger system;  Version 1 made on 10/2024 by A.Rok

// df robot analog pressure sensor, analog condutivity sensor, digital temperature sensor

// This is for use with the Arduino 5v pro mini and DS3231 RTC. 
 

// All software and hardware are open source. Feel free to edit or change anything if you desire. 

// LIBRARIES
#include <SPI.h>  //include libraries for SD and RTC
#include <SD.h>
#include "RTClib.h"
#include <avr/sleep.h>
#include <OneWire.h>  // for temperature sensor
#include <DallasTemperature.h> // for temperature senosr

#include "DFRobot_EC.h"  // for the DF robot conductivity sensor
#include <EEPROM.h>  // for the DF robot conductivity sensor

# define temp_pin 3 // digital signal pin for temperature sensor
#define interruptPin 2 //Pin we are going to use to wake up the Arduino

DFRobot_EC ec;









//------------------------------------------------------ VARIABLES ----------------------------------------------------------------

const int time_interval = 30;// THIS SETS THE SAMPLING INTERVAL IN MINUTES!!!!!!!---------------------------------
// as of 11.14.24 this interval is for seconds - AMR

//---------------------------------------------------------------------------------------------------------------------------------







// storage of sample values

int bufT[10]; // temporary storage of samples for temp
int bufC[10]; // temp storage of samples for conductivity
int bufP[10]; // temp storage of samples for pressure

int conduc_raw; // storage of raw conductance value
float conduc; // calculate conductivity value
float conduc_volt; // storage of the raw voltage from conductivity

float temp; // temperature value

int pressure_raw; // pressure raw reading
float pressure_volt; // pressure volt caculated
float pressure; // calculate pressure value
//const float p_offset = 0.52 ;  // voltage offset for pressure sensor



int power_pinC = 4; // digital power pin for conduc sensors
int power_pinP = 5; // digital power pin for pressure sensor
int power_pinT = 6; // digital power pin for temperature sensor

// legacy code for DF robot temperature sensor
// int DS18S20_Pin = 3; //DS18S20 Signal pin on digital 2
// OneWire ds(DS18S20_Pin);  // on digital pin 2


OneWire oneWire(temp_pin);  // temperature digital pin
DallasTemperature sensors(&oneWire);

// Arduino nano additional parameters
RTC_DS3231 rtc; //for real time clock
File mydata; // what you are writing your data too
int chipSelect = 10; // pin of SD card reader
#define DS3231_ADDRESS     0x68   // defining the I2C address of the RTC module. 

//----------------------------------------------------SETUP-----------------------------------------------------------------------

void setup() {
Serial.begin(9600);  //turn on serial port, baud rate is 9600 in this case
pinMode(power_pinC,OUTPUT);  // set all power pins to output
pinMode(power_pinP,OUTPUT);
pinMode(power_pinT,OUTPUT);

sensors.begin();

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
            rtc.now() + TimeSpan(0,0,0,time_interval),  // time span is (Days, Hours, MInutes, Seconds). MOve your Time_interval to appropriate location
            DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set");  
    }


delay(50); // set all powerpins to off
digitalWrite(power_pinC, LOW);
digitalWrite(power_pinP,LOW);
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
            rtc.now() + TimeSpan(0,0,0,time_interval),  // remember to move your time_interval as needed
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

delay(100);

digitalWrite(power_pinC, HIGH);  // turn on all power pins
digitalWrite(power_pinP, HIGH);
digitalWrite(power_pinT,HIGH);
delay(100);
sensors.begin();  // begin temperature sensor
// wait 5 seconds for sensors to power on and stabalize
delay(5000);



// loop through 8 samples with .2 seconds wait between.
for(int i=0;i<8;i++)
{    
    conduc_raw = analogRead(A0); // Read and store 10 measurement of CO2 sensor
    bufC[i] = conduc_raw;
    sensors.requestTemperatures();
    bufT[i]= sensors.getTempCByIndex(0);  // record temperature sensor values
    pressure_raw = analogRead(A1);  // record pressure sensor values
    bufP[i] = pressure_raw;

    delay(200); // wait .2 sec between each measurment. Overall sampling interaval takes 5 seconds here
}

// recording only the last taken value. But you can record more values if you want a time average or wish to trouble shoot the sensor.
temp = bufT[5];

conduc_volt = bufC[5]/1024.0*5000; // conductance mili voltage
conduc = ec.readEC(conduc_volt,temp);  // calculated conductance in ms/cm

pressure_volt = bufP[5]/1024.0*5.00; // presure voltage
pressure = pressure_volt * 250 ; // presure in kPa



delay(100);
digitalWrite(power_pinC, LOW);  // turn all power pins off
digitalWrite(power_pinP,LOW);
digitalWrite(power_pinT,LOW);
delay(100);

write_file(conduc,temp,pressure_volt);//sends data to write file

delay (100);

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
    Serial.print(";  ms/cm:");
    Serial.print(conduc);
    Serial.print(";  DegreeC:");
    Serial.print(temp);
    Serial.print(";  pressureV:");
    Serial.print(pressure_volt,4);
    Serial.println(";");
 
delay(100);

}

//---------------------------------------------------------WRITE FILE FUNCTION-------------------------------------------------------------------
// writes the data to the SD card
void write_file(float conduc,float temp,float pressure_volt){

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
    mydata.print(conduc);
    mydata.print(';');
    mydata.print(temp);
    mydata.print(';');
    mydata.println(pressure_volt);
      mydata.close();
   
  }
  
  }


//-------------------------------------------------------------------------------------------------------
// legacy code for df robot temperature

// float getTemp(){
//   //returns the temperature from one DS18S20 in DEG Celsius

//   byte data[12];
//   byte addr[8];

//   if ( !ds.search(addr)) {
//       //no more sensors on chain, reset search
//       ds.reset_search();
//       return -1000;
//   }

//   if ( OneWire::crc8( addr, 7) != addr[7]) {
//       Serial.println("CRC is not valid!");
//       return -1000;
//   }

//   if ( addr[0] != 0x10 && addr[0] != 0x28) {
//       Serial.print("Device is not recognized");
//       return -1000;
//   }

//   ds.reset();
//   ds.select(addr);
//   ds.write(0x44,1); // start conversion, with parasite power on at the end

//   byte present = ds.reset();
//   ds.select(addr);
//   ds.write(0xBE); // Read Scratchpad


//   for (int i = 0; i < 9; i++) { // we need 9 bytes
//     data[i] = ds.read();
//   }

//   ds.reset_search();

//   byte MSB = data[1];
//   byte LSB = data[0];

//   float tempRead = ((MSB << 8) | LSB); //using two's compliment
//   float TemperatureSum = tempRead / 16;

//   return TemperatureSum;

// }