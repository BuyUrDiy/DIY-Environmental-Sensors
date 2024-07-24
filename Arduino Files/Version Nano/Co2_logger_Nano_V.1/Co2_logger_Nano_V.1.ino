// Co2 (sandbox electronics) data logger / sensor. Version 1 made on 2/2024 by A.Rok

// This is for use with the Arduino Nano and DS3231 RTC. 

// This program is intended for the use with the sadbox electronics CO2 I2C sensors. 
// This is a modification of the Nano_lowpower_sensor.logger program; specifically it is adding in a CO2 sensor

// All software and hardware are open source. Feel free to edit or change anything; I'm sure there are better ways of doing all this.  

// -----------------------------LIBRARIES-------------------------------------------------------

#include <SPI.h>  //include libraries for SD and RTC
#include <SD.h>
#include "RTClib.h"
#include <avr/sleep.h>
# include <NDIR_I2C.h> // For Sandbox CO2 Sensor - can be found on sandbox electronics git hub if lost
#include <SC16IS750.h> // For Sandbox CO2 Sensor

#define interruptPin 2 //Pin we are going to use to wake up the Arduino


//------------------------------------------------------ VARIABLES ----------------------------------------------------------------

const int time_interval = 15;// THIS SETS THE SAMPLING INTERVAL IN MINUTES!!!!!!!---------------------------------



// storage of co2 sample values

int buf2[10]; // temporary storage of 10 samples taken for CO2
//int ppm1;  // storage of 5th taken ppm value
//int ppm2;  // stoarge of 7th taken ppm value
int ppm3; // storage of 9th taken ppm value

NDIR_I2C mySensor(0x4D); //Adaptor's I2C address (7-bit, default: 0x4D) - infot used by Sandbox CO2, don't change
SC16IS750 i2cuart = SC16IS750(SC16IS750_PROTOCOL_I2C,SC16IS750_ADDRESS_BB); // some code for establishing the I2C protocol for CO2 sensors

// Arduino nano additional parameters
RTC_DS3231 rtc; //for real time clock
File mydata; // what you are writing your data too
int chipSelect = 10; // pin of SD card reader
#define DS3231_ADDRESS     0x68   // defining the I2C address of the RTC module. 

//----------------------------------------------------SETUP-----------------------------------------------------------------------

void setup() {
Serial.begin(9600);  //turn on serial port, baud rate is 9600 in this case


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

i2cuart.begin(9600);   // begin I2C communication with CO2 sensor
// Intialize the CO2 Sensor communication
if (mySensor.begin()) {
        Serial.println("Wait 10 seconds for sensor initialization...");  // Wait 10 seconds to establish communication with sensor (you could probably do less time)
        delay(10000);
    } else {
        Serial.println("ERROR: Failed to connect to the sensor.");  // if fails; check wiring and power to sensor
        while(1);
    }
power(0); // power off Co2 sensor until needed

delay(100);
  
  Serial.println("Power cycle");// will print when the system cycles power or resets the alarm

}

//-------------------------------------------------------------------------INITIAL LOOP-------------------------------------------------------------
void loop() {

delay(4000); //wait 4 seconds before going to sleep
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
power(1); // turn on power to CO2 sensor
delay(50);
mySensor.begin();  // begin co2 sensor

// delay sampling from sensor for 10 seconds. Need to wait for more stable measurments / values.
unsigned long start = millis();
while (millis() - start < 10000);

// loop through 10 samples with .5 seconds wait between.
for(int i=0;i<10;i++)
{    
    mySensor.measure(); // Read and store 10 measurement of CO2 sensor
    buf2[i] = mySensor.ppm;
    delay(500); // wait .5 sec between each measurment. Overall sampling interaval takes 5 seconds here
}

// recording only the last taken value. But you can record more values if you want a time average or wish to trouble shoot the sensor.

//ppm1 = buf2[5];
//ppm2 = buf2[7];
ppm3 = buf2[9];

delay(100);
power(0);  // power off the CO2 sensor
delay(1000);

write_file(ppm3);//sends data to write file


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
    Serial.print("; ppm3:");
    Serial.print(ppm3);
    Serial.println(";");
 

}

//---------------------------------------------------------WRITE FILE FUNCTION-------------------------------------------------------------------
// writes the data to the SD card
void write_file(float ppm3){

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
    mydata.println(ppm3);
      mydata.close();
   
  }
  
  }

//------------------------------------------------------------CO2 POWER FUNCTION---------------------------------------------------------------
// this is necessary for power managment to the sandbox sensor

void power (uint8_t state) {
    i2cuart.pinMode(0, INPUT);      //set up for the power control pin
 
    if (state) {
        i2cuart.pinMode(0, INPUT);  //turn on the power of MH-Z16
    } else {
        i2cuart.pinMode(0, OUTPUT);
        i2cuart.digitalWrite(0, 0); //turn off the power of MH-Z16
    }
}