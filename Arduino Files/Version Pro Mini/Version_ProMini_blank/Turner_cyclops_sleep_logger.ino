
// DF robot Turner Optical logger;  Version 1 made on 1/2025 by A.Rok

// This is for use with the Arduino pro mini and DS3231 RTC. 

// This program is intended for the use with a Turner cyclops system - analog. 
// This should work with any of the analog turner systems as long as they can take 5v power and work.

// All software and hardware are open source. Feel free to edit or change anything if you desire. 

// To connect to the Arduion pro mini, you will need to use a FTDI bridge board. This board should plug in to the 6 pins sticking up from the pro mini.
// Make sure the FTDI board is oriented correctly (i.e. that the pins line up correctly), and that it is rated for the correct voltage:
// i.e. if useing a 5v pro mini that the FTDI board is set to 5v

// When connecting the pro mini to the computer, make sure no serial plotter / monitor is open. This will disrupt the communication signal. 
// This connection is more difficult to establish than other Arduions. You may need to fully disconect everyting and close the IDE to get it to work




// LIBRARIES
#include <SPI.h>  //include libraries for SD and RTC
#include <SD.h>
#include "RTClib.h"  //  RTC library
#include <avr/sleep.h>  // sleep library



#define interruptPin 2 //Pin we are going to use to wake up the Arduino - connection from the RTC






//------------------------------------------------------ VARIABLES ----------------------------------------------------------------

const int time_interval = 5;// THIS SETS THE SAMPLING INTERVAL IN MINUTES!!!!!!!---------------------------------


//---------------------------------------------------------------------------------------------------------------------------------











// storage of sample values

int buf_fdom[15]; // temporary storage of samples for turner

int fdom = 0; // blank variable for fdom
int power_pin_f = 5;  // digital pin that will power sensor


// Arduino nano additional parameters
RTC_DS3231 rtc; //for real time clock
File mydata; // what you are writing your data too
int chipSelect = 10; // pin of SD card reader
#define DS3231_ADDRESS     0x68   // defining the I2C address of the RTC module. 

//----------------------------------------------------SETUP-----------------------------------------------------------------------

void setup() {
delay(100);  // trying to see if this stabilizes SD card writing

Serial.begin(9600);  //turn on serial port, baud rate is 9600 in this case
pinMode(power_pin_f,OUTPUT);


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
digitalWrite(power_pin_f, LOW);

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

digitalWrite(power_pin_f, HIGH);
// wait 2 seconds for sensor to power on and take a few readings
delay(2000);



// loop through 15 samples with .2 seconds wait between.
for(int i=0;i<15;i++)
{    
    fdom = analogRead(A1); // Read and store 15 measurement of sensor with output on A1 pin 
        buf_fdom[i] = fdom; // assign each pass to that position in the fdom array
    
    delay(200); // wait .2 sec between each measurment. Overall sampling interaval takes ~7 seconds here
}

// Taking average of a few measurements sampled at random

float fdom_avg = (buf_fdom[5] + buf_fdom[7] + buf_fdom[9] + buf_fdom[14])/4;   // average of 4 bit reading values

float fdom_volts = fdom_avg * (5.0/1024.0); // voltage reading conversion



delay(100);
digitalWrite(power_pin_f, LOW); // turn off power to sensor
delay(500);

write_file(fdom_volts);//sends data to write file

delay(500);

DateTime now = rtc.now(); 

Serial.print(now.month(), DEC);  // print the date, time, and measurmenets to the serial port - can comment this all out if no longer running tests on sensors.
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
    Serial.print("Voltage = ");
    Serial.print(fdom_volts);
    Serial.println(";");
 

}

//---------------------------------------------------------WRITE FILE FUNCTION-------------------------------------------------------------------
// writes the data to the SD card
void write_file(float fdom_volts){

  DateTime now = rtc.now(); 

  mydata = SD.open("datalog.txt", FILE_WRITE);  // opens file to write to on SD card. Specifially it is looking for a text file named 'datalog'
  
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
    mydata.println(fdom_volts);
      mydata.close();  // closes data file
   
  }
  
  }


//-------------------------------------------------------------------------------------------------------
