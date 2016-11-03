//Clock OS Operating System by Terry Clarkson      10-28-2011
//Ver 1.0   Basic clock routine pb1 set time in CCW mode pb3 advance time
//Ver 1.1   Add second clock routine
//Ver 1.2   Add ability for user to set colors with 3 buttons,saves setup to eeprom
//Ver 1.3   Added servo routines and sound routine
//ver 1.4   Added ability to save 10 clock faces
//ver 1.5   Atmel now controls all clock functions, PIC is slave processor for multiplexing LEDS
//ver 1.6   removed servo and sound routines, cleanup and document all code
//
//
// note 512 byes of eeprom available
//      1k of ram
//      14k of flash
//  command list from atmel chip to Pic consist of 5 bytes -start code is 0xF# ends with 0x03
//  ***turn on or off led
//          1-7                 1-60     0-turns off  EOX
//  0xF1 , [hour,min,sec], [led number], [color 0-7], 0x03    // turns on or off single leds
//
//  *** advance cw the led pattern ***
//  0xF2 , [hour,min,sec], [number of postions to advance], [unused (set to 0x00)], 0x03    // advance leds
//
//  *** retard ccw the led pattern ***
//  0xF3 , [hour,min,sec], [number of postions to backup], [unused (set to 0x00)], 0x03    // retard leds
//
//  *** meter mode  ***
//  0xF4 , [hour,min,sec], [led start number], [led end number], [color], 0x03
//
//  *** turnoff all leds in select circle ****
//  0xF5 , [hour,min,sec], [unused], [unused], [unused], 0x03   //turn off leds
//
//  *** all leds off *****
//
//  0xF6 ,[unused], [unused], [unused], 0x03                    //turn off every led on board
//

//Thankyou to Maurice Ribble from ClockOS for the 1307 RTC code used in this project
// 1307 RTC code by Maurice Ribble
// 4-17-2008
// http://www.glacialwanderer.com/hobbyrobotics

// This code tests the DS1307 Real Time clock on the Arduino board.
// The ds1307 works in binary coded decimal or BCD.  You can look up
// bcd in google if you aren't familior with it.  There can output
// a square wave, but I don't expose that in this code.  See the
// ds1307 for it's full capabilities.
#include <EEPROM.h>
#include "Wire.h"
//#include <Servo.h>
//#include "pitches.h"
//Servo myservo;  // create servo object to control a servo
#define DS1307_I2C_ADDRESS 0x68
#define twelthHour  B00010000
#define everyHour  B00001000
#define qtrHour B00100000
#define traceSec  B00001000

//define colors
#define Red  0x01
#define Green  0x02
#define Orange  0x03
#define Blue  0x04
#define Violet  0x05
#define Cyan  0x06
#define White 0x07
#define Clear  0x00


//setup buttons
const byte pb1 = 8;     //used to set seconds
const byte pb2 = 9;     //used to set minutes
const byte pb3 = 10;     //used to set hours


//declare varibles here
int step_value = 1;
int brightness;
byte ExitFlag;
byte varible;
byte sec_count;
byte ButtonPressed = 0;
byte second_counter = 0;
byte command = 0;
byte old_second = 0;
byte old_minute = 0;
byte old_hour = 0;
byte timeSet_flag = 0;
byte var = 0;
byte loopCtr = 0;
byte clock_face = 1;
byte old_clock_face;
byte pend_dot;
byte pend_direction;
byte color;
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte dechr, decmin, decsec;
unsigned int pendulum_timer,pendulum_time_slice,ptimer,stimer;
unsigned long old_millis,old_sec_millis,old_tube_millis;
byte hour_marker_color = Violet;
byte hour_color = Red;
byte minute_color = Red;
byte second_color = Green;
byte pendulum_color = Blue;
byte sec_shift;
//analog varibles
int sensorPin = A0;               //pin to read analog value from
int sensorValue = 0;              //value of analog input

//arrays need for sound (westminister chimes)
//int melody[] = {NOTE_E4, NOTE_GS4, NOTE_FS4, NOTE_B3, NOTE_E4,NOTE_FS4, NOTE_GS4, NOTE_E4, NOTE_GS4, NOTE_E4, NOTE_FS4, NOTE_B3, NOTE_B3, NOTE_FS4, NOTE_GS4, NOTE_E4};
//int noteDurations[] = {2,2,2,1,2,2,2,1,2,2,2,1,2,2,2,1};
byte clock_color[11][5];   //colors for clock faces,  10 faces with 5 colors each
// 0-off
// 1-red
// 2-green
// 3-red/green
// 4-blue
// 5-blue/red  (pink)
// 6-Blue/Green (Cyan)
// 7- red/green/blue

//   default_color(seconds,minutes,hours,hour_markers,pendulum)  hour marker  3,4 = 0 no hour,   bit3= every hour,   bit4 12th hour only,  bit 5 qtr hour
//                                                               seconds - bit 4 = trace mode
const byte default_color[11][5] =       //clock face 1 factory default colors
  { {0,0,0,0,0},                        //not used currently, for future use
    {0,2,1,4|everyHour,0},     //face 1   green trace second, red minute, red hour, blue every hour marker, no pendulum
    {0,2,1,4|twelthHour,0},             //face 2   green second, red minute, red hour, blue 12th hour markers, blue pendulum
    {0,2,1,4|qtrHour,0},              //face 3   green second, red minute, red hour, blue every hour markers, no pendulum
    {0,2,1,4,0},                //face 4   no second, red minute, red hour, blue qtr hour markers, no pendulum
    {0,5,4,1|qtrHour,0},     //face 5   green second, red minute, red hour, blue qtr hour markers, no pendulum
    {0,1,4,2,0},                //face 6
    {0,1,4,2|twelthHour,0},             //face 7
    {0,1,4,2|qtrHour,0},              //face 8
    {0,2,4,2|everyHour,0},              //face 9
    {1,2,4,0,0}                         //face 10 (tracer)
   };
//===========================  Start of functions  ===============================================================
void led_write(byte ring, byte number, byte color)
         {
          Serial.write(0xF1);       //turn on\off  led command
          Serial.write(ring);       //led   ring
          Serial.write(number);     //led to light 0-59
          Serial.write(color);      //assign color
          Serial.write(0x03);
         }

void arc_write(byte ring, byte start, byte end, byte color)
{
  Serial.write(0xF4);
  Serial.write(ring);
  Serial.write(start);
  Serial.write(end);
  Serial.write(color);
  Serial.write(0x03);
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
   {return ( (val/10*16) + (val%10) );}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
  {return ( (val/16*10) + (val%16) );}

// Stops the DS1307, but it has the side effect of setting seconds to 0
// Probably only want to use this for testing
/*void stopDs1307()
{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  int i = 0;
  Wire.write(i);
  Wire.write(0x80);
  Wire.endTransmission();
}*/

// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers
void setDateDs1307(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
  {
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   int i = 0;
   Wire.write(i);
   Wire.write(decToBcd(second));      // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));        // If you want 12 hour am/pm you need to set
                                     // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
  }
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Gets the date and time from the ds1307
void getDateDs1307(byte *second,
          byte *minute,
          byte *hour,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *month,
          byte *year)
  {
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  int i = 0;
  Wire.write(i);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
  }

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void draw_qtr_hour_markers(void)          //only display alt hours (5) (bit 3 is not set
       {
         int i = 0;
        if (old_minute != 0 && old_hour != 0)
         {led_write(1,i,hour_marker_color & 0x07);}

        if (old_minute != 12 && old_hour * 6 != 12)
            {led_write(1,12,hour_marker_color & 0x07);}

         if (old_minute != 24 && old_hour *6 != 24)
            {led_write(1,24,hour_marker_color & 0x07);}

        if (old_minute != 36 && old_hour * 6 != 36)
            {led_write(1,36,hour_marker_color & 0x07);}

        if (old_minute != 48 && old_hour *6 != 48)
            {led_write(1,48,hour_marker_color & 0x07);}
        }

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void draw_hour_markers(void)                                      //hour markers shown for all hours (10)
       {
        if (old_minute != 0 && old_hour != 0)                      //display 00:00 dots if minute hand is not there
            {led_write(7,0,hour_marker_color & 0x07);}               //mask off bit 4 to look up color

        if (old_minute != 6 && old_hour*6 !=6)                       //display 0:05 min dots if minute hand is not there
            {led_write(1,6,hour_marker_color & 0x07);}

        if (old_minute != 12 && old_hour*6 !=12)                     //display 00:15 dots if minute hand is not there
            {led_write(1,12,hour_marker_color & 0x07);}

        if (old_minute != 18 && old_hour*6 !=18)                    //display 0:20 dots if minute hand is not there
            {led_write(1,18,hour_marker_color & 0x07);}

        if (old_minute != 24 && old_hour*6 !=24)                    //display 0:25 dots if minute hand is not there
            {led_write(1,24,hour_marker_color & 0x07);}

        if (old_minute != 30 && old_hour*6 !=30)                    //display 0:30 dots if minute hand is not there
            {led_write(1,30,hour_marker_color & 0x07);}

        if (old_minute != 36 && old_hour*6 !=36)                    //display 0:35 dots if minute hand is not there
            {led_write(1,36,hour_marker_color & 0x07);}

        if (old_minute != 42 && old_hour*6 !=42)                    //display 0:40 dots if minute hand is not there
            {led_write(1,42,hour_marker_color & 0x07);}

        if (old_minute != 48 && old_hour*6 !=48)                    //display :0:45 dots if minute hand is not there
            {led_write(1,48,hour_marker_color & 0x07);}

        if (old_minute != 54 && old_hour*6 !=54)                    //display 0:55 dots if minute hand is not there
            {led_write(1,54,hour_marker_color & 0x07);}

      }

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void draw_12th_hour_marker(void)                                   //hour markers shown for all hours (10) (bit 4)
       {led_write(3,0,hour_marker_color & 0x07);}                  //only display the 10th hour marker

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void all_seconds_off(void)
    {
    Serial.write(0xF5);                                          //turn on or off led command
    Serial.write(0x01);                                          //led on seconds ring
    Serial.write(0x01);                                           //
    Serial.write(0x01);                                           //
    Serial.write(0x03);                                           //end of command
    }
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void all_minutes_off(void)
    {
    Serial.write(0xF5);              //turn on or off led command
    Serial.write(0x02);              //led on minutes ring
    Serial.write(0x01);              //not used
    Serial.write(0x01);              //not used
    Serial.write(0x03);              //end of command
    }
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void all_hours_off(void)
   {
    Serial.write(0xF5);              //turn on or off led command
    Serial.write(0x04);              //led on hours ring
    Serial.write(0x01);              //
    Serial.write(0x01);              //
    Serial.write(0x03);               //end of command
    }
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void all_leds_off(void)
   {
    Serial.write(0xF6);              //turn on or off led command
    Serial.write(0x07);              //all leds
    Serial.write(0x01);
    Serial.write(0x01);
    Serial.write(0x03);               //end of command
    }

//==========================  Clock Routines  ===============================================

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void dot_clock(void) {
   int totalmin24 = hour * 60 + minute;
   int totalmin10 = (int)((float)totalmin24 * 1000.0 / 1440.0);
   dechr = totalmin10 / 100;
   decmin = (int)(0.6 * (float)(totalmin10 % 100));


   if (second == old_second) {
     return;
   }
   old_second = second;

   // erase old hour if needed.
   if (dechr != old_hour) {
     led_write(3, old_hour * 6, Clear);
     led_write(3, old_hour * 6 + 1, Clear);
     led_write(3, old_hour * 6 + 2, Clear);
     led_write(3, old_hour * 6 + 3, Clear);
     led_write(3, old_hour * 6 + 4, Clear);
     led_write(3, old_hour * 6 + 5, Clear);
     old_hour = dechr;
   }

   // erase old minute.
   if (decmin != old_minute) {
     led_write(7, old_minute, Clear);
   }

  // draw hour markers.
  if (bitRead(hour_marker_color,4) == 1)     //bit 4 (0001 0000) signifies that only the 12th hour is displayed
  {
    draw_12th_hour_marker();
  }
  else if (bitRead(hour_marker_color,3) == 1)    //check for bit 3 (0000 1000) set on color
  {
    draw_hour_markers();
  }                 //all hour leds are lit up
  else if (bitRead(hour_marker_color,5) == 1)      //check to see if bit 5 is set in color byte)
  {
    draw_qtr_hour_markers();
  }

  // Draw hour bar.
  led_write(3, dechr * 6, hour_color);
  led_write(3, dechr * 6 + 1, hour_color);
  led_write(3, dechr * 6 + 2, hour_color);
  led_write(3, dechr * 6 + 3, hour_color);
  led_write(3, dechr * 6 + 4, hour_color);
  led_write(3, dechr * 6 + 5, hour_color);

  //draw minute.
  led_write(7,decmin,minute_color);          //update minutes
  old_minute = decmin;
}

/* This replaces previous drawing routine.
if(second != old_second)                          //update face if a second has elapsed
  { sec_count=0;
      //brightness=0;                               //used to pulse light in tubes
     if(bitRead(second_color,3) == 0)             //if bit 3 is not set then dont trace mode the seconds
       {led_write(1,old_second,Clear);}           //clear the old second when not in trace mode
     else
       {
        if (old_second == 59)
           {var = 0;
             while (var <=59)                   //erase seconds one at a time starting with led 0
              {led_write(1,var,Clear);
               var=var+1;                       //loop clockwise around clock face truning off leds one at a time
              }
          }
       }

   if(minute != old_minute)                     //erase last minute if it has changed
      {led_write(7,old_minute,Clear);}


  //erase last hour
      if(minute != old_minute)                  //anytime minutes change,hours could have changed
         {
         if(old_hour != hour*5+(minute/12))     //if hour marker has changed,erase old value
             {led_write(6,old_hour,Clear);}
         }

     if (bitRead(hour_marker_color,4) == 1)     //bit 4 (0001 0000) signifies that only the 12th hour is displayed
       {draw_12th_hour_marker();}
     else
       {
       if (bitRead(hour_marker_color,3) == 1)    //check for bit 3 (0000 1000) set on color
           {draw_hour_markers();}                 //all hour leds are lit up
       else
          { if (bitRead(hour_marker_color,5) == 1)      //check to see if bit 5 is set in color byte)
              {draw_qtr_hour_markers();}               //display only the qtr hour markers
          }
       }

 //update minutes
      led_write(7,minute,minute_color);          //update minutes
      old_minute = minute;

 //update hours
      old_hour = (hour*5)+(minute/12);           //save for erase routine
      led_write(6,old_hour,hour_color);


  //update seconds
    if(timeSet_flag == 0)                          // do not update seconds if in 'time set' mode
        { led_write(1,second,second_color & 0x07);  //mask out the third bit
          old_second = second;
        }
    }
*/

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/*
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void large_hand_clock(void)
 {
   if (pendulum_color != Clear)      //face 3 is dot clock with penulum (pendulum must have color assigned to operate)
      {  if ((millis() -old_millis) >= 125)            //have 125 milliseconds elapsed?
        {
          old_millis = millis();                       //save value
        //--------- pendulum section ---------------------
        //erase trailing pendulum dots
        if (pend_direction == 1)                       //erase trailing leds
          {led_write(6,pend_dot-2,Clear);}

       if (pend_direction ==0)                         //erase trailing leds
          {led_write(6,pend_dot+2,Clear);}

       if (pend_dot >= 34)                           // are we at maximum left swing?
          {pend_dot = 34;                           //if  so then change direction
           pend_direction = 0;
          // myservo.write(20);
          //  tone(2, 250,20);                        //clicks the speaker for tick tock sound
          }
       else if (pend_dot <= 26)                      //change direction if at max swing
          {pend_dot=26;
           pend_direction = 1;
          // myservo.write(160);
           // tone(2, 200,20);
           }
        if (pend_direction == 0)
           {pend_dot = pend_dot-1;}                 //deincrement counter for next time
        else
           {pend_dot = pend_dot+1;}                  //increment counter for next time


       led_write(6,pend_dot-1,pendulum_color);       //draw pendulum
       led_write(6,pend_dot,pendulum_color);
       led_write(6,pend_dot+1,pendulum_color);
       led_write(6,hour*5+(minute/12),hour_color);  //rewrite the hour leds in case they were written over by pendulum
       if (minute >=25 && minute <= 34)         //rewrite the minute hand, this keeps it on top of the pendulum
          {led_write(7,minute,minute_color);}
 //------end of pendulum routine ------------
        }
      }
//    if ((millis() -old_sec_millis) >= 333)       //3 dot second from center out 333 ms pulse
//        {old_sec_millis = millis();
//         sec_shift = sec_shift >> 1;
//         if (sec_shift == 0)
//             {sec_shift = 4;}
//         led_write(7,second,Clear);
//         led_write(sec_shift,second+1,second_color);
//         }



if(second != old_second)                          //update face if a second has elapsed
  { sec_count=0;

     if(bitRead(second_color,3) == 0)             //if bit 3 is not set then dont trace mode the seconds
       {led_write(1,old_second,Clear);}           //clear the old second when not in trace mode
     else
       {
        if (old_second == 59)
           {var = 0;
             while (var <=59)                   //erase seconds one at a time starting with led 0
              {led_write(1,var,Clear);
               var=var+1;                       //loop clockwise around clock face truning off leds one at a time
              }
          }
       }

   if(minute != old_minute)                     //erase last minute if it has changed
      {led_write(7,old_minute-1,Clear);

    }


  //erase last hour
      if(minute != old_minute)                  //anytime minutes change,hours could have changed
         {
         if(old_hour != hour*5+(minute/12))     //if hour marker has changed,erase old value
                {
                //led_write(4,old_hour-2,Clear);  //erase old hour marker
                led_write(4,old_hour-1,Clear);
                led_write(7,old_hour,Clear);
                led_write(4,old_hour+1,Clear);
               // led_write(4,old_hour+2,Clear);
                }
         }

     if (bitRead(hour_marker_color,4) == 1)     //bit 4 (0001 0000) signifies that only the 12th hour is displayed
       {draw_12th_hour_marker();}
     else
       {
       if (bitRead(hour_marker_color,3) == 1)    //check for bit 3 (0000 1000) set on color
           {draw_hour_markers();}                 //all hour leds are lit up
       else
          { if (bitRead(hour_marker_color,5) == 1)      //check to see if bit 5 is set in color byte)
              {draw_qtr_hour_markers();}               //display only the qtr hour markers
          }
       }

 //update minutes
      led_write(7,minute,minute_color);
      led_write(7,minute+1,minute_color);          //update minutes
      old_minute = minute;

 //update hours
      old_hour = (hour*5)+(minute/12);           //save for erase routine
      //led_write(4,old_hour-2,hour_color);
      led_write(4,old_hour-1,hour_color);
      led_write(7,old_hour,hour_color);
      led_write(4,old_hour+1,hour_color);
     // led_write(4,old_hour+2,hour_color);
  //update seconds
    if(timeSet_flag == 0)                          // do not update seconds if in 'time set' mode
        { led_write(1,second,second_color & 0x07);  //mask out the third bit
          old_second = second;
        }
    }

 } */
//========== Tracer Clock  =============================================================

void tracer_clock(void)                     //leds stay on from led zero location to current

 {
if(second != old_second)                    //update face if a second has elapsed
   {

   if (second == 1)                         //erase seconds if full circle
        {all_leds_off();}

   draw_qtr_hour_markers();                  //refresh hour markers

   var = 0;                                //update minutes  (use meter mode routine (0xF4)
   while(var <= minute)                    //start at zero led and turn on all leds upto and including
                                           //current minute
      {led_write(2,var,minute_color);
       var= var+1;
      }
    old_minute = minute;                   //save current minute to be able to detect change

   var = 0;                                //update hours
   while(var <= hour*5+(minute/12))
      {led_write(4,var,hour_color);
       old_hour = (hour*5)+(minute/12);   //save for erase routine
       var=var+1;
      }

   if(timeSet_flag == 0)                  // do not update seconds if in 'time set' mode
        {led_write(1,second,second_color);
         old_second = second;
        }
   }

 }
//============= New Clock Routine Goes Here ============================================
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//=====================Analog routine  ==================================================
//  void AnalogRoutine(void)
//     {
//       int sensorValue = analogRead(A1);   //get the analog value from Pin A0
//
//
//       sensorValue =map(sensorValue, 0, 1024, 0, 60);
//       if (var != sensorValue)                             //if value has changed
//          {//var = sensorValue;                              //save for next time
//          loopCtr = 0;
//          if (var >= sensorValue)
//           {
//           while (loopCtr <= var)
//             {  if (var >=36)
//                     {color = Blue;}
//                else
//                   {if (var <=24)
//                      {color = Orange;}
//                   else
//                      {color = Green;}
//                   }
//                if ((loopCtr != 25) && (loopCtr != 35))  //do not overwrite red markers
//                   {Serial.write(0xF1);          //turn on or off led command
//                    Serial.write(0x07);          //led on seconds ring
//                    Serial.write(loopCtr);        //light up the current second in rtc module
//                    if (loopCtr <= sensorValue)
//                        {Serial.write(color);}   //make led green
//                     else
//                         {Serial.write(Clear);}
//                    Serial.write(0x03);           //end of command
//                   }
//              loopCtr = loopCtr +1;
//             }
//         }
//         else
//             { while (loopCtr >=var+1)
//               {
//                if ((loopCtr != 25) && (loopCtr != 35))  //do not overwrite red markers
//                  {
//                   Serial.write(0xF1);          //turn on or off led command
//                   Serial.write(0x07);          //led on seconds ring
//                   Serial.write(loopCtr);        //light up the current second in rtc module
//                   Serial.write(Clear);   //make led green
//                   Serial.write(0x03);           //end of command
//                   }
//               loopCtr = loopCtr - 1;
//               }
//             }
//          led_write(7,25,Red);              //right boundry marker
//          led_write(7,35,Red);              //left boundry marker
//         }
//       var = sensorValue;
//      }
//

 //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 void display_dot_colors(void)                              //routine used to set colors for the leds in the different routines
       {
          //draw_qtr_hour_markers();
          if (bitRead(second_color,3) == 1)                      //display 3 dots in trace mode, one dot in non trace mode
              {
              loopCtr = 56;
              while(loopCtr <= 58)                                //display dots 56,57,58 on seconds
                  {led_write(1,loopCtr,second_color & 0x07);
                  loopCtr = loopCtr+1;
                  }
              }
          else
             {
             led_write(1,56,Clear);                          //erase tracer mode dots
             led_write(1,58,Clear);
             led_write(1,57,second_color & 0x07);}          //non tracer mode show single dot

          //display minute dot
          led_write(7,7,minute_color);                       //display the minute hand at the 5th led position

          //display hour dot
         led_write(6,3,hour_color);                           //display the hour hand at the 4th led position

         //display pendulum
          led_write(6,24,pendulum_color);
          led_write(6,25,pendulum_color);
          led_write(6,26,pendulum_color);

       }
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup()
    {
    Wire.begin();                             //i2c interface for the 1307 RTC chip
    pinMode(pb1, INPUT);                      //setup pin8 as input
    pinMode(pb2, INPUT);                      //setup pin9 as input
    pinMode(pb3, INPUT);                      //setup pin10 as input
    pinMode(11, OUTPUT);                    //output to fading 'hours' tube ( pin4 on icsp port)
    pinMode(3, OUTPUT);                    //output to fading 'minutes' tube ( C4 on expansion port)
    pinMode(6, OUTPUT);                     //output to fading 'seconds' tube (A0 on clockOS board)
//    myservo.attach(6);                        //ANO  output on ClockOS Board
    timeSet_flag = 0;                         //clear the time setting flag on cold start
    Serial.begin(9600);                       //enable uart port at desired baud rate

    // Change these values to what you want to set your clock to.
    // You probably only want to set your clock once and then remove
    // the setDateDs1307 call.
//    second = 0;
//    minute = 19;
//    hour = 7;
//    dayOfWeek = 5;
//    dayOfMonth = 27;
//    month = 10;
//    year = 11;
//    setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
    delay(250) ;
    all_leds_off();                                                //clear led memory buffers in Pic processor
    getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year); //on cold start get time and then set it to start clock up
    setDateDs1307(0, minute, hour, dayOfWeek, dayOfMonth, month, year);             //clock must be written to, to start timekeeping module
         if (hour>=24)                               //all clock routines are based on 12 hour time format
           {hour = hour-24;}

 //recall colors from eeprom and assign to clock face
    clock_face = EEPROM.read(0);
    if (clock_face >=11)                             //if not a valid number then assign number
        {
        clock_face = 1;
        EEPROM.write(0,clock_face);                 //save new value to EEprom
        }

//------------recall clock colors use offset of 5 since they are now being used by other features
    loopCtr = 0;
    clock_face = 1;
    while (clock_face <=10)                                        //load in colors saved in eeprom and check for valid, if not load default
        {
         while (loopCtr <=4)                                     //load in the second,min,hour,marker and pendulum colors
            {clock_color[clock_face][loopCtr] = EEPROM.read((clock_face*5)+(loopCtr));
            if (clock_color[clock_face][loopCtr] >= 0x10)          //if out of range assign color
              {clock_color[clock_face][loopCtr] = default_color[clock_face][loopCtr];
               EEPROM.write((clock_face*5)+(loopCtr),clock_color[clock_face][loopCtr]);       //assign default color saved in rom
              }
            loopCtr = loopCtr+1;
            }
         clock_face = clock_face+1;
         loopCtr = 0;
        }
    clock_face = EEPROM.read(0);                          //load in the clock face that was last running
    second_color =   EEPROM.read((clock_face*5)+0);       //load in colors saved in eeprom for the selected clock face
    minute_color =   EEPROM.read((clock_face*5)+1);
    hour_color =     EEPROM.read((clock_face*5)+2);
    hour_marker_color =   EEPROM.read((clock_face*5)+3);
    pendulum_color =   EEPROM.read((clock_face*5)+4);

   //==========================
      // un rem the following for statement to play the westminister chime song
//   for (int thisNote = 0; thisNote < 16; thisNote++)
//       {
//        // to calculate the note duration, take one second divided by the note type.
//        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
//        int noteDuration = 1000/noteDurations[thisNote];
//        tone(2, melody[thisNote],noteDuration);                         //output on 2806 output pin of ClockOS
//        // to distinguish the notes, set a minimum time between them. The note's duration + 30% seems to work well:
//        int pauseBetweenNotes = noteDuration * 1.30;
//        delay(pauseBetweenNotes);
//        noTone(8);                                   // stop the tone playing:
//       }
    }

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//=================================== Start of Main Program Loop  ===================================================
void loop()
   {
    ptimer = ptimer+1;   //used to scycronize the pendulum
    stimer = stimer+1;
    if (digitalRead(pb1) == 1 && digitalRead(pb3) ==1 && digitalRead(pb2) == 1)   //if no buttons are pressed
       {ButtonPressed = 0;}                   //clear flag when all 3 buttons are released

 //********* PushButton 3 - Set time in CCW mode (Run in hands in reverse)    **************************
    if (digitalRead(pb1) == 1 && digitalRead(pb3) == 0 && digitalRead(pb2) == 1)       //run time backwards to set clock time
        {  delay(10);                                          //switch debouce
           if (digitalRead(pb1) == 1 && digitalRead(pb3) == 0 && digitalRead(pb2) == 1) //if button three is still low then proceed
              {
               timeSet_flag = 1;
               minute = minute-1;                             //subract one minute
               if (minute >= 0x60)                            //if de-incremented below zero then set for the 59th led
                   {minute = 59;
                    hour = hour-1;                            //de-increment the hour counter since we passed through the 59th minute led
                    if( hour >= 24)
                       {hour = 23;}
                    }
               second=second+1;                               //forces update of clock face
               if (second >= 60)                              //do not increment past 59th 'second' led
                   {second = 0;}
              }
         }

 //********* PushButton 2 - Select Clock style  ***********************************
     if (digitalRead(pb2) == 0 && digitalRead(pb1) == 1 && digitalRead(pb3) == 1)  //Clear clock face
        {delay(150);                                             //switch debounce
          if (digitalRead(pb2) == LOW && digitalRead(pb1) == 1 && digitalRead(pb3) == 1)//if still low then proceed
             {
              clock_face = clock_face +1;                           //select the next built in clock when button is released
              if (clock_face >=11)                                   //enter total number of clock faces here to compare to.
                 {clock_face = 1;}
              EEPROM.write(0,clock_face);                           //save selected clock face to eeprom
              loopCtr=59;
              delay(50);
              led_write(7,0,Blue);
              while (loopCtr!=30)                          //clear clock face with blue wipe of ledss
                  {led_write(7,loopCtr,Blue);
                   led_write(7,60-loopCtr,Blue);
                   loopCtr = loopCtr-1;
                   delay(10);
                   }

               loopCtr=59;
               led_write(7,0,Clear);                        //erase blue led pattern
               while (loopCtr!=29)
                  {led_write(7,loopCtr,Clear);
                   led_write(7,60-loopCtr,Clear);
                   loopCtr = loopCtr-1;
                   delay(10);
                   }

               loopCtr = 0;
               while (loopCtr!=clock_face)                          //display green dots  in seconds ring to indicate what clock face this is
                  {if((loopCtr == 0) || (loopCtr ==4) || (loopCtr ==9))
                      {led_write(3,loopCtr,Red);}                 //display double dots at location 1,5, and 10 for easier readability
                   else
                      {led_write(1,loopCtr,Red);}                 //display single dot on 2,3,4,6,7,8,9
                   loopCtr = loopCtr + 1;
                  }
               delay(1500);                                           //allow time to view what clock face number we are on

               loopCtr = 0;
               while (loopCtr!=clock_face)                          //erase dots indicating what clock face this is
                  {if(loopCtr == 0 || loopCtr ==4 || loopCtr ==9)   //erase double dots at location 1,5, and 10 for easier readability
                      {led_write(3,loopCtr,Clear);}
                   else
                      {led_write(1,loopCtr,Clear);}                 //erase single dot on 2,3,4,6,7,8,9
                   loopCtr = loopCtr + 1;
                   }


              //get clock colors for selected face
              second_color =   EEPROM.read((clock_face*5)+0);       //load in colors saved in eeprom for the selected clock face
              minute_color =   EEPROM.read((clock_face*5)+1);
              hour_color =     EEPROM.read((clock_face*5)+2);
              hour_marker_color =   EEPROM.read((clock_face*5)+3);
              pendulum_color =   EEPROM.read((clock_face*5)+4);
             }
         }


 //****** Push Button 1 - advance time on RTC CW  *************************************

    if (digitalRead(pb3) == 1 && digitalRead(pb1) == 0 && digitalRead(pb2) == 1)  //run time in advance
        {
          delay(10);                   //only delay 10 ms when "holding" button down
          if (digitalRead(pb3) == 1 && digitalRead(pb1) == 0)  //if still low then proceed
             {
              timeSet_flag = 1;                     //set flag so program knows to update RTC when done
              minute= minute+1;                     //advance clock one minute
              if (minute >= 60)                     //if past the 59th minute then reset to zero and increment the hours
                 {minute = 0;
                  hour= hour+ 1;
                   if (hour >= 24)                  //if past the 11th hour then reset hours to zero
                     {hour = 0;}
                 }
              second=second+1;                       //forces update of clock face
              if (second>= 60)                       //if past the 59th second then reset seconds to zero
                 {second= 0;}
             }
        }
 //****** Push Button 1 and Push Button 3 - Set Clock Colors and bit 4,5 sets special features *************************************

       if (digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 0)
           {delay(80);                                        //switch debounce
            if (digitalRead(pb1) == 0 && digitalRead(pb2) ==1 && digitalRead(pb3) == 0)
               {
                 all_leds_off();                              //turn off entire clock face
                 display_dot_colors();                        //display screen used to set led colors
                /// varible = bitRead(hour_marker_color,3);      //high bit 3 means draw a dot at every hour
                  if (bitRead(hour_marker_color,3) == 1)                          //display hour markings depending on bit 3 setting
                     {draw_hour_markers();}                  //put a marker at every hour
                  else
                     {
                     varible = hour_marker_color;             //save current value
                     hour_marker_color = Clear;               //temporarily make the color clear
                     draw_hour_markers();                      //erase existing dots
                     hour_marker_color = varible;             //recall original color value
                     if(bitRead(hour_marker_color,4)==0)      //4th bit set means display only the 12th hour marker
                         {draw_qtr_hour_markers(); }            //put a marker at every 3 hours
                     else
                         {draw_12th_hour_marker();}           //draw only the 12th hour if bit 4 is set  in color
                     }
                 delay(100);                                  //switch debounce timer
                 while(digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 0)
                        {delay(100);}                         //wait here at least 100 ms

                 ExitFlag=0;
                 while(ExitFlag == 0)                              //loop here until two outside buttons are pressed again to exit this loop
                      {
                        //set new 'seconds' color
                        if(digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 1)      //button one pressed?
                           { delay(250);                      //debounce delay
                          if(digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 1)
                             {second_color = second_color+1;   //increment through the colors
                                if(second_color >= 0x10)          //start over if incremented past the 8th color on second function (0 is off)
                                   {second_color =0;}          //0 causes seconds to not display
                                display_dot_colors();           //display color change on clock face
                                EEPROM.write(((clock_face*5)+0),second_color);  //save to eeprom
                                delay(300);
                              }
                            }

                         //set new 'minute' color
                         if(digitalRead(pb2) == 1 && digitalRead(pb1) == 1 && digitalRead(pb3) == 0)    //button two pressed?
                            { delay(250);                           //debounce delay
                              if(digitalRead(pb1) == 1 && digitalRead(pb2) == 1 && digitalRead(pb3) == 0)
                                 {minute_color = minute_color+1;    //increment through colors
                                  if(minute_color >= 8)             // rotate through colors (minutes cannot be set to off)
                                     {minute_color =1;}
                                  display_dot_colors();             //display color change on clock face
                                  EEPROM.write(((clock_face*5)+1),minute_color);  //save to eeprom
                                  delay(300);
                                  }
                            }

                         //set new hour color
                         if(digitalRead(pb3) == 1 && digitalRead(pb1) == 1 && digitalRead(pb2) == 0)     //button three pressed?
                            { delay(250);                             //debounce delay
                              if(digitalRead(pb3) == 1 && digitalRead(pb1) ==1 && digitalRead(pb2) == 0)
                                {hour_color = hour_color+1;          //increment through colors
                                  if(hour_color >= 8)                //rotate through colors (hours cannot be turned off)
                                     {hour_color =1;}
                                  display_dot_colors();              //update clock face
                                  EEPROM.write(((clock_face*5)+2),hour_color);  //save to eeprom
                                  delay(300);
                                }
                            }

                         //set new hour marker color
                         if(digitalRead(pb1) == 0 && digitalRead(pb2) == 0 && digitalRead(pb3) == 1)
                            { delay(250);                               //debounce delay
                              if(digitalRead(pb1) == 0 && digitalRead(pb2) ==0 && digitalRead(pb3) == 1)
                                {hour_marker_color= hour_marker_color+1;//increment to next color
                                  if(hour_marker_color >= 0x20)         //bit 4 changes pattern from qtr hour to every hour
                                     {hour_marker_color =0;}            //if set to 0, this turns off the hour markers
                                  display_dot_colors();                 //update clock face (sec,min,hour,pendulum)with new color settings
                                  varible = bitRead(hour_marker_color,3);
                                  if (varible == 1)                      //display hour markings depending on bit 3 setting
                                     {draw_hour_markers();}              //put a marker at every hour
                                  else
                                     {
                                     varible = hour_marker_color;             //save current value
                                     hour_marker_color = Clear;
                                     draw_hour_markers();                     //erase existing dots
                                     hour_marker_color = varible;             //recall value
                                     if(bitRead(hour_marker_color,4)==0)      //bit 4 indicates only the 12th hour  gets displayd
                                         {draw_qtr_hour_markers(); }            //put a marker at every 3 hours
                                     else
                                         {draw_12th_hour_marker();}           //draw only the 12th hour if bit 4 is set  in color
                                     }
                                  EEPROM.write((clock_face*5)+3,hour_marker_color);  //save to eeprom
                                  delay(300);
                                }
                            }

                         //set pendulum color
                         if(digitalRead(pb1) == 1 && digitalRead(pb2) == 0 && digitalRead(pb3) == 0)
                            { delay(250);                           //debounce delay
                              if(digitalRead(pb1) == 1 && digitalRead(pb2) ==0 && digitalRead(pb3) == 0)
                                {pendulum_color= pendulum_color+1;
                                  if(pendulum_color >= 8)
                                     {pendulum_color =0;}             //0 turns off the pendulum function
                                  display_dot_colors();               //update clock face with new color setting
                                  EEPROM.write(((clock_face*5)+4),pendulum_color);  //save to eeprom
                                  delay(300);
                                }
                            }
                       if(digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 0)
                           {ExitFlag=1;}                                   //set flag to exit this current 'while' loop

                      }

                   //clear clock face
                   loopCtr=59;
                    delay(50);
                    led_write(7,0,Red);
                   while (loopCtr!=30)                          //clear clock face with red wipe of ledss
                    {led_write(7,loopCtr,Red);
                     led_write(7,60-loopCtr,Red);
                     loopCtr = loopCtr-1;
                     delay(10);
                     }
                 loopCtr=59;
                 led_write(7,0,Clear);                        //erase red led pattern
                 while (loopCtr!=30)
                    {led_write(7,loopCtr,Clear);
                     led_write(7,60-loopCtr,Clear);
                     loopCtr = loopCtr-1;
                     delay(10);
                     }

                   while (digitalRead(pb1) == 0 && digitalRead(pb2) == 1 && digitalRead(pb3) == 0)  //loop until buttons are released
                      {delay(80);}                                     //give time to release buttons
                }

             }
          if(digitalRead(pb1) == 0 && digitalRead(pb2) == 0 && digitalRead(pb3) ==1)        //hold 1 and 2 buttons for 5 seconds to reset to factory defaults
              {
               delay(500);                                     //switch debounce
               while(digitalRead(pb1) == 0 && digitalRead(pb2) == 0 && digitalRead(pb3) == 1)  //are buttons still being held?
                   {
                    loopCtr=59;
                    delay(50);
                    led_write(7,0,Blue);
                    while (loopCtr!=30 && (digitalRead(pb1) == 0 && digitalRead(pb2) == 0 && digitalRead(pb3) == 1))  //advance blue wipe of ledss
                       {led_write(7,loopCtr,Blue);
                        led_write(7,60-loopCtr,Blue);
                        loopCtr = loopCtr-1;
                        delay(100);
                        }
                     if (loopCtr ==30)              //if 2 buttons are held for 3 seconds reset clock
                           {
                            loopCtr = 0;
                            clock_face = 1;
                             while (clock_face <=10)                                        //load in colors saved in eeprom and check for valid, if not load default
                              {
                               while (loopCtr <=4)                                     //load in the second,min,hour,marker and pendulum colors
                                  {clock_color[clock_face][loopCtr] = default_color[clock_face][loopCtr];
                                   loopCtr = loopCtr+1;
                                  }
                               clock_face = clock_face+1;
                               loopCtr = 0;
                              }
                             all_leds_off();                              //clear screen
                             loopCtr = 0;
                             clock_face = 1;
                             while (clock_face <=10)                                        //save default colors to eeprom
                              {
                               while (loopCtr <=4)                                     //load in the second,min,hour,marker and pendulum colors
                                  {EEPROM.write((clock_face*5) + loopCtr,clock_color[clock_face][loopCtr]); //write the default colors to eeprom
                                   loopCtr = loopCtr+1;
                                  }
                               clock_face = clock_face+1;
                               loopCtr = 0;
                              }
                              clock_face = 1;                       //after reloading in default colors reset to clock face 1
                              EEPROM.write(0,clock_face);
                            }
                      else
                           {all_leds_off();}                //if buttons were released early then clear screen

                     }
                  while(digitalRead(pb1) == 0 && digitalRead(pb2) == 0 && digitalRead(pb3) == 1)   //wait for buttons to be released
                      {delay(10);}

             }






 //************************************************* END OF PUSHBUTTON ROUTINES ********************************************************


    if(timeSet_flag == 0) //if time set buttons are not being pressed then get current time
        {getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
         if (hour>=24)                               //all clock routines are based on 12 hour time format
           {hour = hour-24;}
        }

  /*    //this code used to  display the time on the serial port
  Serial.print(hour, DEC);
  Serial.print(":");
  Serial.print(minute, DEC);
  Serial.print(":");
  Serial.print(second, DEC);
  Serial.println("  ");

  //Serial.print(month, DEC);
  //Serial.print("/");
  //Serial.print(dayOfMonth, DEC);
  //Serial.print("/");
  //Serial.print(year, DEC);
  //Serial.print("  Day_of_week:");
  //Serial.println(dayOfWeek, DEC);
  */

//***** Start of routine to update clock face ****************************
//****  Update clock face every second

       if(old_clock_face != clock_face)         //if new style was selected clear all display buffers in Pic
         {//all_leds_off();
          old_clock_face = clock_face;           //save new face number for next time through
         }
       switch(clock_face)                       //this switch statement allows you to write your own functions to call from here
            {case 1:
                dot_clock();
                break;
             case 2:
                 dot_clock();
                 break;
             case 3:
                 dot_clock();
                 break;
             case 4:
                 dot_clock();
                 break;
             case 5:
                 dot_clock();
                 break;
              case 6:
                 dot_clock();
                 break;
              case 7:
                 dot_clock();
                 break;
              case 8:
                 dot_clock();
                 break;
              case 9:
                 dot_clock();
                 break;
              case 10:
                 tracer_clock();
                 break;
              default:
                 clock_face = 1;
            }

      //this following code only executes after the time set buttons have been used
      if ((timeSet_flag == 1) && (digitalRead(pb3)== 1) && (digitalRead(pb1) == 1))//update if no buttons are pressed and the timeset flag is set
         {
           setDateDs1307(0, minute, hour, dayOfWeek, dayOfMonth, month, year);    //send new time to the RTC chip
           timeSet_flag = 0;                        //reset flag so update will not keep
         }

   if(timeSet_flag == 1 && ButtonPressed == 0)    //if in time set mode and first increment increase delay for the first pulse
      {delay(200);
       ButtonPressed = 1;                         //set flag after first increment so rapid advance will occur
      }

     if ((millis() - old_tube_millis) >=4)
         { old_tube_millis = millis();
           analogWrite(11, brightness);
           brightness = brightness + step_value;
           if (brightness == 0 || brightness >= 255)
               {step_value = step_value*-1;}


         }

}

//Items to add or work on
//
//  setting then month and day for daylight saving auto settings
//  external display
//  external keypad
//  link to usb on pc to set clock up
//  save color settings for each clock face
