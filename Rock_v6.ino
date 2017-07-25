// TODO!!!!! SBDIX with blank msg! DONe!!!!!
//27/06/17

// v7.4  with potbot v4 board
//
//includes events for wet switch
// ROCKBLOCK    8  - RX
//              9  - TX
//               A4  - Powermode 
//GPS            12  - RX
//               13  - TX
//Switch         A3  - Input:   Nothing connected --> 5V (off),  connected to pin 3 --> GND (trigger)
//               3  - GND   - 
//Open Log
//#include </Time/src/Time.h>     
#include "libs/Time.h"     
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "libs/IridiumSBD.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#define EEPROM_addrs  1
#define LED A3
#define V_BATT A1
#define SW_GPS  A2   
#define RX_GPS  3
#define TX_GPS  4
#define SW_TOL  5
#define RX_OL  10
#define TX_OL  11
#define I_HUM  2    // air --> 1, water --> 0
#define SW_RCK 8
#define RX_RCK 13
#define TX_RCK 12

#define BIG_SLEEP 10                     // in minutes
//////// ROCKBLOCK CONFIG //////////////////////////////////////////////////////////
#define MIN_SIG_CSQ 0                       // Minimum signal 0 -->5 ,    Time in seconds
#define AT_TIMEOUT 20                       //  30 default Wait for response timeout
#define SBDIX_TIMEOUT 30                  // 300 default send SBDIX loop timout
#define SBDIX_LOOP 1                      // number of loops SBDIX_TIMEOUT, it will read the GPS in between
////////////////////////////////////////////////////////////////////////////////////

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define GPS_TIMEOUT 1    // GPS timeout in minutes
#define GPS_THRESHOLD 2   // get the second reading



SoftwareSerial myRock(RX_RCK, TX_RCK); // RX, TX
IridiumSBD isbd(myRock, SW_RCK);
SoftwareSerial myGPS(RX_GPS, TX_GPS); // RX, TX       
SoftwareSerial ol_serial(RX_OL, TX_OL); // RX, TX   // OL  

TinyGPS gps;
int err;
int signalQuality = 0 ;   //signal quality  for rockblock
byte ol_status=0;     //ol status
bool screen=0;   // connects OpenLog directly
bool logger=0;   //logs acceleromter
bool S_en=0;    //checked at beginning
bool gps_twiceloop=0;
time_t GPS_timed;       // global gps
time_t gf_time;
byte state=0;    // 1: in air    3: deployed    5: in water    7: recovered
//                  dummy, GF_time, RB_time, # GPS_time, Big_Sleep, Rock/GPS, No_sigsleep
int parameters[7]={   0,    300,      300,         300,         30,          1,       30};
bool ws=0, rb=0, gp=0, gf=0;  
int get_gps(float *latitude, float *longitude);
int command(char data);
void digitalClockDisplay();
void printDigits(int digits);
int ol_cmd();
int ol_append();
int ol_potbot_parameters(int num_parameters);
int rock_rx_parameters(int num_parameters, uint8_t rock_rx[]);
int sequence();
void ol_timestamp();
void ol_printDigits(char separator, int digits);
float latitude,longitude;
void ol_logevent(int event);
void myWatchdogEnable(int select_time) ;
void print_time(time_t temp1);
void rck_on_off(bool state);
void clear_buffer();
void LED_blink(int blinks, unsigned long delay_time);

void setup() {
  bool err_gps=0, err_rock=0, err_ol=0;
  pinMode(SW_GPS,OUTPUT);
  pinMode(SW_TOL,OUTPUT);
  pinMode(SW_RCK, OUTPUT);
  pinMode(I_HUM,INPUT);
  pinMode(LED,OUTPUT);
  pinMode(9,INPUT);
  digitalWrite(SW_TOL,LOW);     // Openlog ON
  command('G');                 //GPS OFF
  digitalWrite(SW_RCK,LOW);     //ROCK OFF

  analogReference(INTERNAL);

// Description

// Configures the reference voltage used for analog input (i.e. the value used as the top of the input range). The options are:

// DEFAULT: the default analog reference of 5 volts (on 5V Arduino boards) or 3.3 volts (on 3.3V Arduino boards)
// INTERNAL: an built-in reference, equal to 1.1 volts on the ATmega168 or ATmega328 and 2.56 volts on the ATmega8 (not available on the Arduino Mega)

  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  delay(100);
   for (unsigned long start = millis(); millis() - start < 2000;)
   {
     if(Serial) {S_en=1; break;}
   }
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  // set the data rate for the SoftwareSerial port
  myRock.begin(19200);   // 19200 rockblock
  myGPS.begin(4800);   // 4800 GPS
  ol_serial.begin(9600);
  delay (1000);

   if(Serial)
    Serial.println(F("Rockblock v 7.3"));
  LED_blink(5,100);
  delay(500);

  
  if (!ol_potbot_parameters(6))
    err_ol=1;
  state=0;

  // TEST GPS
  command('g');
  myGPS.listen();
  err_gps=1;
  for (unsigned long start = millis(); millis() - start < 2000;)
  {
   if (myGPS.available())
   {
   char c = myGPS.read();
   Serial.write(c); // uncomment this line if you want to see the GPS data flowing
   if (c=='G') // valid data?
      {Serial.println(F("GPS valid"));
      err_gps=0; break;}  //clear error flag
   }
  }
  command('G');


 ///////wait for command
  // Serial.println(F("Wait for command: 5s"));
  // char data;
  // for (unsigned long start = millis(); millis() - start < 2000;)
  // {
  // if (Serial.available()){
  //   data   = Serial.read();
  //   Serial.write(data);
  //   Serial.print(F("   :"));
  //   command(data);
  //   return;
  //   }
  // }
  // INIT ROCK BLOCK  
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  myRock.listen();
  isbd.setPowerProfile(1);
  int rock_init = isbd.begin();
  isbd.setMinimumSignalQuality(MIN_SIG_CSQ);   // set to 0 for continous send
  isbd.adjustATTimeout(AT_TIMEOUT);                       // Wait for response timeout
  isbd.adjustSendReceiveTimeout(SBDIX_TIMEOUT);               // send SBDIX loop timout

  if (rock_init) 
  {
    err_rock=1; Serial.println(F("Rock error!"));
  }else 
    {err_rock=0; Serial.println(F("Rock OK!"));}

  delay(1000);
  // LED error indicator
  if (err_ol | err_gps | err_rock)
  {
    if (err_ol)    // OL EROR
      {LED_blink(3,250); delay(1000);}
    if (err_gps)
      {LED_blink(5,250); delay(1000);}  // GPS
    if (err_rock)
      {LED_blink(8,200); delay(1000);}   //ROCK
  }
  Serial.println(F("Self test passed!!"));
  LED_blink(1,2000);
  pinMode(LED,INPUT);
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  //arduino   PWR sace SLEEP_MODE_PWR_SAVE
  Serial.println(F("Sleep mode set"));
}

void loop() {
  // Serial.println(F("Begin loop"));
  // Serial.println(F("screen"));
  char data;
  static bool cur_hum, prev_hum;
  
  if ((now()>gf_time) && (gf))
    {gf=0; Serial.println("GF expired");}

  while (screen)                        // OPENLONG SCREEN INTERFACE
       {  
        ol_serial.listen();
        if (ol_serial.available()) {
        Serial.write(ol_serial.read());
        }
        if (Serial.available()) {
        ol_serial.write(Serial.read());
         }
      }
  // Serial.println(F("After screen"));
          // DEBUG interface
      if (Serial.available() && (data   = Serial.read()) != 13){
      Serial.print("new command");
      Serial.write(data);
      Serial.print("   :");
      command(data);
      }
  // Serial.println(F("After debug"));
      // high priority log humdity sensor
      cur_hum=digitalRead(I_HUM);
      if (cur_hum!=prev_hum) 
        {
          // if (cur_hum==1) ol_logevent(9);    // E9 in air
          // else ol_logevent(8);               //E8 in water
          prev_hum=cur_hum;
        }
  // Serial.println(F("After log hum"));
      delay(10)  ;
      sequence();
      delay(10)  ;
      myWatchdogEnable(8);
      // Serial.println("sleep.");
      delay(10);
      sleep_mode();
}


/// get_gps:
//Read GPS for a second and 
//return 1: valid sentence received 
//return 0: no valid
// latitude and longitude as floats
//SYNC TIME
int get_gps(float *latitude, float *longitude)
{
  static byte gps_thresh=0;     // threshold of valid readings
   myGPS.listen();
   bool newData = false;
   int year;
   byte month, day, hour, minutes, second, hundredths;
   unsigned long fix_age;
   
   // For one second we parse GPS data and report some key values
  Serial.println("Waiting for GPS");
  
   
   for (unsigned long start = millis(); millis() - start < 2000;)
   {
     while (myGPS.available())
     {
     char c = myGPS.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
     if (gps.encode(c)) // Did a new valid sentence come in?
     {
      newData = true;
      gps_thresh++;
     }
     }
   }

   if (newData && gps_thresh>GPS_THRESHOLD) // threshold # of good sentences
   {
    unsigned long age;
    gps.f_get_position(latitude, longitude, &age);
    gps.crack_datetime(&year, &month, &day,
    &hour, &minutes, &second, &hundredths, &fix_age);
    setTime(hour,minutes,second,day,month,year);
    setTime(now()+28800);
    Serial.print("GPS received, messages:");
    Serial.println(gps_thresh,DEC);
        
    gps_thresh=0;
    
   gp=1; gf=1;
   gf_time=now() + parameters[1];
    return 1;
   }
return 0;
  
 }
/////////////////////////////////////////////////////////
/////////////// POTBOT CODE /////////////////////////////
/////////////////////////////////////////////////////////

int command(char data){
  switch (data){

    case 'T': 
      digitalWrite(SW_TOL, HIGH);
      Serial.print(digitalRead(SW_TOL),DEC);
      Serial.print(" LOGGER OFF \n");
      break;

    case 'o': 
      //ol_cmd();
      break;

    case 'g': 
      digitalWrite(SW_GPS, LOW);
      Serial.println("GPS on...");
  break;
  
  case 'G': 
      digitalWrite(SW_GPS, HIGH);
      Serial.println("GPS off...");
  break;
      
    case 'h':
      Serial.print(digitalRead(I_HUM),DEC);
      Serial.print("  hum\n");
      break;


    case 'w':
      digitalWrite(SW_TOL, LOW);
      Serial.print("Open log writing test\n");
      delay(1000);
      for(int i = 1 ; i < 10 ; i++) {
      ol_serial.print(i, DEC);     
      ol_serial.println(":abcdefsdff-!#");
      delay(100);
      }
      
      delay(1000);
      Serial.print("finish writing\n");
      digitalWrite(SW_TOL, HIGH);  

      break;

     case 's':                                   //screen
     digitalWrite(SW_TOL, LOW);
     ol_serial.listen();
     ol_status=0;
     ol_cmd();
     state=100;
     screen=1;
     Serial.println("SCREEN OL");
     break;


    case 'r':
    EEPROM.write(EEPROM_addrs, 0);
    Serial.print("  reset logger file to new01.txt\n");
    break;

    case 13:
    break;
      
    default:
      Serial.print("  error\n");
      break;
    }
  return 1;
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}


void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void ol_timestamp(){
  // digital clock display of the time
  ol_printDigits(0,day());
  ol_printDigits('/',month());
  ol_printDigits('/',year());
  ol_printDigits(';',hour());
  ol_printDigits(':',minute());
  ol_printDigits(':',second());
  ol_serial.print(';'); 
}

// separator = 0 --> doesn't print
void ol_printDigits(char separator, int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  if (separator) ol_serial.write(separator);
  if(digits < 10)
    ol_serial.print('0');
  ol_serial.print(digits);
}


int ol_cmd(){
  char temp;
  digitalWrite(SW_TOL, LOW);
  Serial.print(digitalRead(SW_TOL),DEC);
  Serial.print("Open command mode\n");
  ol_serial.listen();
  delay(50);
  for (int i=0; ; i++){
    delay(1);
    ol_serial.write(26);
    delay(1);
    ol_serial.write(26);
    delay(1);
    ol_serial.write(26);
    delay(10);
    for (int z=0; z<10 ; z++)
      if (ol_serial.available())
        //Serial.println(ol_serial.read());
        if(ol_serial.read()=='>')
        { Serial.print("attempt: "); Serial.print(i, DEC); Serial.println(">");   delay(5); 
          ol_serial.println("");
          for (int j=0; j<200; j++)          //flush
            if (ol_serial.available())
              {temp=ol_serial.read(); 
              //Serial.print(temp);
              }
          delay(5);
          Serial.println("");
          Serial.println("OL CMD_MODE");
          ol_status=1; 
          return 1;
        }
      delay(5);
      if (i>=250) { Serial.println("error openlong cmd"); return 0;}
    }     
    
} 

int ol_append(){
  if (ol_status!=2)
  {
      ol_cmd();
      ol_serial.println("");
      delay(10);
      ol_serial.println("append log.txt");
      delay(10);
      Serial.println("Openlong ready");
      ol_status=2;
      return 1;
  }
}


int ol_potbot_parameters(int num_parameters){
  int  par_index=0;  
  char character;
  int temp=0;
  ol_cmd();
  delay(3000);
  ol_serial.println("read potbot");
  for (int i=0; i< 200; i++) {
  if (ol_serial.available()) 
  {
    character= ol_serial.read(); //Serial.write(character);
    Serial.print(character);    //uncomment to see openlog parameters load
    if (par_index>num_parameters) //end of line
    {
    Serial.println("\tparameters loaded");  ol_serial.println("reset"); return 1; break;
    }   //LOW
    if ((character>= '0') &&  (character<= '9'))
      temp= temp*10 + character - '0';
    else if (character==',')
    {
      parameters[par_index]=temp;
      par_index ++;
      temp=0;
      Serial.print("\t   paramter[");
      Serial.print(par_index-1);
      Serial.print("] :");
      Serial.println(parameters[par_index - 1]);
    }
   }
   delay(10);
  }
   Serial.println("\tError OL_parameters");
   digitalWrite(SW_TOL, HIGH);
   return 0;
}

int rock_rx_parameters(int num_parameters, uint8_t rock_rx[]){
  int  par_index=0;  
  char character;
  int temp=0;
  for (int i=0; i< 50; i++) {
  if (rock_rx[i]>40) 
  {
    character= rock_rx[i];
    Serial.println(character);    //uncomment to see openlog parameters load
    if (par_index>num_parameters) //end of line
    {
    Serial.println("\tparameters loaded"); return 1; break;
    }   //LOW
    if ((character>= '0') &&  (character<= '9'))
      temp= temp*10 + character - '0';
    else if (character==',')
    {
      parameters[par_index]=temp;
      par_index ++;
      temp=0;
      Serial.print("\t   paramter[");
      Serial.print(par_index-1);
      Serial.print("] :");
      Serial.println(parameters[par_index - 1]);
    }
   }
   delay(10);
  }
   Serial.println("\tError OL_parameters");
   digitalWrite(SW_TOL, HIGH);
   return 0;
}

int sequence(){
  // Serial.print(F("Init sequence:"));
  // Serial.println(state);  
  static time_t temp1, temp2;     //global timers
  String str_rock= " ";     //Create String and Capture current time
  uint8_t rock_rx[20];
  size_t rock_rx_buff_size;
  int batt;
  float voltage;

  switch(state){  
  case 0:
      ws=0; rb=0; gp=0;   //init variables  except from gf
      Serial.print("Start at"); print_time(now());
      digitalWrite(SW_GPS, HIGH);  //stops gps
      digitalWrite(SW_TOL, HIGH);   //openlog off;
      rck_on_off(0);   // Rockblock off
      Serial.println("waiting wetswitch ...");

      state=10;
      break;

  case 10:
      
      if (digitalRead(I_HUM))       // HUM --> AIR
      {  Serial.print("Turn GPS and Rockblock ON...");
      state=1;
      }
  break;

  case 1:

      // String str_rock= " ";     //Create String and Capture current time
       // str_rock= str_rock + "Time: " + String(day()) + "/" + String(month()) + "/" + String(year()) + "   " + String(hour())+ ":" + String(minute()) + ":" + String(second());
       // str_rock= str_rock + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
    ////////////////////////////////////
      // Send RockBlock
    //////////////////////////////////////
      batt = analogRead(V_BATT);
      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
      voltage = batt * (1.1/1023.0)*3.125;
      //str_rock= str_rock + "T: " + String(day()) + "/" + String(month()) + "/" + String(year()) + " " + String(hour())+ ":" + String(minute()) + ":" + String(second()) + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6) + " Battery:" +  String(voltage,2) ;
      str_rock= str_rock + " " + String(day()) + "/" + String(month()) + "/" + String(year()) + " " + String(hour())+ ":" + String(minute()) + ":" + String(second()) + ";" + String(latitude,6) + ";"+ String(longitude,6) + ";" +  String(voltage,2) ;
      Serial.println(str_rock);
      char charBuf[80];
      str_rock.toCharArray(charBuf, 80) ;
      Serial.println("Message to send SBD:");
      Serial.println(charBuf);
      rck_on_off(1);   // start Rockblock
      command('g');  //GPS ON
      myRock.listen();
      delay(100);
      err = isbd.sendReceiveSBDText(charBuf, rock_rx, rock_rx_buff_size);
      ol_logevent(1);
      if (err != 0)
      {
        Serial.print("sendSBDText failed: error ");
        Serial.println(err);
      } else if (err==0)                    //TX Successfully 
        {
          ol_logevent(3); rb=1;
          Serial.println("Msg sent confirmed!");
          Serial.print("Messages left: ");
          Serial.println(isbd.getWaitingMessageCount());
          rck_on_off(0);   // Rockblock off
          delay(2000);
        // if (err==0 && rock_rx[0]>48 && rock_rx_buff_size>1)
        if (rock_rx_buff_size>1)
          {
            Serial.println("\t\t\t...MSG RX MSG RX MSG RX....");
            Serial.println((char)rock_rx);
            Serial.println(rock_rx_buff_size);
            rock_rx_parameters(6,rock_rx);
          }

          if (!gp)
            ol_logevent(7);
          else
            ol_logevent(5);

          if((gf) || (gps_twiceloop))
          {
            rck_on_off(0);   // Rockblock off
            command('G'); //GPS OFF
            Serial.println("All done going to sleep!");
            Serial.println(F("BIG SLEEP \t BIG SLEEP\t BIG SLEEP\tBIG SLEEP"));
            // temp1=now() + (int)60*parameters[4];    // Big Sleep
            temp1=now() + parameters[4];    // Big Sleep
            gps_twiceloop=0;
            state=5;
          }else
          {
            Serial.println("Not good fix, try again!");
            gps_twiceloop=1;
            state=1;
          }

          break;
        }


      //if (digitalRead(I_HUM))       // wait hum
      if (1)       // wait hum
      { 
        if (gf)
          {
            Serial.print("GF OK--> Burst RB, GPS off...");
            ol_logevent(6);
            
          }
        else 
          {
            Serial.print("GF NOT OK-->Burst RB, GPS ON...");
            ol_logevent(2);
            command('g');  //GPS ON
          }
        get_gps(&latitude, &longitude);
////////////////  New message every time
        // temp1=now() + 60*(int)parameters[2];    // RB time
        temp1=now() + parameters[2];    // RB time
        print_time(temp1);
        // temp2=now() + 60*(int)parameters[3];    // GPS time
        temp2=now() + parameters[3];    // GPS time
        print_time(temp2);
        //Serial.println("To sleep");  rck_on_off(0); delay(2000);
        //Serial.println(F("XXX wait to turn on:"));
        Serial.println(F("Not tx on first go"));
        state++;
      }
      break;

  case 2:               // send message rock block + GPS   GOTO 3 or 4
  if (rb && gf)          //evaluate GPS and RB
      {
        Serial.println(F("All done going to sleep!"));
        // temp1=now() + 60*parameters[4];    // Big Sleep
        rck_on_off(0);   // Rockblock off
        command('G'); //GPS OFF
        Serial.println(F("BIG SLEEP \t BIG SLEEP\t BIG SLEEP\tBIG SLEEP"));
        temp1=now() + parameters[4];    // Big Sleep
        print_time(now());print_time(temp1);
        if (!gp)
          ol_logevent(7);
        else
          ol_logevent(5);
        state=5;
        break;
      }

      ///////////
    if ((now()> temp1) && (now()> 0))    //   evaluate RB time out
      {
        clear_buffer();
        rck_on_off(0);   // Rockblock off
        print_time(temp1);
        print_time(now());
        Serial.println(F("\t\tRCK timed out"));
      }
    if ((now()> temp2) && (now()> 0))   //   evaluate GPS time out
      {
        command('G');            
        print_time(temp2);   //GPS off
        print_time(now());
        Serial.println(F("\t\tGPS timed out"));
      }
    
    if ((now()> temp1) && (now()> temp2))  //   evaluate overall time out
      {
        // temp1=now() + 60*(int)parameters[6];    // Sleep no signal
        ol_logevent(10); state=5;
        clear_buffer();
        delay(2000);
        rck_on_off(0);   // Rockblock off
        command('G'); //GPS OFF
        if (digitalRead(I_HUM))       // HUM --> AIR
        {
          Serial.println(F("No signal going to small_sleep"));
          Serial.println(F("SMALL SLEEP \t SMALL SLEEP\t SMALL SLEEP\tSMALL SLEEP"));
          temp1=now() + parameters[6];    // Sleep no signalQuality
        }
        else
        {
          Serial.println(F("No signal + in water sleep!"));
          // temp1=now() + 60*parameters[4];    // Big Sleep
          Serial.println(F("BIG SLEEP \t BIG SLEEP\t BIG SLEEP\tBIG SLEEP"));
          temp1=now() + parameters[4];    // Big Sleep
        }

        print_time(now()); print_time(temp1);
        break;
      }

      /////////////////////
      // String str_gps= " ";     //Create String and Capture current time
      // str_gps= str_gps + "  Loc: " + String(latitude,6) + ", "+ String(longitude,6);
      // Serial.println(str_gps);
////////////// RCKBCK SBDIX!!!!
      rck_on_off(1);   // start Rockblock
      myRock.listen();

      Serial.print("Try SBDIX");

      err = isbd.try_SBDIX(rock_rx, rock_rx_buff_size);
      if (err != 0)
      {
        Serial.print(F("sendSBDText failed: error "));
        Serial.println(err);
      }
      if (err==0)                    //TX Successfully 
      {
        ol_logevent(3); rb=1;
        Serial.println(F("Msg sent confirmed!"));
        Serial.print(F("Messages left: "));
        Serial.println(isbd.getWaitingMessageCount());
        //RX  TEST
        if (rock_rx_buff_size>1)
          {
            Serial.println("\t\t\t...MSG RX MSG RX MSG RX....");
            Serial.println((char)rock_rx);
            Serial.println(rock_rx_buff_size);
            rock_rx_parameters(6,rock_rx);
          }
          //RX  TEST
        rck_on_off(0);   // Rockblock off
      }

      if (get_gps(&latitude, &longitude))
        {
           ol_logevent(4);             //GET_GPS
           command('G');                //turn GPS off
           gp=1; gf=1;
           gf_time=now() + parameters[1];
           // gf_time=now() + parameters[1]*60;
           clear_buffer();
           state=1;
           break;
        }
    break;
    
  case 5:
      //print_time(now());
      if (now()> temp1)  //   Sleep
      {
        print_time(now());
        Serial.println("Back to beginning");
        state=0;
        break;
      }
      break;
  
      
  }  // switch end 
    // Serial.println(F("Finish sequence"));
}// function sequence() end
// logs event timestamp and position

void ol_logevent(int event)
{
      ol_serial.listen();
      digitalWrite(SW_TOL,LOW);
      delay(2000);
      ol_timestamp();
      ol_serial.print("E");
      ol_serial.print(event);
      ol_serial.print(";");
      ol_serial.print(latitude,6);
      ol_serial.print(';');
      ol_serial.print(longitude,6);
      ol_serial.println("");
      delay(4000);
      digitalWrite(SW_TOL,HIGH);

}

void myWatchdogEnable(int select_time) {  // turn on watchdog timer; interrupt mode every
  cli();

   MCUSR = 0;
  WDTCSR |= B00011000;     
  WDTCSR =  B01000110;     //1s    was 0110

  sei();
}

ISR(WDT_vect) {
 cli();
 wdt_disable();
 setTime(now()+1); // corect time
 // Serial.println(F("wakeup!"));
 sei();
}


void print_time(time_t temp1)
{
  tmElements_t tm;
  breakTime(temp1, tm);
  String str_time= " ";     //Create String and Capture current time
  str_time= str_time + "Time: " + String(tm.Day) + "/" + String(tm.Month) + "/" + String(tm.Day) + "   " + String(tm.Hour)+ ":" + String(tm.Minute) + ":" + String(tm.Second);
  Serial.println(str_time);
}

void rck_on_off(bool state)
{
    delay(2000);
    if (state)
    {
      Serial.println("Rck ON");  digitalWrite(SW_RCK, HIGH);  
    }
    else
    {
      Serial.println("Rck OFF");  digitalWrite(SW_RCK, LOW);   
    }
    delay(2000);
}

void clear_buffer()
{
  delay(10);   //CLEAR BUFFER
  rck_on_off(1);   // start Rockblock
  myRock.listen();

  Serial.println(F("XXX start clear buffer:"));
  err=isbd.clear_buffer(0);
  if (err != 0)
    {
      Serial.print(F("Clear Buffer failed: error "));
      Serial.println(err);
    }else
    {
      Serial.print(F("Clear Buffer Successfully "));
    }
  delay(100);
}

void LED_blink(int blinks, unsigned long delay_time)
{
   for (int i=0; i<blinks; i++)
   {
    digitalWrite(LED, HIGH);
    delay(delay_time);
    digitalWrite(LED, LOW);
    delay(delay_time);
  }
}

// Openlog
// 9600,26,3,1,1,1,0
// baud,escape,esc#,mode,verb,echo,ignoreRX