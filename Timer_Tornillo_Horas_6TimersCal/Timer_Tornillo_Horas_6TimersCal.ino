//Nicolas Jarpa
//Multiple Timer  RTC3231+LCD+Single Relay+Everyday+eeprom


#include <EEPROM.h>
#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);//0x3F is my lcd address, maybe not yours! (is a example)
RTC_DS3231 RTC;

unsigned long a=0;
int b=0;
int c=0;
//int t=62;
//int d=24;
//TODO: const int #_DE_PULSO_POR_REVOLUCION_1000 = 1000;   // 
const int drive_distance = 1;   // cm
const int motor_power = 255;      // 0-255
const int motor_offset = 0;       // Diff. when driving straight
const int wheel_d = 1;           // Wheel diameter (mm)
const float wheel_c = wheel_d; // Wheel circumference (mm)
unsigned char counts_per_rev = 0;   // (4 pairs N-S) * (48:1 gearbox) * (2 falling/rising edges) = 384
boolean flag1 = false;
boolean flag2 = false;
boolean flag3 = false;
boolean flag4 = false;
boolean flag5 = false;
//-----------------------//
//------Variables--------//
//-----------------------//

//------First Timer 
byte onhour1;
byte onmin1;
byte onsec1;

byte offhour1;
byte offmin1;
byte offsec1;
//-----Second Timer              //hour= 0-23
byte onhour2=0;
byte onmin2=0;                   //minutes,seconds= 0-59
byte onsec2=0;

byte offhour2=0;
byte offmin2=0;
byte offsec2=0;
//-----Third Timer
byte onhour3=0;
byte onmin3=0;
byte onsec3=0;

byte offhour3=0;
byte offmin3=0;
byte offsec3=0;
//-----Fourth Timer
byte onhour4=0;
byte onmin4=0;
byte onsec4=0;

byte offhour4=0;
byte offmin4=0;
byte offsec4=0;
//----Fifth Timer
byte onhour5=0;
byte onmin5=0;
byte onsec5=0;

byte offhour5=0;
byte offmin5=0;
byte offsec5=0;
//----Calibration



//------Pages or menus
int page_counter=1;
int subpage1_counter=0; //Timer 1 on/off
int subpage2_counter=0; //Timer 2 on/off
int subpage3_counter=0; //Timer 3 on/off
int subpage4_counter=0; //Timer 4 on/off
int subpage5_counter=0; //Timer 5 on/off
int subpage6_counter=0; // Calibration

//-------To convert clock into single number
unsigned long Time;
unsigned long Hour;
unsigned long Min;
unsigned long Sec;
//------To convert first timer into Single number
unsigned long on_Time1;
unsigned long on_hour1;
unsigned long on_min1;
unsigned long on_sec1;

unsigned long off_Time1;
unsigned long off_hour1;
unsigned long off_min1;
unsigned long off_sec1;
//------To convert second timer into Single number
unsigned long on_Time2;
unsigned long on_hour2;
unsigned long on_min2;
unsigned long on_sec2;

unsigned long off_Time2;
unsigned long off_hour2;
unsigned long off_min2;
unsigned long off_sec2;
//------To convert Third timer into Single number
unsigned long on_Time3;
unsigned long on_hour3;
unsigned long on_min3;
unsigned long on_sec3;

unsigned long off_Time3;
unsigned long off_hour3;
unsigned long off_min3;
unsigned long off_sec3;
//------To convert Fourth timer into Single number
unsigned long on_Time4;
unsigned long on_hour4;
unsigned long on_min4;
unsigned long on_sec4;

unsigned long off_Time4;
unsigned long off_hour4;
unsigned long off_min4;
unsigned long off_sec4;
//------To convert Fifth timer into Single number
unsigned long on_Time5;
unsigned long on_hour5;
unsigned long on_min5;
unsigned long on_sec5;

unsigned long off_Time5;
unsigned long off_hour5;
unsigned long off_min5;
unsigned long off_sec5;
//-------Push buttons current/last state 
boolean current_up = LOW;          
boolean last_up=LOW;            
boolean current_sel = LOW;
boolean last_sel = LOW;
boolean last_down = LOW;
boolean current_down = LOW;
//-------Pins
byte Relay =13;//Relay to pin 13

//------Relay States
boolean RelayState1;
boolean RelayState2;
boolean RelayState3;
boolean RelayState4;
boolean RelayState5;
boolean RelayState6;


int up=8;      //Up button to pin 2
int sel=9;     //Select button to pin 3
int down=10;   //Down button to pin 4

const int enc_l_pin = 2;          // Motor A
const int enc_r_pin = 3;          // Motor B
const int pwma_pin = 5;

volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;

//Custom return char
byte back[8] = {
  0b00100,
  0b01000,
  0b11111,
  0b01001,
  0b00101,
  0b00001,
  0b00001,
  0b11111
};

//Custom arrow char
byte arrow[8] = {
  0b01000,
  0b00100,
  0b00010,
  0b11111,
  0b00010,
  0b00100,
  0b01000,
  0b00000
};

void setup() { 

  //Serial.begin(9600);
  
  pinMode(Relay, OUTPUT);
  pinMode(enc_l_pin, INPUT_PULLUP);
  pinMode(enc_r_pin, INPUT_PULLUP);
  pinMode(pwma_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(enc_l_pin), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), countRight, CHANGE);
  delay(200);   
     Wire.begin();
     RTC.begin();
     //lcd.init();
     lcd.backlight();
     lcd.clear();
  lcd.createChar(1, back);//Custom chars
  lcd.createChar(2, arrow);    
//--------eePROM  read values-------//
//------First Timer 
onhour1=EEPROM.read(0);
onmin1=EEPROM.read(1);
onsec1=EEPROM.read(2);

offhour1=EEPROM.read(3);
offmin1=EEPROM.read(4);
offsec1=EEPROM.read(5);
//------Second Timer 
onhour2=EEPROM.read(6);
onmin2=EEPROM.read(7);
onsec2=EEPROM.read(8);

offhour2=EEPROM.read(9);
offmin2=EEPROM.read(10);
offsec2=EEPROM.read(11);
//------Third Timer 
onhour3=EEPROM.read(12);
onmin3=EEPROM.read(13);
onsec3=EEPROM.read(14);

offhour3=EEPROM.read(15);
offmin3=EEPROM.read(16);
offsec3=EEPROM.read(17);
//------Fourth Timer 
onhour4=EEPROM.read(18);
onmin4=EEPROM.read(19);
onsec4=EEPROM.read(20);

offhour4=EEPROM.read(21);
offmin4=EEPROM.read(22);
offsec4=EEPROM.read(23);
//------Fifth Timer
onhour5=EEPROM.read(24);
onmin5=EEPROM.read(25);
onsec5=EEPROM.read(26);

offhour5=EEPROM.read(27);
offmin5=EEPROM.read(28);
offsec5=EEPROM.read(29);

a=EEPROM.read(30);
counts_per_rev=EEPROM.read(31);

driveStraight(drive_distance, motor_power);

}//void setup

   //---- De-bouncing function for all buttons----//
boolean debounce(boolean last, int pin)
{
boolean current = digitalRead(pin);
if (last != current)
{
delay(10);
current = digitalRead(pin);
}
return current;
}

void countLeft() {
  enc_l++;
}

void countRight() {
  enc_r++;
}

void brake() {

  analogWrite(pwma_pin, 0);
 
}

void driveStraight(float dist, int power) {
 // b = a.toInt()*1;
  int power_l=motor_power;
  for (int i=0; i<a; i++){
  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = motor_power;
  int power_r = motor_power;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = enc_l;   
  unsigned long enc_r_prev = enc_r;

  // Calculate target number of ticks
  float num_rev = (dist * 1) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * (counts_per_rev*6+12500);//TODO: Aqui se va a desbordar
  
  // Debug
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");

  // Drive until one of the encoders reaches desired count
  while ( (enc_l < target_count) && (enc_r < target_count) ) {

    // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;

    // Print out current number of ticks
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);

    // Drive
    drive(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev;
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;


    delay(20);
  }  
 
  brake();
  delay(c);
 }
delay(200);

if (Time >= on_Time1 && Time < off_Time1)
{
 flag1=true;
}
else if (Time >= on_Time2 && Time < off_Time2) 
{
  flag2=true;
}
else if (Time >= on_Time3 && Time < off_Time3)
{
  flag3=true;
}
else if (Time >= on_Time4 && Time < off_Time4) 
{
  flag4=true;
}
else if (Time >= on_Time5 && Time < off_Time5) 
{
  flag5=true;
}
}

void drive(int power_a, int power_b) {

  
  power_a = constrain(power_a, -255, 255);

  analogWrite(pwma_pin, abs(power_a));
  
}

void loop() {
  
current_up = debounce(last_up, up);         //Debounce for Up button
current_sel = debounce(last_sel, sel);      //Debounce for Select  button
current_down = debounce(last_down, down);   //Debounce for Down button  
  
DateTime now = RTC.now();        // Clock call
now = RTC.now();

//-----Up/Down functions to move main pages------///   
 
if(subpage1_counter==0 && subpage2_counter==0 && subpage3_counter==0 && subpage4_counter==0 && subpage5_counter==0 && subpage6_counter==0){ //up/down buttons enabled if subpages counters are 0,Disabled if 1,2..etc to work on submenus
  
//Page Up
    if (last_up== LOW && current_up == HIGH){ //Up button pressed
      lcd.clear();                            //Clear lcd if page is changed to print new one
      if(page_counter <8){                   //Page counter never higher than 8(total of pages)
      page_counter ++;                       //Page up      
      }
      else{
      page_counter= 1;                       //If higher than 3 (last page)go to main page
      }
  
} 
last_up = current_up;                   //Save up button last state 

//Page Down
    if (last_down== LOW && current_down == HIGH){//Down button pressed
      lcd.clear();                               //Clear lcd if page is changed to print new one
      if(page_counter >1){                      //Page counter never lower than 1 
      page_counter --;                          //Page down
      
      }
      else{
      page_counter= 8;                 //If lower than 1(first page)go to last page
      }
  }    
    last_down = current_down;         //Save down button last state
}  
//------------Pages and submenus display and control----------//  
  switch (page_counter){
    case 1:                      //Content of main page
     last_sel=current_sel;  //Save last state of select button when we jump from the save screen
      lcd.setCursor(0,0);
      lcd.print("CONCENTRADO TEMP");
      lcd.setCursor(0,1);
      lcd.print("HORA");
      
//--------Show  Time On LCD

lcd.setCursor(7,1);                 
if(now.hour() < 10)
{
lcd.print("0");
}
lcd.print(now.hour(), DEC); //Print hour
lcd.print(':');
if(now.minute() < 10)
{
lcd.print("0");
}
lcd.print(now.minute(), DEC); //Print min
lcd.print(':');
if(now.second() < 10)
{
lcd.print("0");
}
lcd.print(now.second(), DEC); //Print sec
    //case 1
    break;

    case 2:                   //Content and functions of page 2
      lcd.setCursor(0,0);
      lcd.print("T1");
      lcd.setCursor(3,0);
      lcd.print("ON");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("OFF");

            lcd.setCursor(7,0);    //Printing on/off values
            if(onhour1<10){
              lcd.print("0");
            }
            lcd.print(onhour1);
            lcd.setCursor(10,0);
            if(onmin1<10){
              lcd.print("0");
            }
            lcd.print(onmin1);
            lcd.setCursor(13,0);
            if(onsec1<10){
              lcd.print("0");
            }
            lcd.print(onsec1);
            lcd.setCursor(7,1);
            if(offhour1<10){
              lcd.print("0"); 
            }
            lcd.print(offhour1);
            lcd.setCursor(10,1);
            if(offmin1<10){
              lcd.print("0");
            }
            lcd.print(offmin1);
            lcd.setCursor(13,1);
            if(offsec1<10){
              lcd.print("0");
            }
            lcd.print(offsec1);
                        
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage1_counter <7){                    // subpage counter never higher than 7 (total of items)
     subpage1_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage1_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage_counter =1) onhour1
     if(subpage1_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(6,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onhour1 < 23){
     onhour1 ++;
      }
      else{
     onhour1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onhour1 >0){
      onhour1 --; 
     }
     else{
      onhour1=23;
     }
     }
     last_down=current_down;
    }//subpage1_counter 1

     //Second item control(subpage_counter =2) onmin1
     if(subpage1_counter==2){
     lcd.setCursor(6,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onmin1 < 59){
     onmin1 ++;
      }
      else{
     onmin1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onmin1 >0){
      onmin1 --; 
     }
     else{
      onmin1=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 2
    
     //Thirth item control(subpage_counter =3) onsec1
     if(subpage1_counter==3){
     lcd.setCursor(9,0);         //Delete last arrow position (onmin1)
     lcd.print(" ");                                        
     lcd.setCursor(12,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onsec1 < 59){
     onsec1 ++;
      }
      else{
     onsec1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onsec1 >0){
      onsec1 --; 
     }
     else{
      onsec1=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 3

     //fourth item control(subpage_counter =4) offhour1
     if(subpage1_counter==4){
     lcd.setCursor(12,0);         //Delete last arrow position (onsec1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offhour1 < 23){
     offhour1 ++;
      }
      else{
     offhour1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offhour1 >0){
      offhour1 --; 
     }
     else{
      offhour1=23;
     }
     }
     last_down=current_down;
    }//subpage1_counter 4

     //fifth item control(subpage_counter =5) offmin1
     if(subpage1_counter==5){
     lcd.setCursor(6,1);         //Delete last arrow position (offhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offmin1 < 59){
     offmin1 ++;
      }
      else{
     offmin1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offmin1 >0){
      offmin1 --; 
     }
     else{
      offmin1=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 5 

     //sixth item control(subpage_counter =6) offsec1
     if(subpage1_counter==6){
     lcd.setCursor(9,1);         //Delete last arrow position (offmin1)
     lcd.print(" ");                                         
     lcd.setCursor(12,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offsec1 < 59){
     offsec1 ++;
      }
      else{
     offsec1 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offsec1 >0){
      offsec1 --; 
     }
     else{
      offsec1=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 6 
          

     //seventh item control(subpage_counter =7) back
     if(subpage1_counter==7){
     lcd.setCursor(12,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage1_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage1_counter=1;         //Go to first item (onhour1)
     }
     last_down=current_down;
    }//subpage1_counter 7
    //case 2
    break;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case 3:                   //Content and functions of page 3
      lcd.setCursor(0,0);
      lcd.print("T2");
      lcd.setCursor(3,0);
      lcd.print("ON");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("OFF");

            lcd.setCursor(7,0);    //Printing on/off values
            if(onhour2<10){
              lcd.print("0");
            }
            lcd.print(onhour2);
            lcd.setCursor(10,0);
            if(onmin2<10){
              lcd.print("0");
            }
            lcd.print(onmin2);
            lcd.setCursor(13,0);
            if(onsec2<10){
              lcd.print("0");
            }
            lcd.print(onsec2);
            lcd.setCursor(7,1);
            if(offhour2<10){
              lcd.print("0"); 
            }
            lcd.print(offhour2);
            lcd.setCursor(10,1);
            if(offmin2<10){
              lcd.print("0");
            }
            lcd.print(offmin2);
            lcd.setCursor(13,1);
            if(offsec2<10){
              lcd.print("0");
            }
            lcd.print(offsec2);
                        
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage2_counter <7){                    // subpage counter never higher than 7 (total of items)
     subpage2_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage2_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage2_counter =1) onhour2
     if(subpage2_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(6,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onhour2 < 23){
     onhour2 ++;
      }
      else{
     onhour2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onhour2 >0){
      onhour2 --; 
     }
     else{
      onhour2=23;
     }
     }
     last_down=current_down;
    }//subpage2_counter 1

     //Second item control(subpage2_counter =2) onmin2
     if(subpage2_counter==2){
     lcd.setCursor(6,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onmin2 < 59){
     onmin2 ++;
      }
      else{
     onmin2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onmin2 >0){
      onmin2 --; 
     }
     else{
      onmin2=59;
     }
     }
     last_down=current_down;
    }//subpage2_counter 2
    
     //Thirth item control(subpage2_counter =3) onsec2
     if(subpage2_counter==3){
     lcd.setCursor(9,0);         //Delete last arrow position (onmin1)
     lcd.print(" ");                                        
     lcd.setCursor(12,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onsec2 < 59){
     onsec2 ++;
      }
      else{
     onsec2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onsec2 >0){
      onsec2 --; 
     }
     else{
      onsec2=59;
     }
     }
     last_down=current_down;
    }//subpage2_counter 3

     //fourth item control(subpage2_counter =4) offhour2
     if(subpage2_counter==4){
     lcd.setCursor(12,0);         //Delete last arrow position (onsec1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offhour2 < 23){
     offhour2 ++;
      }
      else{
     offhour2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offhour2 >0){
      offhour2 --; 
     }
     else{
      offhour2=23;
     }
     }
     last_down=current_down;
    }//subpage2_counter 4

     //fifth item control(subpage2_counter =5) offmin2
     if(subpage2_counter==5){
     lcd.setCursor(6,1);         //Delete last arrow position (offhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offmin2 < 59){
     offmin2 ++;
      }
      else{
     offmin2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offmin2 >0){
      offmin2 --; 
     }
     else{
      offmin2=59;
     }
     }
     last_down=current_down;
    }//subpage2_counter 5 

     //sixth item control(subpage2_counter =6) offsec2
     if(subpage2_counter==6){
     lcd.setCursor(9,1);         //Delete last arrow position (offmin1)
     lcd.print(" ");                                         
     lcd.setCursor(12,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offsec2 < 59){
     offsec2 ++;
      }
      else{
     offsec2 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offsec2 >0){
      offsec2 --; 
     }
     else{
      offsec2=59;
     }
     }
     last_down=current_down;
    }//subpage2_counter 6 
          

     //seventh item control(subpage2_counter =7) back
     if(subpage2_counter==7){
     lcd.setCursor(12,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage2_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage2_counter=1;         //Go to first item (onhour2)
     }
     last_down=current_down;
    }//subpage2_counter 7
    //case 3
    break;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case 4:                   //Content and functions of page 4
      lcd.setCursor(0,0);
      lcd.print("T3");
      lcd.setCursor(3,0);
      lcd.print("ON");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("OFF");

            lcd.setCursor(7,0);    //Printing on/off values
            if(onhour3<10){
              lcd.print("0");
            }
            lcd.print(onhour3);
            lcd.setCursor(10,0);
            if(onmin3<10){
              lcd.print("0");
            }
            lcd.print(onmin3);
            lcd.setCursor(13,0);
            if(onsec3<10){
              lcd.print("0");
            }
            lcd.print(onsec3);
            lcd.setCursor(7,1);
            if(offhour3<10){
              lcd.print("0"); 
            }
            lcd.print(offhour3);
            lcd.setCursor(10,1);
            if(offmin3<10){
              lcd.print("0");
            }
            lcd.print(offmin3);
            lcd.setCursor(13,1);
            if(offsec3<10){
              lcd.print("0");
            }
            lcd.print(offsec3);
                        
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage3_counter <7){                    // subpage counter never higher than 7 (total of items)
     subpage3_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage3_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage3_counter =1) onhour3
     if(subpage3_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(6,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onhour3 < 23){
     onhour3 ++;
      }
      else{
     onhour3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onhour3 >0){
      onhour3 --; 
     }
     else{
      onhour3=23;
     }
     }
     last_down=current_down;
    }//subpage3_counter 1

     //Second item control(subpage3_counter =2) onmin3
     if(subpage3_counter==2){
     lcd.setCursor(6,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onmin3 < 59){
     onmin3 ++;
      }
      else{
     onmin3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onmin3 >0){
      onmin3 --; 
     }
     else{
      onmin3=59;
     }
     }
     last_down=current_down;
    }//subpage3_counter 2
    
     //Thirth item control(subpage3_counter =3) onsec3
     if(subpage3_counter==3){
     lcd.setCursor(9,0);         //Delete last arrow position (onmin1)
     lcd.print(" ");                                        
     lcd.setCursor(12,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onsec3 < 59){
     onsec3 ++;
      }
      else{
     onsec3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onsec3 >0){
      onsec3 --; 
     }
     else{
      onsec3=59;
     }
     }
     last_down=current_down;
    }//subpage3_counter 3

     //fourth item control(subpage3_counter =4) offhour3
     if(subpage3_counter==4){
     lcd.setCursor(12,0);         //Delete last arrow position (onsec1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offhour3 < 23){
     offhour3 ++;
      }
      else{
     offhour3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offhour3 >0){
      offhour3 --; 
     }
     else{
      offhour3=23;
     }
     }
     last_down=current_down;
    }//subpage3_counter 4

     //fifth item control(subpage3_counter =5) offmin3
     if(subpage3_counter==5){
     lcd.setCursor(6,1);         //Delete last arrow position (offhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offmin3 < 59){
     offmin3 ++;
      }
      else{
     offmin3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offmin3 >0){
      offmin3 --; 
     }
     else{
      offmin3=59;
     }
     }
     last_down=current_down;
    }//subpage3_counter 5 

     //sixth item control(subpage3_counter =6) offsec3
     if(subpage3_counter==6){
     lcd.setCursor(9,1);         //Delete last arrow position (offmin1)
     lcd.print(" ");                                         
     lcd.setCursor(12,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offsec3 < 59){
     offsec3 ++;
      }
      else{
     offsec3 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offsec3 >0){
      offsec3 --; 
     }
     else{
      offsec3=59;
     }
     }
     last_down=current_down;
    }//subpage3_counter 6 
          

     //seventh item control(subpage_counter =7) back
     if(subpage3_counter==7){
     lcd.setCursor(12,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage3_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage3_counter=1;         //Go to first item (onhour3)
     }
     last_down=current_down;
    }//subpage3_counter 7
    //case 4
    break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case 5:                   //Content and functions of page 5
      lcd.setCursor(0,0);
      lcd.print("T4");
      lcd.setCursor(3,0);
      lcd.print("ON");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("OFF");

            lcd.setCursor(7,0);    //Printing on/off values
            if(onhour4<10){
              lcd.print("0");
            }
            lcd.print(onhour4);
            lcd.setCursor(10,0);
            if(onmin4<10){
              lcd.print("0");
            }
            lcd.print(onmin4);
            lcd.setCursor(13,0);
            if(onsec4<10){
              lcd.print("0");
            }
            lcd.print(onsec4);
            lcd.setCursor(7,1);
            if(offhour4<10){
              lcd.print("0"); 
            }
            lcd.print(offhour4);
            lcd.setCursor(10,1);
            if(offmin4<10){
              lcd.print("0");
            }
            lcd.print(offmin4);
            lcd.setCursor(13,1);
            if(offsec4<10){
              lcd.print("0");
            }
            lcd.print(offsec4);
                        
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage4_counter <7){                    // subpage counter never higher than 7 (total of items)
     subpage4_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage4_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage4_counter =1) onhour4
     if(subpage4_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(6,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onhour4 < 23){
     onhour4 ++;
      }
      else{
     onhour4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onhour4 >0){
      onhour4 --; 
     }
     else{
      onhour4=23;
     }
     }
     last_down=current_down;
    }//subpage4_counter 1

     //Second item control(subpage4_counter =2) onmin4
     if(subpage4_counter==2){
     lcd.setCursor(6,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onmin4 < 59){
     onmin4 ++;
      }
      else{
     onmin4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onmin4 >0){
      onmin4 --; 
     }
     else{
      onmin4=59;
     }
     }
     last_down=current_down;
    }//subpage4_counter 2
    
     //Thirth item control(subpage4_counter =3) onsec4
     if(subpage4_counter==3){
     lcd.setCursor(9,0);         //Delete last arrow position (onmin1)
     lcd.print(" ");                                        
     lcd.setCursor(12,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onsec4 < 59){
     onsec4 ++;
      }
      else{
     onsec4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onsec4 >0){
      onsec4 --; 
     }
     else{
      onsec4=59;
     }
     }
     last_down=current_down;
    }//subpage4_counter 3

     //fourth item control(subpage4_counter =4) offhour4
     if(subpage4_counter==4){
     lcd.setCursor(12,0);         //Delete last arrow position (onsec1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offhour4 < 23){
     offhour4 ++;
      }
      else{
     offhour4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offhour4 >0){
      offhour4 --; 
     }
     else{
      offhour4=23;
     }
     }
     last_down=current_down;
    }//subpage4_counter 4

     //fifth item control(subpage4_counter =5) offmin4
     if(subpage4_counter==5){
     lcd.setCursor(6,1);         //Delete last arrow position (offhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offmin4 < 59){
     offmin4 ++;
      }
      else{
     offmin4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offmin4 >0){
      offmin4 --; 
     }
     else{
      offmin4=59;
     }
     }
     last_down=current_down;
    }//subpage4_counter 5 

     //sixth item control(subpage4_counter =6) offsec4
     if(subpage4_counter==6){
     lcd.setCursor(9,1);         //Delete last arrow position (offmin1)
     lcd.print(" ");                                         
     lcd.setCursor(12,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offsec4 < 59){
     offsec4 ++;
      }
      else{
     offsec4 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offsec4 >0){
      offsec4 --; 
     }
     else{
      offsec4=59;
     }
     }
     last_down=current_down;
    }//subpage4_counter 6 
          

     //seventh item control(subpage_counter =7) back
     if(subpage4_counter==7){
     lcd.setCursor(12,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage4_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage4_counter=1;         //Go to first item (onhour4)
     }
     last_down=current_down;
    }//subpage4_counter 7
    //case 5
    break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case 6:                   //Content and functions of page 2
      lcd.setCursor(0,0);
      lcd.print("T5");
      lcd.setCursor(3,0);
      lcd.print("ON");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("OFF");

            lcd.setCursor(7,0);    //Printing on/off values
            if(onhour5<10){
              lcd.print("0");
            }
            lcd.print(onhour5);
            lcd.setCursor(10,0);
            if(onmin5<10){
              lcd.print("0");
            }
            lcd.print(onmin5);
            lcd.setCursor(13,0);
            if(onsec5<10){
              lcd.print("0");
            }
            lcd.print(onsec5);
            lcd.setCursor(7,1);
            if(offhour5<10){
              lcd.print("0"); 
            }
            lcd.print(offhour5);
            lcd.setCursor(10,1);
            if(offmin5<10){
              lcd.print("0");
            }
            lcd.print(offmin5);
            lcd.setCursor(13,1);
            if(offsec5<10){
              lcd.print("0");
            }
            lcd.print(offsec5);
                        
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage1_counter <7){                    // subpage counter never higher than 7 (total of items)
     subpage1_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage1_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage_counter =1) onhour1
     if(subpage1_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(6,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onhour5 < 23){
     onhour5 ++;
      }
      else{
     onhour5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onhour5 >0){
      onhour5 --; 
     }
     else{
      onhour5=23;
     }
     }
     last_down=current_down;
    }//subpage1_counter 1

     //Second item control(subpage_counter =2) onmin1
     if(subpage1_counter==2){
     lcd.setCursor(6,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onmin5 < 59){
     onmin5 ++;
      }
      else{
     onmin5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onmin5 >0){
      onmin5 --; 
     }
     else{
      onmin5=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 2
    
     //Thirth item control(subpage_counter =3) onsec1
     if(subpage1_counter==3){
     lcd.setCursor(9,0);         //Delete last arrow position (onmin1)
     lcd.print(" ");                                        
     lcd.setCursor(12,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(onsec5 < 59){
     onsec5 ++;
      }
      else{
     onsec5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(onsec5 >0){
      onsec5 --; 
     }
     else{
      onsec5=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 3

     //fourth item control(subpage_counter =4) offhour1
     if(subpage1_counter==4){
     lcd.setCursor(12,0);         //Delete last arrow position (onsec1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offhour5 < 23){
     offhour5 ++;
      }
      else{
     offhour5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offhour5 >0){
      offhour5 --; 
     }
     else{
      offhour5=23;
     }
     }
     last_down=current_down;
    }//subpage1_counter 4

     //fifth item control(subpage_counter =5) offmin1
     if(subpage1_counter==5){
     lcd.setCursor(6,1);         //Delete last arrow position (offhour1)
     lcd.print(" ");                                         
     lcd.setCursor(9,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offmin5 < 59){
     offmin5 ++;
      }
      else{
     offmin5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offmin5 >0){
      offmin5 --; 
     }
     else{
      offmin5=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 5 

     //sixth item control(subpage_counter =6) offsec1
     if(subpage1_counter==6){
     lcd.setCursor(9,1);         //Delete last arrow position (offmin1)
     lcd.print(" ");                                         
     lcd.setCursor(12,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(offsec5 < 59){
     offsec5 ++;
      }
      else{
     offsec5 =0;
      }
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(offsec1 >0){
      offsec5 --; 
     }
     else{
      offsec5=59;
     }
     }
     last_down=current_down;
    }//subpage1_counter 6 
          

     //seventh item control(subpage_counter =7) back
     if(subpage1_counter==7){
     lcd.setCursor(12,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage1_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage1_counter=1;         //Go to first item (onhour1)
     }
     last_down=current_down;
    }//subpage1_counter 7
    //case 6
    break;
        case 7:                   //Content and functions of page 2
      lcd.setCursor(0,0);
      lcd.print("Calibracion");
      lcd.setCursor(12,0);
      lcd.print("a");
      lcd.setCursor(1,1);
      lcd.write(byte(1));
      lcd.setCursor(3,1);
      lcd.print("PPR");

            lcd.setCursor(14,0);    //Printing a and PPR
            if(a<10){
              lcd.print("0");
            }
            lcd.print(a);
            lcd.setCursor(7,1);
            if(counts_per_rev<10){
            lcd.print("7500");//TODO: GESTIONAR ESTE COMPORTAMIENTO, FUENTE DE ERROR
            }
            lcd.print(counts_per_rev*6+12500);
//--------------Modifying on/off values-------//
     // Sub counter control
     if (last_sel== LOW && current_sel == HIGH){ //select button pressed
      if(subpage1_counter <3){                    // subpage counter never higher than 7 (total of items)
     subpage1_counter ++;                         //subcounter to move beetwen submenu
     }
     else{                                       //If subpage higher than 7 (total of items) return to first item
      subpage1_counter=1;
     }
     }
     last_sel=current_sel;                      //Save last state of select button

     //First item control(subpage_counter =1) onhour1
     if(subpage1_counter==1){
     lcd.setCursor(0,1);         //Delete last arrow position (back)
     lcd.print(" ");                                         
     lcd.setCursor(13,0);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if(a < 10){
     a ++;
      }
      else{
     a =0;
      }
     }
     last_up=current_up; 

    
     if(last_down== LOW && current_down == HIGH){//Down
     if(a >0){
      a --; 
     }
     else{
      a=10;
     }
     }
     last_down=current_down;
    }//subpage1_counter 1
     //Second item control(subpage_counter =2) onmin1
     if(subpage1_counter==2){
     lcd.setCursor(13,0);         //Delete last arrow position (onhour1)
     lcd.print(" ");                                         
     lcd.setCursor(6,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up 
      if((counts_per_rev*6+12500) < 14000){
     counts_per_rev += 1;
      }
      else{
     counts_per_rev =255;
      }
     }
     
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
     if(counts_per_rev*6+12500 > 12500){
      counts_per_rev -= 1;
     }
     else{
      counts_per_rev=0;
     }
     }
     last_down=current_down;
    }//subpage1_counter 2
     if(subpage1_counter==3){
     lcd.setCursor(6,1);         //Delete last arrow position (offsec1)
     lcd.print(" ");                                         
     lcd.setCursor(0,1);          //Place arrow in front of selected item
     lcd.write(byte(2));         
     //Move item + or -
     if (last_up== LOW && current_up == HIGH){  //Up
      lcd.setCursor(0,1);         //Delete last arrow position (back) to exit
      lcd.print(" "); 
     subpage1_counter=0;         //Exit submenu. Up/down butons enabled to move main pages     
     }
     last_up=current_up;
     
     if(last_down== LOW && current_down == HIGH){//Down
      lcd.setCursor(0,1);         //Delete last arrow position (back)
      lcd.print(" "); 
     subpage1_counter=1;         //Go to first item (onhour1)
     }
     last_down=current_down;
    }//subpage1_counter 7
     break;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    case 8:                //Page 3 display and functions
      lcd.setCursor(2,0);
      lcd.print("PRESIONE SEL");
      lcd.setCursor(2,1);
      lcd.print("PARA GUARDAR");
     if (last_sel== LOW && current_sel == HIGH){  //select button pressed.Save settings to eeprom
  EEPROM.write(0, onhour1);
  EEPROM.write(1, onmin1);
  EEPROM.write(2, onsec1);
  EEPROM.write(3, offhour1);
  EEPROM.write(4, offmin1);
  EEPROM.write(5, offsec1);

  EEPROM.write(6, onhour2);
  EEPROM.write(7, onmin2);
  EEPROM.write(8, onsec2);
  EEPROM.write(9, offhour2);
  EEPROM.write(10, offmin2);
  EEPROM.write(11, offsec2);

  EEPROM.write(12, onhour3);
  EEPROM.write(13, onmin3);
  EEPROM.write(14, onsec3);
  EEPROM.write(15, offhour3);
  EEPROM.write(16, offmin3);
  EEPROM.write(17, offsec3);
  
  EEPROM.write(18, onhour4);
  EEPROM.write(19, onmin4);
  EEPROM.write(20, onsec4);
  EEPROM.write(21, offhour4);
  EEPROM.write(22, offmin4);
  EEPROM.write(23, offsec4);  

  
  EEPROM.write(24, onhour5);
  EEPROM.write(25, onmin5);
  EEPROM.write(26, onsec5);
  EEPROM.write(27, offhour5);
  EEPROM.write(28, offmin5);
  EEPROM.write(29, offsec5);

  EEPROM.write(30, a);
EEPROM.write(31, counts_per_rev);
    
  lcd.clear();                 //Print message "SAVED!"
  lcd.setCursor(2,1);
  lcd.print("GUARDADO!");
  delay(2000);
  lcd.clear();               //Clear lcd and go to main page
  page_counter=1;
     }
  last_sel=current_sel;  //Save last state of select button 
      
    //Case 6
    break;
  }//switch



//-------------Conversion----------//

//---------Converting clock time into single number 
 
Hour = now.hour();
Min = now.minute();
Sec = now.second();
Time = (Hour*10000+ Min*100 +Sec*1);  

//--------Converting firt timer on/off into single number
on_hour1=onhour1;
on_min1=onmin1;
on_sec1=onsec1;
on_Time1=(on_hour1*10000 + on_min1*100 + on_sec1);

off_hour1=offhour1;
off_min1=offmin1;
off_sec1=offsec1;
off_Time1=(off_hour1*10000 + off_min1*100 + off_sec1);
   
//--------Converting second timer on/off into single number
on_hour2=onhour2;
on_min2=onmin2;
on_sec2=onsec2;
on_Time2=(on_hour2*10000 + on_min2*100 + on_sec2);

off_hour2=offhour2;
off_min2=offmin2;
off_sec2=offsec2;
off_Time2=(off_hour2*10000 + off_min2*100 + off_sec2);
//--------Converting third timer on/off into single number
on_hour3=onhour3;
on_min3=onmin3;
on_sec3=onsec3;
on_Time3=(on_hour3*10000 + on_min3*100 + on_sec3);

off_hour3=offhour3;
off_min3=offmin3;
off_sec3=offsec3;
off_Time3=(off_hour3*10000 + off_min3*100 + off_sec3);   
//--------Converting fourth timer on/off into single number
on_hour4=onhour4;
on_min4=onmin4;
on_sec4=onsec4;
on_Time4=(on_hour4*10000 + on_min4*100 + on_sec4);

off_hour4=offhour4;
off_min4=offmin4;
off_sec4=offsec4;
off_Time4=(off_hour4*10000 + off_min4*100 + off_sec4);
//--------Converting fifth timer on/off into single number
on_hour5=onhour5;
on_min5=onmin5;
on_sec5=onsec5;
on_Time5=(on_hour5*10000 + on_min5*100 + on_sec5);

off_hour5=offhour5;
off_min5=offmin5;
off_sec5=offsec5;
off_Time5=(off_hour5*10000 + off_min5*100 + off_sec5);
//----Relay Function----//
if ((Time >= on_Time1 && Time < off_Time1) && (flag1==false)){
 a=a;
 c=500;
 driveStraight(1,100);
}
else if (Time >= off_Time1) {
 flag1=false;
}

if(onhour1 == offhour1 && onmin1==offmin1 && onsec1==offsec1){
   digitalWrite(Relay, LOW);
}

if(on_Time1 < off_Time1){
  
             if(Time >= on_Time1 && Time < off_Time1){  //Start
             digitalWrite(Relay, HIGH);
             }
             else if(Time >= off_Time1) {
             digitalWrite(Relay, LOW);
             }
             else{
             digitalWrite(Relay, LOW);
             }
}
if (on_Time1 > off_Time1){

            if(Time >= on_Time1 && Time <= 235959){     //Start
            digitalWrite(Relay, HIGH);  
            }
            else if(Time < off_Time1 ){
            digitalWrite(Relay, HIGH);
            }
            else if(Time >= off_Time1 && Time < on_Time1){
            digitalWrite(Relay, LOW);  
            }
}
//----RelayState2 Function----//
if ((Time >= on_Time2 && Time < off_Time2) && (flag2==false)){
 a=a;
 c=500;
 driveStraight(1,100);
}
else if (Time >= off_Time2) {
 flag2=false; 
}

if(onhour2 == offhour2 && onmin2==offmin2 && onsec2==offsec2){
   RelayState2=LOW;
}

if(on_Time2 < off_Time2){
  
             if(Time >= on_Time2 && Time < off_Time2){  //Start
             RelayState2= HIGH;
             }
             else if(Time >= off_Time2) {
             RelayState2= LOW;
             }
             else{
             RelayState2= LOW;
             }
}
if (on_Time2 > off_Time2){

            if(Time >= on_Time2 && Time <= 235959){     //Start
            RelayState2= HIGH;  
            }
            else if(Time < off_Time2 ){
            RelayState2= HIGH;
            }
            else if(Time >= off_Time2 && Time < on_Time2){
            RelayState2= LOW;  
            }
}
//----RelayState3 Function----//
if ((Time >= on_Time3 && Time < off_Time3) && (flag3==false)){
 a=a;
 c=500;
 driveStraight(1,100);
}
else if (Time >= off_Time3) {
 flag3=false;  
}
if(onhour3 == offhour3 && onmin3==offmin3 && onsec3==offsec3){
   RelayState3=LOW;
}

if(on_Time3 < off_Time3){
  
             if(Time >= on_Time3 && Time < off_Time3){  //Start
             RelayState3= HIGH;
             }
             else if(Time >= off_Time3) {
             RelayState3= LOW;
             }
             else{
             RelayState3= LOW;
             }
}
if (on_Time3 > off_Time3){

            if(Time >= on_Time3 && Time <= 235959){     //Start
            RelayState3= HIGH;  
            }
            else if(Time < off_Time3 ){
            RelayState3= HIGH;
            }
            else if(Time >= off_Time3 && Time < on_Time3){
            RelayState3= LOW;  
            }
}
//----RelayState4 Function----//
if ((Time >= on_Time4 && Time < off_Time4) && (flag4==false)){
 a=a;
 c=500;
 driveStraight(1,100);
}
else if (Time >= off_Time4) {
 flag4=false;  
}
if(onhour4 == offhour4 && onmin4==offmin4 && onsec4==offsec4){
   RelayState4=LOW;
}

if(on_Time4 < off_Time4){
  
             if(Time >= on_Time4 && Time < off_Time4){  //Start
             RelayState4= HIGH;
             }
             else if(Time >= off_Time4) {
             RelayState4= LOW;
             }
             else{
             RelayState4= LOW;
             }
}
if (on_Time4 > off_Time4){

            if(Time >= on_Time4 && Time <= 235959){     //Start
            RelayState4= HIGH;  
            }
            else if(Time < off_Time4 ){
            RelayState4= HIGH;
            }
            else if(Time >= off_Time4 && Time < on_Time4){
            RelayState4= LOW;  
            }

}
//----RelayState5 Function----//            
if ((Time >= on_Time5 && Time < off_Time5) && (flag5==false)){
 a=a;
 c=500;
 driveStraight(1,100);
}
else if (Time >= off_Time5) {
 flag5=false;  
}
if(onhour5 == offhour5 && onmin5==offmin5 && onsec5==offsec5){
   RelayState4=LOW;
}

if(on_Time5 < off_Time5){
  
             if(Time >= on_Time5 && Time < off_Time5){  //Start
             RelayState5= HIGH;
             }
             else if(Time >= off_Time4) {
             RelayState5= LOW;
             }
             else{
             RelayState5= LOW;
             }
}
if (on_Time5 > off_Time5){

            if(Time >= on_Time5 && Time <= 235959){     //Start
            RelayState5= HIGH;  
            }
            else if(Time < off_Time5 ){
            RelayState5= HIGH;
            }
            else if(Time >= off_Time5 && Time < on_Time5){
            RelayState5= LOW;  
            }            
}

//-------Relay function
if(RelayState1 ==HIGH || RelayState2==HIGH || RelayState3==HIGH || RelayState4==HIGH || RelayState5==HIGH){
  digitalWrite(Relay,HIGH);  
}
else{
  digitalWrite(Relay,LOW);
}


}//void loop
