/* L&An0R v0.13 Home Sweeper Robot by Nearlyt*/
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

//   B A T T E R Y    M O N I T O R I N G    &    M A N A G E M E N T
  int sensorPin = A10;             // PIN used to monitor battery voltage (2x1MOhm (tbc) resistor voltage divider + lower 220nF (tbc) capacitor to allow measure (resistors too big, not enough current).
  int batLevel = 2; //2 is good, 1 is low, 0 is stop and shout.
  unsigned long sensorValue = 0;
  unsigned long Endtimer3 = 0;
  int tryHarder = 0; // Set to 1 to force operation even if battery is considered low (IDE USB link for instance)

//   P R O X I M I T Y     S E N S O R      (FRONT CENTER ONLY)
    #define trigPin 16    // PIN used for emitting the echo
    #define echoPin 14    // PIN used for capturing the echo
    long duration, distance; unsigned long Endtimer2;
    // "Stuck robot" state detection through the rolling record of distance measures. If all are the same = robot is stuck.
    int record[] = {1,2,3,4,5,5,5,5,5,5};
   
//   C O L L I S I O N     S E N S O R      (FRONT SIDES ONLY)
    int leftBump = 8;    // PIN used for left bumper sensor
    int rightBump = 4;   // PIN used for right bumper sensor
    int leftState = 0;
    int rightState = 0;

//   M O T O R    D R I V E
  //motor A connected between A01 and A02
  //motor B connected between B01 and B02
  int STBY = 7;          // PIN used for standby
  //Motor A
  int PWMA = 6;          // PIN used for Speed control 
  int AIN1 = A1;         // PIN used for Direction
  int AIN2 = A0;         // PIN used for Direction
  //Motor B
  int PWMB = 9;          // PIN used for Speed control
  int BIN1 = A2;         // PIN used for Direction
  int BIN2 = A3;         // PIN used for Direction
  // Speed definitions :
  int ff = 255;  int f = 128; int s = 64; int ss = 32;
  // Rotation definitions at normal speed (i.e. f : 128, not 255!) :
  unsigned long smallT = 400; unsigned long quarterT = 2000;
  unsigned long thirdT = 3000; unsigned long halfT = 4000;

// R O B O T    B E H A V I O R
  unsigned long Starttimer; unsigned long Endtimer; unsigned long Present; unsigned long ligneDroite = 10000;
  int antiCol = 0; // variable to trigger anticollision behavior
  // 0 is normal, 1 is front proximity alert, 2 is left collision, 3 is right collision
  int state = 0; // variable to trigger drive mode
  // 0 is full forward, 1 is right angle right hand turn, 2 is small forward, 3 is right angle right hand turn again
  int okGo = 0; // is used to go back to normal behavior after anticollision - when conditions are met.
  int whichWay = 0;
    
// H U M A N     I N T E R F A C E
//##LED BUTTON##
  int buttonLed = 5;    // PIN used for the blue LED button
  unsigned long endBlink; unsigned long startBlink;
  int blinkState = 1;

 //##BUZZER##
  int buzzerPin = 15;    // PIN used for the buzzer
  int length = 32; // the number of notes of Tiger song
  char notes[] = "ccccdffdcfdcffdcfdcfdcfdcffgagdc "; // a space represents a rest
  int beats[] = { 1,1,1,1,1,1,2,1,1,2,1,1,1,1,1,1,2,1,1,2,1,1,2,1,1,1,1,1,1,2,1,4 };
  int tempo = 150;

 //##LED MATRIX##
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
static const uint8_t PROGMEM
  smile_bmp[] =
  { B01111110,
    B10000001,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B10000001,
    B01111110 },
    smilebat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000000,
    B11111111,
    B11011011,
    B11100111,
    B01111110 },
  right_bmp[] =
  { B01111110,
    B10000001,
    B11001001,
    B10000001,
    B10111001,
    B10000001,
    B10000001,
    B01111110 },
  rightbat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000000,
    B11111111,
    B10000111,
    B11111111,
    B01111110 },
  left_bmp[] =
  { B01111110,
    B10000001,
    B10010011,
    B10000001,
    B10011101,
    B10000001,
    B10000001,
    B01111110 },
  leftbat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000000,
    B11111111,
    B11100001,
    B11111111,
    B01111110 },
  frown_bmp[] =
  { B01111110,
    B10000001,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B10000001,
    B01111110 },
  frownbat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000000,
    B11111111,
    B11100111,
    B11011011,
    B01111110 },
  stuck_bmp[] =
  { B01111110,
    B11111111,
    B10011001,
    B10011001,
    B11111111,
    B10101011,
    B11010101,
    B01111110 },
  stuckbat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000000,
    B11111111,
    B10101011,
    B11010101,
    B01111110 },
  bat_bmp[] =
  { B00000000,
    B01100010,
    B10010111,
    B10010010,
    B10010000,
    B10010111,
    B11110000,
    B00000000,},
  done_bmp[] =
  { B00000000,
    B00000001,
    B00000011,
    B00000110,
    B10001100,
    B11011000,
    B01110000,
    B00100000,},
  donebat_bmp[] =
  { B01000000,
    B11100111,
    B01000000,
    B00000011,
    B00000110,
    B10001100,
    B11011000,
    B01110000,};
    
void setup() {
  //   D E B U G
    Serial.begin(9600);
    beacon();

  //   B A T T E R Y    M O N I T O R I N G    &    M A N A G E M E N T
    pinMode(sensorPin, INPUT);
    Present = 1; 
    checkbat(); // STARTUP BATTERY CHARGE CHECK

  //   P R O X I M I T Y     S E N S O R
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Endtimer2=Starttimer + 1000;
    distance = 500;

  //   C O L L I S I O N     S E N S O R
    pinMode(leftBump , INPUT);
    pinMode(rightBump, INPUT);

  //   M O T O R     D R I V E
    pinMode(STBY, OUTPUT); 
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    
  //   S T A R T U P     A N I M A T I O N   (button led, matrix and buzzer sound) 
    pinMode(buttonLed, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    startup(); // SIMULTANEOUS LED AND MATRIX STARTUP ANIMATION
    
  //   R O B O T     B E H A V I O R
    Starttimer=millis(); 
    Endtimer=Starttimer + ligneDroite;
    
} //end setup void

void loop() {

  Present = millis();
  checkbat(); // BATTERY CHARGE MONITORING
  jobsdone();
  distme(); // FRONT CENTER PROXIMITY CHECK
 
  // D E B U G
    debugme();

  // C O L L I S I O N    C H E C K  (FRONT SIDES CONTACT ONLY)
  leftState = digitalRead(leftBump);
  rightState = digitalRead(rightBump);
  if      ( leftState == HIGH) { stop(); frown(); antiCol = 2; state = 0; Starttimer=millis(); Endtimer=Starttimer + quarterT;}
  else if (rightState == HIGH) { stop(); frown(); antiCol = 3; state = 0; Starttimer=millis(); Endtimer=Starttimer + quarterT;} 

  // B A C K    T O    N O R M A L     C H E C K 
  if ( okGo == 1) {antiCol = 0; state=0; Starttimer=millis(); Endtimer=Starttimer + ligneDroite; analogWrite(buttonLed, 255); matrix.blinkRate(0); okGo = 0;}

  // R O B O T   D R I V E
  if (antiCol == 0) { // ##NORMAL## sweeping behavior
    if (state == 0) { // Normal forward movement
        smile();
        distcheck(); 
        move(1, ff, 1); //motor 1, full speed, forward
        move(2, ff, 1); //motor 2, full speed, forward
    }  // end state if
    else if (state == 1) { turn(); } // end state if
    else if (state == 2){ // Slow forward movement
        smile();
        distcheck();
        move(1, f, 1); //motor 1, half speed, forward
        move(2, f, 1); //motor 2, half speed, forward
    }  // end state if
    else if (state == 3){ turn(); } // end state if   
    if (Endtimer <= Present){
      if      (state==0) { state=1; Starttimer=millis(); Endtimer=Starttimer + quarterT;}
      else if (state==1) { state=2; Starttimer=millis(); Endtimer=Starttimer + (ligneDroite/4);}
      else if (state==2) { state=3; Starttimer=millis(); Endtimer=Starttimer + quarterT;}
      else               { state=0; Starttimer=millis(); Endtimer=Starttimer + ligneDroite; 
      } // end state if
    } // end endtimer if
  } // end antiCol if  
////////////////////////
  else if (antiCol == 1) { // ##CENTER## anti-collision behavior (when a front center proximity is detected)
    blinky();
    if (state == 0) { // Backwards turning movement around left wheel
        move(1, 0, 0); //motor 1 stops
        move(2, s, 0); //motor 2, slow speed, BACKWARDS
    }  // end state if
    else if (state == 1) { // Forward turning movement around RIGHT wheel
        distcheck();
        move(1, s, 1); //motor 1, slow speed, forward
        move(2, 0, 0); //motor 2 stops
    } // end state if
    if (Endtimer <= Present){
      if      (state==0) { state=1; Starttimer=millis(); Endtimer=Starttimer + quarterT;}
      else               { okGo = 1; 
      } // end state if
    } // end endtimer if
  } // end antiCol if  
////////////////////////
  else if (antiCol == 2) { // ##LEFT BUMP## anti-collision behavior (collision on the left side)
    blinky();
    if (state == 0) { // Small backwards turning movement around RIGHT wheel
        move(1, s, 0); //motor 1, slow speed, BACKWARDS
        move(2, 0, 0); //motor 2 stops
    }  // end state if
    else if (state == 1) { // Backwards turning movement around left wheel
        move(1, 0, 0); //motor 1 stops
        move(2, s, 0); //motor 2, slow speed, BACKWARDS
    } // end state if
    if (Endtimer <= Present){
      if      (state==0) { state=1; Starttimer=millis(); Endtimer=Starttimer + (quarterT*4);}
      else               { okGo = 1; 
      } // end state if
    } // end endtimer if
  } // end antiCol if  
////////////////////////
  else if (antiCol == 3) { // ##RIGHT BUMP## anti-collision behavior (collision on the right side) 
    blinky();
    if (state == 0) { // Small backwards turning movement around left wheel
        move(2, s, 0); //motor 2, slow speed, BACKWARDS
    }  // end state if
    else if (state == 1) { // Backwards turning movement around RIGHT wheel
        move(1, s, 0); //motor 1, slow speed, BACKWARDS
    } // end state if
    if (Endtimer <= Present){
      if      (state==0) { state=1; Starttimer=millis(); Endtimer=Starttimer + (quarterT*4);}
      else               { okGo = 1; 
      } // end state if
    } // end endtimer if
  } // end antiCol if
} // end loop void

//##  P  R  O  C  E  D  U  R  E  S  ##

//   D E B U G
void debugme(){
    Serial.print("battery : ");
    Serial.print(sensorValue);
    Serial.print("   distance : ");
    Serial.print(distance);
    Serial.print("   antiCol : ");
    Serial.print(antiCol);
    Serial.print("   state : ");
    Serial.print(state);
    Serial.print("   okGo : ");
    Serial.println(okGo);
    delay(500);
} // end void debugme

  //   B A T T E R Y    M O N I T O R I N G    &    M A N A G E M E N T
void checkbat(){
  if (Endtimer3 <= Present) {
  sensorValue = analogRead(sensorPin);     // read the battery voltage (directly CENTIvolts):
  if      (sensorValue >= 520 && sensorValue <=620) {batLevel = 1;}
  else if (sensorValue <= 519) {stop(); batLevel = 0; bat(); delay(500);
    if (tryHarder == 1) { stuck(); delay(500); beacon () ; beacon() ; }
   else { stop() ; while (digitalRead(leftBump) == LOW) { bat() ; delay(1000) ; matrix.clear() ; delay(4000) ; beacon() ; }
            tryHarder = 1;}}
  else {batLevel = 2;}
    Starttimer=millis(); Endtimer3=Starttimer + 20000;
  } // end Endtimer3 if   
}

void jobsdone(){
  if (1200000 <= Present) {
  stop() ; while (digitalRead(leftBump) == LOW) { done() ; delay(1000) ; matrix.clear() ; delay(4000) ; beacon() ; }
  tryHarder = 1;}
 } // end Job Is Done if   
} end jobsdone void
 

//   P R O X I M I T Y     S E N S O R
void distme(){
    if (Endtimer2 <= Present) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1; // distance variable stored for the rest of the sketch
    if (distance >=700) {distance =700;} else if (distance <=5) {distance = 5;}
    stuckcheck();
    Starttimer=millis(); Endtimer2=Starttimer + 500;
  } // end Endtimer2 if   
}

void distcheck () {
if (distance <= 15) { stop(); frown(); antiCol = 1; state = 0; Starttimer=millis(); Endtimer=Starttimer + (quarterT*4);
    } // end distance if
}

void stuckcheck () {
int marker = 0;
    for ( int i = 0 ; i <= 9 ; i++ ) { 
      for ( int j = i ; j <= 9 ;  j++ ) {
      if ( record[i] == record[j] ) { delay(1); }
      else { marker = marker + 1; }
      } //end secondary for 
   } //end primary for
   if ( marker == 0 ) { // If all ten last distance measures recorded remained constant : the robot might be stuck !
      stop(); stuck(); for ( int k = 1 ; k <= 4 ;  k++ ) { tiger(); delay(10000); } // end tiger music for 
      while (digitalRead(leftBump) == LOW) { stuck(); delay(1000); matrix.clear(); delay(4000); beacon(); }
      tryHarder = 1;
   } // end marker if
   for ( int i = 1 ; i <= 9 ; i++ ) { 
        record[i-1] = record[j]
   } //end "re-record" for
        record[9] = distance;
} // end stuckcheck void 


//   M O T O R     D R I V E
void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise
  digitalWrite(STBY, HIGH); //disable standby
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if(motor == 2){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}

void randway(){  //  RANDOMLY GENERATES VARIABLE TO DECIDE WHICH WAY TO TURN NEXT
 whichWay = 0;  #### TO BE FINALIZED ####
}

void turn(){ // RANDOMLY TURNS LEFT OR RIGHT
        randway();
        if ( whichWay == 0 ) {
        right();
        move(1, f, 1); //motor 1, half speed, forward
        move(2, f, 0); } //motor 2, half speed, BACKWARDS
        else { 
        left();
        move(1, f, 0); //motor 1, half speed, BACKWARDS
        move(2, f, 1); } //motor 2, half speed, forward
}

//   M A T R I X    D I S P L A Y    A N D   B U T T O N    L E D

void startup(){ // SIMULTANEOUS LED AND MATRIX STARTUP ANIMATION
    matrix.begin(0x70);  // pass in the address
    matrix.setRotation(3);
    matrix.setBrightness(0);
    matrix.setTextSize(1);
    matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
    matrix.setTextColor(LED_ON);
    Int clock = 0;
    int8_t x=0;

    while ( x>=-67 ) { // Simultaneous button led and matrix animation
    if ( clock <= 255 ) { analogWrite(buttonLed, clock); } //end fade in if
    else if ( clock >= 256 && clock  <=  510) { analogWrite(buttonLed, 510 - clock); } //end fade out if
    else if ( clock >= 511 && clock  <=  700) { analogWrite(buttonLed,0); } //end blink step 1 if
    else if ( clock >= 701 && clock  <=  900) { analogWrite(buttonLed,255); } //end blink step 2 if
    else if ( clock >= 901 && clock  <= 1100) { analogWrite(buttonLed,0); } //end blink step 3 if
    else if ( clock >= 1101 && clock <= 1300) { analogWrite(buttonLed,255); } //end blink step 4 if
    if (x>=-66) { matrix.clear(); matrix.setCursor(x,0); matrix.print("- Coucou !"); matrix.writeDisplay(); }
    x--; delay(20); clock = clock + 20 ;
    } // end led and matrix animation while
    chime();

} // End void startup

void blinky(){ // COLLISION LED BLINK
  if (endBlink <= Present){
      if       (blinkState == 0) { analogWrite(buttonLed, 255); startBlink=millis(); endBlink=startBlink +  200; blinkState=1;}
      else if  (blinkState == 1) { analogWrite(buttonLed,   0); startBlink=millis(); endBlink=startBlink + 1000; blinkState=2;}
      else if  (blinkState == 2) { analogWrite(buttonLed, 255); startBlink=millis(); endBlink=startBlink +  200; blinkState=3;}
      else if  (blinkState == 3) { analogWrite(buttonLed,   0); startBlink=millis(); endBlink=startBlink +  200; blinkState=0;
      } // end blinkState if
  } // end endBlink if
}

void smile() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, smilebat_bmp, 8, 8, LED_ON);}
  else {matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);}
  matrix.writeDisplay();}
  
void right() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, rightbat_bmp, 8, 8, LED_ON);}
  else {matrix.drawBitmap(0, 0, right_bmp, 8, 8, LED_ON);}
  matrix.writeDisplay();}

void left() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, leftbat_bmp, 8, 8, LED_ON);}
  else {matrix.drawBitmap(0, 0, left_bmp, 8, 8, LED_ON);}
  matrix.writeDisplay();}

void frown() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, frownbat_bmp, 8, 8, LED_ON);}
  else {matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);}
  matrix.writeDisplay();
  matrix.blinkRate(1);}

void bat() {
matrix.clear();
matrix.drawBitmap(0, 0, bat_bmp, 8, 8, LED_ON);
matrix.writeDisplay();
matrix.blinkRate(3);}

void stuck() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, stuckbat_bmp, 8, 8, LED_ON);}
  else {matrix.drawBitmap(0, 0, stuck_bmp, 8, 8, LED_ON);}
  matrix.writeDisplay();
  matrix.blinkRate(3);} // end void stuck

void done() {
  matrix.clear();
  if (batLevel==1) {
  matrix.drawBitmap(0, 0, donebat_bmp, 8, 8, LED_ON) ; matrix.blinkRate(3) ; }
  else {matrix.drawBitmap(0, 0, done_bmp, 8, 8, LED_ON) ; matrix.blinkRate(1) ; }
  matrix.writeDisplay();
  } // end void stuck

//   B U Z Z E R    S O U N D S

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(buzzerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(buzzerPin, LOW);
    delayMicroseconds(tone);
  }
} end void playTone

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };

  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
    delay(5000);
  }
} // end void playNote

void tiger() { // PLay the Eye Of The Tiger from Katy Perry (in case of 
  for (int i = 0; i < length; i++) {
    if (notes[i] == ' ') {
      delay(beats[i] * tempo); // rest
    } else {
      playNote(notes[i], beats[i] * tempo);
    }
    // pause between notes
    delay(tempo / 2); 
  }
} // end void tiger

void beacon() { // distress beacon sound when robot is stuck or battery critically low
   for (long i = 0; i <= 1000; i) {
    digitalWrite(buzzerPin, HIGH);
    delayMicroseconds(1000-i);
    digitalWrite(buzzerPin, LOW);
    delayMicroseconds(1000-i);
  }
   for (long i = 0; i <= 1000; i) {
    digitalWrite(buzzerPin, HIGH);
    delayMicroseconds(i);
    digitalWrite(buzzerPin, LOW);
    delayMicroseconds(i);
  }
} // end void beacon

void chime() { // startup sound to signal beginning of main loop()
#### TO BE FINALIZED ####
// ##### ENTER BEACON SOUND CODE HERE ######
} // end void chime
