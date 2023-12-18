///////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Zoe Berg                                                                                             //
//  MENG 471                                                                                             //
//  Yale University, Fall 2023                                                                           //
//  This reads the cardid of an NFC chip and will run the head or arms of "Grogu" while playing a sound  // 
//  The white card runs the arm and the duck card runs the head and the lightsaber runs the other arm    //
//                                                                                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <SoftwareSerial.h>
#include "Adafruit_Soundboard.h"

//add in time 
unsigned long totalTime; //this is the total time 

//define pins for NFC and sound board 
#define PN532_IRQ   (2) //IRQ line for NFC shield 
#define PN532_RESET (4)  // reset for NFC shield 
#define SFX_TX 5 //sound board 
#define SFX_RX 6 //sound board
#define SFX_RST 0 //this is not actually plugged in 
//need to leave zero and one open because those are used for serial by arduino

SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX); //software serial for sound board 
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST); //pass  software serial to soundboard, 2nd argument=debug (not rly used), 3rd arg=reset pin

//for motor channel A 
int directionPinA = 12; //direction
int pwmPinA = 3; //speed
int brakePinA = 9; //brake

//for motor channel B 
int directionPinB = 13; //direction
int pwmPinB = 11; //speed
int brakePinB = 8; //brake

//for the buttons 
const int buttonPinArm = 10;  //the pin that the arm button is attached to
const int buttonPinHead = 7; //the pin that head button is attached to
//const int ledPin = 8;    //the pin that the LED is attached to as a physical check - not in use anymore 

//variables that are not constant 
int buttonState = 0; //current state of the button
int lastButtonState = 0; //previous state of the button

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); //for shield with an I2C connection

//declare motor functions
void runMotor(int&, int&, int&, float&, int&, int&); 
void runMotorCont(int&, int&, int&, float&, int&, int&); 


//setup //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);  //115200 used previously 
  nfc.begin(); 
  ss.begin(9600);

  //motor A is the left arm and the head 
  pinMode(directionPinA, OUTPUT); //12
  pinMode(pwmPinA, OUTPUT); //3
  pinMode(brakePinA, OUTPUT); //9

  //motor B is the right arm 
  pinMode(directionPinB, OUTPUT); //13
  pinMode(pwmPinB, OUTPUT); //11
  pinMode(brakePinB, OUTPUT); //8

  pinMode(buttonPinArm, INPUT); //init button pin for arm as input 
  pinMode(buttonPinHead, INPUT); //init button pin for head as input 
  //pinMode(ledPin, OUTPUT); //initialize the LED as an output

  //for variable duty function- set c char as HIGH for default
  char c[] = "HIGH"; 

  //increase volume (this is still really quiet! )
  Serial.println("Vol up...");
  uint16_t v ;
  if (! (v = sfx.volUp()) ) 
  {
    Serial.println("Failed to adjust");
  } else 
  {
    Serial.print("Volume: "); Serial.println(v);
  }
}

//main loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  //NFC chip- prep to read in UID 
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) 
  {
    if (uidLength == 4) //this just checks for a classic card 
    {
      uint32_t cardid = uid[0];
      cardid <<= 8; cardid |= uid[1]; cardid <<= 8; cardid |= uid[2]; cardid <<= 8; cardid |= uid[3]; 
      Serial.print("Seems to be a Mifare Classic card #"); //print out which card it is being scanned 
      Serial.println(cardid);

      if (cardid == 3041994145) //this is the duck (runs the head)
      {
        Serial.println("rubber dino-duck NFC for head");
        runMotor (directionPinA, brakePinA, pwmPinA, HIGH, 150, 200); //to get out of notch

        //play sound
        uint8_t n = 0; //plays sound zero 
        Serial.print("\nPlaying track #"); Serial.println(n);
        if (! sfx.playTrack((uint8_t)n) ) 
        {
          //Serial.println("Failed to play track?");
        }

        while (digitalRead(buttonPinHead) == HIGH) // while the button remains pressed in 
        {
          //digitalWrite(ledPin, HIGH); //as long as the button is pressed down, the led will stay on! 

          //run the motor - this goes for the duration that the button is pressed! 
          digitalWrite(brakePinA, LOW); //release brake 
          digitalWrite(directionPinA, HIGH); //high direction for head 
          analogWrite(pwmPinA, 150); //150 for duty
        } 

          //digitalWrite(ledPin, LOW); //led turns off as soon as the button is released 
          //stop motor
          digitalWrite(brakePinA, HIGH); //set brake to high to stop
          analogWrite(pwmPinA, 0); //set work duty for the motor to 0 (off)
          // Delay a little bit to avoid bouncing
          delay(50);
      }
      
      //this is the white card which runs the arm 
      if (cardid == 3735233578) 
      {
        Serial.println("white card NFC for left arm");
        runMotor (directionPinA, brakePinA, pwmPinA, LOW, 150, 200); //to get out of notch

        //play sound
        uint8_t n = 1; //plays sound 1 
        Serial.print("\nPlaying track #"); Serial.println(n);
        if (! sfx.playTrack((uint8_t)n) ) 
        {
          //Serial.println("Failed to play track?");
        }

        while (digitalRead(buttonPinArm) == HIGH) // while the button is pressed down 
        {
          //digitalWrite(ledPin, HIGH); //as long as the button is pressed down, the led will stay on! 
          //run the motor - this goes for the duration that the button is pressed! 
          digitalWrite(brakePinA, LOW); //release brake 
          digitalWrite(directionPinA, LOW); //low dir for the arm 
          analogWrite(pwmPinA, 100); //150 for the duty 
        } 

          Serial.println("off");
          //digitalWrite(ledPin, LOW); //led turns off as soon as the button is released 
          //stop motor
          digitalWrite(brakePinA, HIGH); //set brake to high to stop
          analogWrite(pwmPinA, 0); //set work duty for the motor to 0 (off)
          // Delay a little bit to avoid bouncing
          delay(50);
      }  

      //this is the lightsaber and runs the other arm (simple motor arm)
      if (cardid == 90187169) 
      {
        Serial.println("lightsaber NFC for right arm");

        //play sound
        uint8_t n = 2; //plays sound 1 
        Serial.print("\nPlaying track #"); Serial.println(n);
        if (! sfx.playTrack((uint8_t)n) ) 
        {
          //Serial.println("Failed to play track?");
        }

        //runs the arm at a varying duty cycle value 

        //time since porgram started in milliseconds  this is always going up 
        //we are controlling the pwm aka the work duty this is in hertz which is cycle per second! 
        int minimumDuty = 70; //60 for the geared motor 60 for current motor
        float outputDuty = 0; 
        int maximumDuty = 150;  
        float t=0 ;
        int distTraveled = 0; 
        int maximumTime = 6500; //milliseconds ! 

        digitalWrite(brakePinB, LOW); //release brake 

        totalTime = millis();
        Serial.println("total time:");
        Serial.println(totalTime);
        float timeAtStartOfRun = totalTime; //gets this value here only while totalTime increases throughout
        Serial.println("time at start of run should be the same as ^:");
        Serial.println(timeAtStartOfRun);
        Serial.println("");
        float funcResult; 
        char armDirection; 
        //if I printed it it would be the same as the total time

        while (t < maximumTime) 
        {
          //time calculations
          totalTime = millis();
          t = totalTime - timeAtStartOfRun; //this is the time we will use in the function this should be zero 
          float tSeconds = t/1000;
          float tMicroseconds = t/100; 

          //function one: this just goes up and down 
          //funcResult = (20 * sin(tMicroseconds/2)); 
          //function two: this shakes at the start and then just runs at a constant value
          //funcResult =  exp(-(tMicroseconds*tMicroseconds)) * (20 * sin(3 * tMicroseconds)); 
          //function three: this goes direction to direction with some slow down periods on them! 
          //funcResult = sin(3 * sin(3 * sin(sin(tMicroseconds/3)))) * 20; 
          //function four- one dir then opposite then fast in first direction but still fairly constant 
          //funcResult = 1+((cos(tMicroseconds/10) + (cos(0.2*tMicroseconds)/2) ) * exp(abs(tMicroseconds/10)/5)); 

          //current function in use: 
          funcResult = (20 * sin(tMicroseconds/2)); 
          //calculate the output duty 
          outputDuty = abs(funcResult) + minimumDuty; 
          
          //determine if the function will be HIGH (arm goes up) or LOW (arm goes down)
          if (funcResult > 0) {
            armDirection = HIGH; 
            char c[]= "HIGH";
          }

          if (funcResult <= 0) {
            armDirection = LOW; 
            char c[]= "LOW";
          }

          //run motor for 1 microsecond at the value from the function 
          runMotorCont (directionPinB, brakePinB, pwmPinB, armDirection, outputDuty, 1); 
        }

        Serial.println("run one complete! "); //for visual check of when time completes 
        Serial.println("");
        digitalWrite(brakePinB, HIGH); //set brake on 
        delay(1000);

      }                    
    }
    Serial.println(""); //add some space in the serial between scans 
  }
}

//functions ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//runs the motor in a direction for a specific time at a specific duty then goes to a full stop 
void runMotor (int directionPin, int brakePin, int pwmPin, char dir, int duty, int duration)
{
	digitalWrite(directionPin, dir); //high or low for dir 
  digitalWrite(brakePin, LOW); //release brake 
  analogWrite(pwmPin, duty); //number for the duty goes up to 255
  delay(duration); //number for delay Amount aka how long it runs for

  digitalWrite(brakePin, HIGH); //set brake to high to stop
        
  analogWrite(pwmPin, 0); //set work duty for the motor to 0 (off)
  delay(1000);
	return;
}

//runs the motor for a constant duration- used for variances in the PWM
void runMotorCont (int directionPin, int brakePin, int pwmPin, char dir, int duty, int duration)
{
	digitalWrite(directionPin, dir); //high or low for dir 
  analogWrite(pwmPin, duty); //number for the duty goes up to 255
  delay(duration); //number for delay Amount aka how long it runs for
	return;
}