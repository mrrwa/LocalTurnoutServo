// Arduino Sketch to provide local facia push button control of a turnout position driven by a servo
//
// Author               : Alex Shepherd     
// 
// License              : This is free software; you can redistribute it and/or modify it under the terms of the 
//                      : GNU Lesser General Public License as published by the Free Software Foundation;
//                      : either version 2.1 of the License, or (at your option) any later version.
//
// The sketch has different definitions for an Arduino UNO for easier development and a ATTiny85 for production.
//
// For the ATTiny85 use the ATTinyCore Arduino Core from here: https://github.com/SpenceKonde/ATTinyCore
//
// The Required I/O are defined below as follows:
// 
//   NORMAL_POS_AN_INP  : Analog input from TrimPOT to set the Normal  servo position
//   REVERSE_POS_AN_INP : Analog input from TrimPOT to set the Reverse servo position
//
//   TOGGLE_BUTTON      : Digital input with internal pull-up enabled for a momentary push button to GND to toggle turnout position
//
//   SERVO_OUTPUT       : Digital output to drive the servo position
//
//   LED_OUTPUT         : Digital output to drive two LEDs, one connected to VCC via a resistor and the other connectd to GND via a resistor
//                        so that one is on when the pin is HIGH and the other is on when the pin is LOW
//

#include <EEPROM.h>
#include <elapsedMillis.h>
#include <ButtonKing.h>

#if defined(ARDUINO_AVR_UNO)
#include <Servo.h>

// Uncomment the next line to enable Debug Print
#define ENABLE_SERIAL_DEBUG

#define NORMAL_POS_AN_INP   A0
#define REVERSE_POS_AN_INP  A1
#define TOGGLE_BUTTON_PIN   4   
#define SERVO_OUTPUT        5
#define LED_OUTPUT          LED_BUILTIN

#elif defined(ARDUINO_AVR_ATTINYX5) 
#include <Servo_ATTinyCore.h>

#define NORMAL_POS_AN_INP   A2
#define REVERSE_POS_AN_INP  A3
#define TOGGLE_BUTTON_PIN   2
#define SERVO_OUTPUT        1
#define LED_OUTPUT          0

#else
#error "Unspecified or Unsupported Board"
#endif

#define MILLIS_PER_SERVO_STEP 20
#define MILLIS_PER_LED_FLASH_WHILE_SERVO_MOVES 50
#define MILLIS_PER_LED_FLASH_UNKNOWN_POS 1000
#define CALIBRATION_TIMEOUT_MILLIS  60000

// The last Turnout Position is stored in EEPROM at the index defined below
#define LAST_TURNOUT_POS_EEPROM_INDEX 0

// Uncomment the next line to enable Servo Detach after X ms since the last movement
#define SERVO_DETACH_MILLIS 1000

enum TURNOUT_POS{
  NORMAL_TURNOUT_POS = 0,
  REVERSE_TURNOUT_POS = 1,
  UNKNOWN_TURNOUT_POS = 0xFF  // Default value for unprogrammed or Erased EEPROM
};

TURNOUT_POS lastTurnoutPos;

Servo myservo;
int16_t lastServoPos = -1;
int16_t newServoPos = -1;

boolean enableCalibrateMode = false;

elapsedMillis sinceServoMoved = 0;
elapsedMillis sinceLEDChanged = 0;
elapsedMillis sinceCalibrationEnabled = 0;

ButtonKing button(TOGGLE_BUTTON_PIN, HIGH, true);

int16_t getServoPositionSetting(TURNOUT_POS turnoutPos)
{
  int16_t retServoPos = -1; // Default to Unknown 

  if(turnoutPos != UNKNOWN_TURNOUT_POS)
  {     
    int16_t potVal = (turnoutPos == NORMAL_TURNOUT_POS) ? analogRead(NORMAL_POS_AN_INP) : analogRead(REVERSE_POS_AN_INP);
    retServoPos = map(potVal, 0, 1023, 0, 180);
  }
  
#ifdef ENABLE_SERIAL_DEBUG
  Serial.print("getServoPositionSetting: Turnout Pos: ");
  Serial.print( turnoutPos == UNKNOWN_TURNOUT_POS ? "Unknown: " : (turnoutPos == NORMAL_TURNOUT_POS) ? "Normal: " : "Reverse: ");
  Serial.print(" Servo Pos: ");
  Serial.println(retServoPos);
#endif
    
  return   retServoPos;
}


void setServoPos(int16_t servoPos)
{
#ifdef ENABLE_SERIAL_DEBUG
  Serial.print( "setServoPos: " ); Serial.println( servoPos );
#endif

  if( servoPos >= 0 )
  {
    if(!myservo.attached())
      myservo.attach(SERVO_OUTPUT);
      
    myservo.write(servoPos);
  
    sinceServoMoved = 0;
  }
}

void setup() {
#ifdef ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("\nLocal Turnout Servo Driver");
#endif

  button.setClick(handleButtonClick);
  button.setLongDoubleStop(handleLongDoublePress);  // Trigger on Release
  pinMode(LED_OUTPUT, OUTPUT);

    // Uncomment the line below to test the initial unknown Turnout Position handling
//  EEPROM.update(LAST_TURNOUT_POS_EEPROM_INDEX, 0xFF);

  lastTurnoutPos = (TURNOUT_POS) (EEPROM.read(LAST_TURNOUT_POS_EEPROM_INDEX));

    // If we know what position the turnout was previously in then set it to that again 
  if(lastTurnoutPos != UNKNOWN_TURNOUT_POS)
  {
    newServoPos = getServoPositionSetting( lastTurnoutPos );
    lastServoPos = newServoPos;
    setServoPos( newServoPos );
  }
}

  
void loop() {
  button.isClick();
 
  if(enableCalibrateMode)
  {
    if(sinceCalibrationEnabled >= CALIBRATION_TIMEOUT_MILLIS)
      enableCalibrateMode = false;
    else
    {
      newServoPos = getServoPositionSetting( lastTurnoutPos );
      lastServoPos = newServoPos;

#ifdef ENABLE_SERIAL_DEBUG
      Serial.print("loop: Calibration Mode Servo Pos: "); Serial.println(newServoPos); 
#endif      
      setServoPos( newServoPos );
    }
  }

  else
  {
      // Do we need to move the servo?
    if((newServoPos != lastServoPos) && (sinceServoMoved >= MILLIS_PER_SERVO_STEP))
    {
      int8_t moveDirection = (newServoPos > lastServoPos) ? 1 : -1;
      lastServoPos += moveDirection;
  
#ifdef ENABLE_SERIAL_DEBUG
      Serial.print("loop: Move Servo: Last Pos: "); Serial.println(lastServoPos);
#endif
      
      setServoPos( lastServoPos );
    }
  }

    // Update the LED state or Flash at different rates if servo Moving or Turnout Position Unknown
  if((newServoPos == lastServoPos) && (newServoPos >= 0))
  {
    digitalWrite(LED_OUTPUT, lastTurnoutPos == NORMAL_TURNOUT_POS);
  }

  else
  {
    uint16_t flashInterval = (lastTurnoutPos != UNKNOWN_TURNOUT_POS) ? MILLIS_PER_LED_FLASH_WHILE_SERVO_MOVES : MILLIS_PER_LED_FLASH_UNKNOWN_POS;
    if( sinceLEDChanged >= flashInterval )
    {
      digitalWrite( LED_OUTPUT, !digitalRead( LED_OUTPUT ));
      sinceLEDChanged = 0;
    }
  }

#ifdef SERVO_DETACH_MILLIS
    // Detach the Servo drive signal after the defined millis since the last movement 
  if(myservo.attached() && sinceServoMoved > SERVO_DETACH_MILLIS)
     myservo.detach();
#endif     
}

void handleLongDoublePress()
{
  enableCalibrateMode = !enableCalibrateMode;

#ifdef ENABLE_SERIAL_DEBUG
  Serial.print(F("handleLongDoublePress: Calibration Mode: ")); Serial.println(enableCalibrateMode ? "Enabled" : "Disabled" );
#endif

  if(enableCalibrateMode)
    sinceCalibrationEnabled = 0;
}

void handleButtonClick()
{
    // Toggle the turnout from the last position
  TURNOUT_POS newTurnoutPos = (lastTurnoutPos == NORMAL_TURNOUT_POS) ? REVERSE_TURNOUT_POS : NORMAL_TURNOUT_POS ;

  newServoPos = getServoPositionSetting( newTurnoutPos );

    // If this is the first time we've set the server from Unknown then just move imediately to that position 
  if(lastServoPos == -1)
  {
    setServoPos( newServoPos );
    lastServoPos = newServoPos;
  }

#ifdef ENABLE_SERIAL_DEBUG
  Serial.print("loop: Button Pressed: New Servo Pos: "); Serial.println(newServoPos);
#endif
  
  EEPROM.update(LAST_TURNOUT_POS_EEPROM_INDEX, newTurnoutPos);
  lastTurnoutPos = newTurnoutPos;
}
