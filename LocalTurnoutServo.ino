// Arduino Sketch to provide Local Facia control of a turnout position driven by a srvocontrol

#include <EEPROM.h>
#include <Bounce2.h>
#include <elapsedMillis.h>

// Uncomment the next line to enable Debug Print
#define ENABLE_DEBUG

#if defined(ARDUINO_AVR_UNO)
#include <Servo.h>

#define NORMAL_POS_AN_INP   A0
#define REVERSE_POS_AN_INP  A1

#define TOGGLE_BUTTON       4
#define SERVO_OUTPUT        5
#define LED_OUTPUT          6

#elif defined(ARDUINO_AVR_ATTINYX5) 
#include <Servo_ATTinyCore.h>

#define NORMAL_POS_AN_INP   A2
#define REVERSE_POS_AN_INP  A3

#define TOGGLE_BUTTON       2
#define SERVO_OUTPUT        1
#define LED_OUTPUT          0

#else
#error "Unspecified or Unsupported Board"
#endif

#define NORMAL_POS  false
#define REVERSE_POS true
#define LAST_TURNOUT_POS_EEPROM_INDEX 0

// Uncomment the next line to enable Servo Detach after X ms since the last movement
#define SERVO_DETACH_MILLIS 1000

enum TURNOUT_POS{
  NORMAL_TURNOUT_POS = 0,
  REVERSE_TURNOUT_POS = 1,
  UNKNOWN_TURNOUT_POS = 0xFF
};

TURNOUT_POS lastTurnoutPos;

Servo myservo;

Bounce2::Button togglePosButton = Bounce2::Button();

#ifdef SERVO_DETACH_MILLIS
elapsedMillis sinceServoMoved;
#endif

void setServoPos(TURNOUT_POS newPos)
{
  uint16_t potVal = (newPos == NORMAL_TURNOUT_POS) ? analogRead(NORMAL_POS_AN_INP) : analogRead(REVERSE_POS_AN_INP);
  uint16_t newServoPos = map(potVal, 0, 1023, 0, 180);
  myservo.attach(SERVO_OUTPUT);
  myservo.write(newServoPos);
  digitalWrite(LED_OUTPUT, newPos == NORMAL_TURNOUT_POS);

#ifdef SERVO_DETACH_MILLIS
  sinceServoMoved = 0;
#endif
}

void setup() {
#ifdef ENABLE_DEBUG
  Serial.begin(115200);
  Serial.println("\nLocal Turnout Server Driver");
#endif

  togglePosButton.attach( TOGGLE_BUTTON, INPUT_PULLUP );
  togglePosButton.interval(5);
  togglePosButton.setPressedState(LOW); 

  pinMode(LED_OUTPUT, OUTPUT);

  lastTurnoutPos = (TURNOUT_POS) (EEPROM.read(LAST_TURNOUT_POS_EEPROM_INDEX));

  if(lastTurnoutPos != UNKNOWN_TURNOUT_POS)
    setServoPos(lastTurnoutPos);

}

void loop() {
  togglePosButton.update();
  
  if ( togglePosButton.pressed() ) {

      // Toggle the turnout from the last position
    TURNOUT_POS newTurnoutPos = (lastTurnoutPos == NORMAL_TURNOUT_POS) ? REVERSE_TURNOUT_POS : NORMAL_TURNOUT_POS ;
    setServoPos(newTurnoutPos);
    EEPROM.update(LAST_TURNOUT_POS_EEPROM_INDEX, newTurnoutPos);
    lastTurnoutPos = newTurnoutPos;
  }

#ifdef SERVO_DETACH_MILLIS
    // Detach the Servo Pin 1000ms after the last movement 
  if(myservo.attached() && sinceServoMoved > 1000)
     myservo.detach();
#endif     
}
