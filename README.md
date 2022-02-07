# LocalTurnoutServo

Arduino Sketch to provide local facia push button control of a turnout position driven by a servo

The sketch has different definitions for an Arduino UNO for easier development and a ATTiny85 for production.

For the ATTiny85 use the ATTinyCore Arduino Core from here: https:github.com/SpenceKonde/ATTinyCore

The Required I/O are defined below as follows:

  NORMAL_POS_AN_INP  : Analog input from TrimPOT to set the Normal  servo position
  REVERSE_POS_AN_INP : Analog input from TrimPOT to set the Reverse servo position

  TOGGLE_BUTTON      : Digital input with internal pull-up enabled for a momentary push button to GND to toggle turnout position

  SERVO_OUTPUT       : Digital output to drive the servo position

  LED_OUTPUT         : Digital output to drive two LEDs, one connected to VCC via a resistor and the other connectd to GND via a resistor
                       so that one is on when the pin is HIGH and the other is on when the pin is LOW


