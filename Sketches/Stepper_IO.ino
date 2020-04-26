// Dual Stepper Controller with OLED/rotary encoder menu
// Uses PCF8574 GPIO expanders to control A4988 stepper driver modules
// Compiled for Arduino Pro Micro 5v version in Arduino IDE 1.8.10
//
// Libraries used: PCF8574 https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
//                 Adafruit SSD1306 https://github.com/adafruit/Adafruit_SSD1306
//                 Adafruit GFX https://github.com/adafruit/Adafruit-GFX-Library
//                 Encoder https://github.com/0xPIT/encoder/tree/arduino
//                 Timer  https://github.com/PaulStoffregen/TimerOne
//
// Rotary Encoder and menu based on example at
// https://educ8s.tv/arduino-rotary-encoder-menu/
//
// Gadget Reboot
//

#include <Wire.h>
#include <PCF8574.h>
#define addr1  0x20    // pcf8574 device 1 I2C address
#define addr2  0x21    // pcf8574 device 2 I2C address

// create pcf8574 controller objects
PCF8574 pcf8574_1(addr1);
PCF8574 pcf8574_2(addr2);

// oled display driver
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
#define OLED_addr  0x3C

// create OLED display object
Adafruit_SSD1306 display(OLED_RESET);

// rotary encoder related libraries
#include <ClickEncoder.h>
#include <TimerOne.h>

// rotary encoder object and variables
ClickEncoder *encoder;
int16_t last, value;

// rotary encoder inputs
#define encA     1     // rotary encoder input A 
#define encB     0     // rotary encoder input B
#define encSW   21     // rotary encoder push switch

// pcf8574 GPIO assignment for outputs to control motor drivers
#define nRST     0     // nReset
#define nSLP     1     // nSleep
#define nENA     2     // nEnable
#define STEP     3     // Step
#define DIR      4     // Direction
#define MS1      5     // Microstep mode pin 1
#define MS2      6     // Microstep mode pin 2
#define MS3      7     // Microstep mode pin 3

// inputs/outputs for optional switches/sensors/external signals
#define in1      7     // input 1
#define in2      8     // input 2 
#define in3      9     // input 3
#define in4      10    // input 4
#define io1      16    // input/output spare 1
#define io2      14    // input/output spare 2

// direct motor step control outputs if not using pcf8574 for step incrementing
#define step1    18    // motor 1 alternative step control
#define step2    15    // motor 2 alternative step control

// misc gpio
#define redLED   4     // general purpose leds
#define yelLED   5
#define grnLED   6
#define spare1   20    // spare io 1
#define spare2   19    // spare io 2 

// misc assignments for convenience within functions
#define baseMotor     1         // motor1 drives the rotating platform
#define bladeMotor    2         // motor2 drives the cutting blade
#define cwDir         true      // represents clockwise rotation
#define ccwDir        false     // represents counter-clockwise rotation
#define baseMotor_speed    200  // base  motor delay between steps for controlling speed 
#define bladeMotor_speed   700  // blade motor delay between steps for controlling speed  
#define baseMotor_steps    3200 // number of steps to rotate platform while cutting 
int     bladeMotor_steps = 142; // how many steps needed to advance blade from home sensor to cutting base

// oled menu system variables
int menuitem = 1;
int frame = 1;
int page = 1;
int lastMenuItem = 1;
boolean up = false;
boolean down = false;
boolean encButtonPress = false;

// oled menu selections
String menuItem1 = "Normal Run";   // run the complete cutting operation
String menuItem2 = "Rotate Base";  // test: just rotate the cutting platform
String menuItem3 = "Blade Home";   // test: Send the blade to the home position
String menuItem4 = "Blade Add 1";  // test: increment blade-to-cutting-platform motor step count
String menuItem5 = "Blade Sub 1";  // test: decrement blade-to-cutting-platform motor step count
String menuItem6 = "Reserved";     // unused future option

void setup() {

  Serial.begin(9600); // debug message output option
  Wire.begin();       // prepare for I2C communication

  // initialize PCF8574's and assign default motor control signals to drivers
  // bits [7..0] = [MS3, MS2, MS1, DIR, STEP, nENA, nSLP, nRST]
  // eg.    0x07 = 0000 0111
  // microstepping controls:
  // MS1  MS2  MS3  Step Mode
  //  0    0    0    Full
  //  1    0    0    Half
  //  0    1    0    1/4
  //  1    1    0    1/8
  //  1    1    1    1/16
  pcf8574_1.begin(0x67); // motor controls: 1/8 step, not asleep, not in reset, not enabled
  pcf8574_2.begin(0x67); // motor controls: 1/8 step, not asleep, not in reset, not enabled

  // configure misc IO as inputs
  pinMode(in1, INPUT_PULLUP);
  pinMode(in2, INPUT_PULLUP);
  pinMode(in3, INPUT_PULLUP);
  pinMode(in4, INPUT_PULLUP);
  pinMode(io1, INPUT_PULLUP);
  pinMode(io2, INPUT_PULLUP);
  pinMode(spare1, INPUT_PULLUP);
  pinMode(spare2, INPUT_PULLUP);

  // configure optional step direct outputs
  digitalWrite(step1, LOW);
  digitalWrite(step2, LOW);
  pinMode(step1, OUTPUT);
  pinMode(step2, OUTPUT);

  // configure LEDs (using all 3 as home sensor confirmation)
  digitalWrite(redLED, LOW);
  digitalWrite(yelLED, LOW);
  digitalWrite(grnLED, LOW);
  pinMode(redLED, OUTPUT);
  pinMode(yelLED, OUTPUT);
  pinMode(grnLED, OUTPUT);

  // configure rotary encoder
  encoder = new ClickEncoder(encB, encA, encSW);
  encoder->setAccelerationEnabled(false);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = encoder->getValue();

  // initialize OLED with the I2C address of the display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_addr);
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.display();

  Serial.println("Stepper Driver and I/O PCB: Debug Output Port\n");
}


void loop() {

  // read blade home sensor and update LED status
  checkHomeSensor();

  // handle oled display and rotary encoder
  drawMenu();
  readRotaryEncoder();

  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Clicked:
        encButtonPress = true;
        break;
    }
  }
  processMenuNavigation();
  processEncoderButton();
}


// Rotary encoder interrupt
void timerIsr() {
  encoder->service();
}

// -------------------------
// Main Operational Routine
// -------------------------

void  runMotorsNormalOperation() {

  Serial.println("Begin cutting operation...");

  // prepare motors for motion
  enableMotor(bladeMotor);
  enableMotor(baseMotor);

  // move blade motor slightly away from home position if it is currently home because
  // the sensor detects a wide blocking region and it's best to detect entry into the sensor zone.
  // if the motor had been manually moved and is blocking the sensor, it may not be
  // in the expected home position
  Serial.println("Blade:  Move to home position...");
  if (checkHomeSensor()) {
    do {
      moveMotor(bladeMotor, cwDir, 1);   // move blade motor away from sensor one step at a time
    } while (checkHomeSensor());
  }
  moveMotor(bladeMotor, cwDir, 40);      // when sensor is cleared, move a little more for clearance
  delay(300);

  // move blade motor back to home position (now calibrated to detect entry into sensor zone)
  if (!checkHomeSensor()) {
    do {
      moveMotor(bladeMotor, ccwDir, 1);  // move blade motor toward sensor one step at a time
    } while (!checkHomeSensor());
  }

  // move blade to cutting surface and keep enabled to hold blade in position
  Serial.println("Blade:  Move to cutting position...");
  moveMotor(bladeMotor, cwDir, bladeMotor_steps);

  // rotate platform to make cut
  Serial.println("Base:  Rotate to make a cut...");
  moveMotor(baseMotor, cwDir, baseMotor_steps);
  delay(500);                            // hold position to prevent momentum overshoot
  disableMotor(baseMotor);

  // raise blade motor from base
  Serial.println("Blade:  Move to home position...");
  moveMotor(bladeMotor, ccwDir, bladeMotor_steps);
  delay(500);                            // hold position to avoid momentum overshoot
  disableMotor(bladeMotor);
  Serial.println("End cutting operation...\n");
}


// ------------------------
// Motor Control  Routines
// ------------------------

// enable requested motor
void enableMotor(byte motor) {

  if (motor == baseMotor) {
    pcf8574_1.write(nENA, LOW);  // enable motor 1
  }
  else if (motor == bladeMotor) {
    pcf8574_2.write(nENA, LOW);  // enable motor 2
  }
}

// disable requested motor
void disableMotor(byte motor) {

  if (motor == baseMotor) {
    pcf8574_1.write(nENA, HIGH);  // disable motor 1
  }
  else if (motor == bladeMotor) {
    pcf8574_2.write(nENA, HIGH);  // disable motor 2
  }
}

// move requested motor in a direction for a number of steps
void moveMotor(byte motor, bool dir, int steps) {

  if (motor == baseMotor) {
    pcf8574_1.write(DIR,  dir);  // set motor direction
    pcf8574_1.write(nENA, LOW);  // enable motor
    // move motor number of requested steps
    for (int i = 0; i < steps; i++) {
      pcf8574_1.write(STEP, HIGH);
      delayMicroseconds(baseMotor_speed);
      pcf8574_1.write(STEP, LOW);
      delayMicroseconds(baseMotor_speed);
    }
  }
  else if (motor == bladeMotor) {
    pcf8574_2.write(DIR,  dir);  // set motor direction
    pcf8574_2.write(nENA, LOW);  // enable motor
    // move motor number of requested steps
    for (int i = 0; i < steps; i++) {
      pcf8574_2.write(STEP, HIGH);
      delayMicroseconds(bladeMotor_speed);
      pcf8574_2.write(STEP, LOW);
      delayMicroseconds(bladeMotor_speed);
    }
  }
}

// check if blade sensor is blocked (blade is home) and control status LEDs
bool checkHomeSensor() {
  bool sensorState = digitalRead(in3);
  if (sensorState == HIGH) {
    digitalWrite(redLED, HIGH);
    digitalWrite(yelLED, HIGH);
    digitalWrite(grnLED, HIGH);
  }
  else {
    digitalWrite(redLED, LOW);
    digitalWrite(yelLED, LOW);
    digitalWrite(grnLED, LOW);
  }
  return (sensorState);
}

// --------------------------------
// Diagnostic/Calibration Routines
// --------------------------------

// test mode - just rotate platform
void rotateBaseMotor() {
  Serial.println("Rotating base...");
  enableMotor(baseMotor);
  moveMotor(baseMotor, cwDir, baseMotor_steps);
  delay(500);                            // hold position to prevent momentum overshoot
  disableMotor(baseMotor);
  Serial.println("Rotation complete...\n");
}

// test mode - move blade motor to home position if not home
void sendBladeMotorHome() {
  Serial.println("Send blade to home position...");
  if (!checkHomeSensor()) {
    enableMotor(bladeMotor);
    do {
      moveMotor(bladeMotor, ccwDir, 1);  // move blade motor toward sensor one step at a time
    } while (!checkHomeSensor());
    delay(500);                          // hold position to prevent momentum overshoot
    disableMotor(bladeMotor);
  }
  Serial.println("Operation complete...\n");
}// calibration mode: add 1 step to blade travel between home and cutting surface
void bladeAdd1() {
  bladeMotor_steps++;
}

// calibration mode: subtract 1 step from blade travel between home and cutting surface
void bladeSub1() {
  bladeMotor_steps--;
}




// --------------------------------------
// Rotary Encoder Menu Interface Routines
// --------------------------------------

void displayIntMenuPage(String menuItem, int value)
{
  display.setTextSize(0);
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(15, 0);
  display.print(menuItem);
  display.drawFastHLine(0, 7, 83, WHITE);
  display.setCursor(5, 8);
  display.print("Value");
  display.setTextSize(0);
  display.setCursor(5, 16);
  display.print(value);
  display.setTextSize(0);
  display.display();
}

void displayStringMenuPage(String menuItem, String value)
{
  display.setTextSize(0);
  display.clearDisplay();
  display.setTextColor(WHITE, BLACK);
  display.setCursor(15, 0);
  display.print(menuItem);
  display.drawFastHLine(0, 7, 83, WHITE);
  display.setCursor(5, 8);
  display.print("Value");
  display.setTextSize(0);
  display.setCursor(5, 16);
  display.print(value);
  display.setTextSize(0);
  display.display();
}

void displayMenuItem(String item, int position, boolean selected)
{
  if (selected)
  {
    display.setTextColor(BLACK, WHITE);
  } else
  {
    display.setTextColor(WHITE, BLACK);
  }
  display.setCursor(0, position);
  display.print(">" + item);
}

void readRotaryEncoder()
{
  value += encoder->getValue();

  if (value / 2 > last) {
    last = value / 2;
    down = true;
    delay(150);
  } else   if (value / 2 < last) {
    last = value / 2;
    up = true;
    delay(150);
  }
}

void drawMenu()
{

  if (page == 1)
  {
    display.setTextSize(0);
    display.clearDisplay();
    display.setTextColor(BLACK, WHITE);
    display.setCursor(15, 0);
    display.print("MAIN MENU  ");
    display.print(bladeMotor_steps);
    display.drawFastHLine(0, 10, 83, BLACK);

    if (menuitem == 1 && frame == 1)
    {
      displayMenuItem(menuItem1, 8, true);
      displayMenuItem(menuItem2, 16, false);
      displayMenuItem(menuItem3, 24, false);
    }
    else if (menuitem == 2 && frame == 1)
    {
      displayMenuItem(menuItem1, 8, false);
      displayMenuItem(menuItem2, 16, true);
      displayMenuItem(menuItem3, 24, false);
    }
    else if (menuitem == 3 && frame == 1)
    {
      displayMenuItem(menuItem1, 8, false);
      displayMenuItem(menuItem2, 16, false);
      displayMenuItem(menuItem3, 24, true);
    }
    else if (menuitem == 4 && frame == 2)
    {
      displayMenuItem(menuItem2, 8, false);
      displayMenuItem(menuItem3, 16, false);
      displayMenuItem(menuItem4, 24, true);
    }

    else if (menuitem == 3 && frame == 2)
    {
      displayMenuItem(menuItem2, 8, false);
      displayMenuItem(menuItem3, 16, true);
      displayMenuItem(menuItem4, 24, false);
    }
    else if (menuitem == 2 && frame == 2)
    {
      displayMenuItem(menuItem2, 8, true);
      displayMenuItem(menuItem3, 16, false);
      displayMenuItem(menuItem4, 24, false);
    }

    else if (menuitem == 5 && frame == 3)
    {
      displayMenuItem(menuItem3, 8, false);
      displayMenuItem(menuItem4, 16, false);
      displayMenuItem(menuItem5, 24, true);
    }

    else if (menuitem == 6 && frame == 4)
    {
      displayMenuItem(menuItem4, 8, false);
      displayMenuItem(menuItem5, 16, false);
      displayMenuItem(menuItem6, 24, true);
    }

    else if (menuitem == 5 && frame == 4)
    {
      displayMenuItem(menuItem4, 8, false);
      displayMenuItem(menuItem5, 16, true);
      displayMenuItem(menuItem6, 24, false);
    }
    else if (menuitem == 4 && frame == 4)
    {
      displayMenuItem(menuItem4, 8, true);
      displayMenuItem(menuItem5, 16, false);
      displayMenuItem(menuItem6, 24, false);
    }
    else if (menuitem == 3 && frame == 3)
    {
      displayMenuItem(menuItem3, 8, true);
      displayMenuItem(menuItem4, 16, false);
      displayMenuItem(menuItem5, 24, false);
    }
    else if (menuitem == 2 && frame == 2)
    {
      displayMenuItem(menuItem2, 8, true);
      displayMenuItem(menuItem3, 16, false);
      displayMenuItem(menuItem4, 24, false);
    }
    else if (menuitem == 4 && frame == 3)
    {
      displayMenuItem(menuItem3, 8, false);
      displayMenuItem(menuItem4, 16, true);
      displayMenuItem(menuItem5, 24, false);
    }
    display.display();
  }
  else if (page == 2 && menuitem == 1)
  {
    displayStringMenuPage(menuItem1, "test");
  }

  else if (page == 2 && menuitem == 2)
  {
    displayIntMenuPage(menuItem2, 10);
  }
  else if (page == 2 && menuitem == 3)
  {
    displayStringMenuPage(menuItem3, "test");
  }
  else if (page == 2 && menuitem == 4)
  {
    displayStringMenuPage(menuItem4, "test");
  }
  else if (page == 2 && menuitem == 5)
  {
    displayStringMenuPage(menuItem5, "test");
  }
}

// execute display menu functions if rotary encoder button has been pressed
void processEncoderButton()
{
  if (encButtonPress) // encoder button is Pressed
  {
    encButtonPress = false;

    if (page == 1 && menuitem == 1) // Run a normal motor control operation cycle
    {
      runMotorsNormalOperation();
      runMotorsNormalOperation();
      runMotorsNormalOperation();
    }

    if (page == 1 && menuitem == 2) // T85 Reset Disabled
    {
      rotateBaseMotor();  // run motor test
    }

    if (page == 1 && menuitem == 3) // T85 Reset Enabled
    {
      sendBladeMotorHome();  // run motor test
    }

    if (page == 1 && menuitem == 4) // Reserved
    {
      bladeAdd1();
    }

    if (page == 1 && menuitem == 5) // Reserved
    {
      bladeSub1();
    }

    if (page == 1 && menuitem == 6) // Reserved
    {
      // reserved feature
    }

    /*  // sub-menu levels
        else if (page == 1 && menuitem <= 4) {
          page = 2;
        }
    */
    else if (page == 2)  // esc back to top level menu
    {
      page = 1;
    }
  }
}

void processMenuNavigation()
{
  if (up && page == 1 ) {

    up = false;
    if (menuitem == 2 && frame == 2)
    {
      frame--;
    }

    if (menuitem == 4 && frame == 4)
    {
      frame--;
    }
    if (menuitem == 3 && frame == 3)
    {
      frame--;
    }
    lastMenuItem = menuitem;
    menuitem--;
    if (menuitem == 0)
    {
      menuitem = 1;
    }
  }
  else if (up && page == 2 && menuitem == 1 ) {
    up = false;
    //  readFuses();

  }
  else if (up && page == 2 && menuitem == 2 ) {
    up = false;
    //  volume--;
  }
  else if (up && page == 2 && menuitem == 3 ) {
    up = false;
    //  selectedLanguage--;
    //  if (selectedLanguage == -1)
    //   {
    //     selectedLanguage = 2;
    //  }
  }
  else if (up && page == 2 && menuitem == 4 ) {
    up = false;
    // selectedDifficulty--;
    // if (selectedDifficulty == -1)
    //  {
    //    selectedDifficulty = 1;
    //  }
  }

  if (down && page == 1) //We have turned the Rotary Encoder Clockwise
  {

    down = false;
    if (menuitem == 3 && lastMenuItem == 2)
    {
      frame ++;
    } else  if (menuitem == 4 && lastMenuItem == 3)
    {
      frame ++;
    }
    else  if (menuitem == 5 && lastMenuItem == 4 && frame != 4)
    {
      frame ++;
    }
    lastMenuItem = menuitem;
    menuitem++;
    if (menuitem == 7)
    {
      menuitem--;
    }

  } else if (down && page == 2 && menuitem == 1) {
    down = false;
    // readFuses();
  }
  else if (down && page == 2 && menuitem == 2) {
    down = false;
  }
  else if (down && page == 2 && menuitem == 3 ) {
    down = false;
  }
  else if (down && page == 2 && menuitem == 4 ) {
    down = false;
  }
}
