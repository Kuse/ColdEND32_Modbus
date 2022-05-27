/*

  ColdEND32 v1.2 Minimum Quantity Lubrication
  https://www.end-cnc-shop.de

  Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License

  Written by Tilman, 2021-12-11
  Last edited by Tilman, 2021-12-29
  Last edited by Kuse, 2022-05-27

*/


// Create hardware timer
hw_timer_t * stepTimer = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

#include "config.h"
#include "vars.h"
#include "func.h"
#include "modbus.h"
#include "rotary_encoder.h"

#if defined OLED || defined LED || defined LCD
  #include "display.h"
#endif


void setup() {
  // Set input pins
  pinMode(IN_MIST, INPUT);
  pinMode(IN_FAST, INPUT);
  pinMode(IN_AIR, INPUT);
  pinMode(IN_REM, INPUT);

  // Set output pins
  pinMode(OUT_COOLANT, OUTPUT);
  pinMode(OUT_AIR, OUTPUT);
  pinMode(OUT_LIFT, OUTPUT);
  pinMode(OUT_SPIT_LED, OUTPUT);
  pinMode(OUT_STEP, OUTPUT);
  pinMode(OUT_DIR, OUTPUT);
  pinMode(OUT_ENABLE, OUTPUT);

  #ifdef EXT_LED
    pinMode(OUT_5V_1, OUTPUT);
    pinMode(OUT_5V_2, OUTPUT);
    pinMode(OUT_5V_3, OUTPUT);
    pinMode(OUT_5V_4, OUTPUT);
  #endif

  // Init Serial monitor
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize stepper
  digitalWrite(OUT_DIR, FLOW_DIR);                      // Set flow direction
  digitalWrite(OUT_ENABLE, HIGH);                       // Disable stepper

  // Timer setup
  stepTimer = timerBegin(0, 80, true);                  // Use first timer at 80MHz/80 = 1µs, count up
  timerAttachInterrupt(stepTimer, &stepPulse, true);    // Attach stepPulse function to timer, edge (not level) triggered

  // Initialize display
  #ifdef OLED
    u8g2.begin();
  #endif

  // Initialize Modbus
  #if defined MODBUS
    modbusSetup(); 
  #endif

  // Initialialize Rotary encoder
  #if defined ROTARY
    rotarySetup();
  #endif

  #ifdef LED
    led1.begin(LED1_ADD);
    led1.setBrightness(BRIGHTNESS);
    led2.begin(LED2_ADD);
    led2.setBrightness(BRIGHTNESS);
  #endif

  #ifdef LCD
    lcd.begin();
    lcd.backlight();
  #endif


  Serial.println("__ Setup OK");
}


void loop() {
  // Read switch states
  switchStat();

  // Set air and coolant valves depending on switches
  setValves();

  #ifdef ROTARY
    // read rotary values
    rotaryVals();
  #else
    // Read potentiometer values
    potVals();  
  #endif

  //  Modbus
  #ifdef MODBUS
    doModbus(); 
  #endif  

  // Switch LEDs
  switchLEDs();

  // Control coolant pump
  pumpControl();
  
  // Refresh display (if defined)
  #if defined OLED || defined LED || defined LCD
    refDisplay();
  #endif


}





//
