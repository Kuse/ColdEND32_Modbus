# ColdEND32-Modbus Fork

# Modbus support
In this fork Modbus RTU server support was added. For concurrent manual and Modbus control, rotary encoders instead of potis could be sed.

**The Software is Beta stage and not fully tested yet. Usage of this software is your own risk including damage of the ColdEND32 hardware**

**Modbus Server ID**  10

**holding registers**

Name           | Address  | Type     | Range                          | Access
---------------|----------|----------|--------------------------------|-------
HOLDING_SPIT   |  1       | UINT16   | MIN_SPIT_TIME .. MAX_SPIT_TIME | RW
HOLDING_MIST   |  2       | UINT16   | MIN_RPM * 10 .. MAX_RPM * 10   | RW

**coils**

Name                | Address  | Access
--------------------|----------|-------
COIL_MIST_STAT      | 1        |  RW
COIL_AIR_STAT       | 2        |  RW
COIL_FAST_MODE      | 3        |  RW
COIL_COOLANT_VALVE  | 4        |  R
COIL_AIR_VALVE      | 5        |  R


# ColdEND32
Firmware for ColdEND32 controller supporting various displays
Installation and configuration:

1) Download and import the following libraries depending on your display:

   **SSD1306 or SH1106**
   https://github.com/olikraus/U8g2_Arduino

   **HT16K33**
   https://github.com/adafruit/Adafruit_LED_Backpack

   **LCD 16x2 or 16x4**
   https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

2) Uncomment (define) the appropriate display in config.h. Note that only one type of
   display at one time is allowed so don't forget to comment the other ones.
   For 7 segment LED displays, don't forget to jumper different I2C addresses for both
   displays and enter these values in config.h ("LED1_ADD" and "LED2_ADD").

3) Uncomment (define) MOMENTARY_SWITCH if you want to use monentary switches.
   Required library: https://github.com/thomasfredericks/Bounce2
   For solid state switches leave MOMENTARY_SWITCH commented.

4) Uncomment (define) DRAIN_SYSTEM if you want to have the option to drain the coolant
   pump and pipes anytime. To drain your system, turn the coolant pot all the way down
   and push the fast mode switch.

5) Uncomment (define) REMOTE_CTRL if you want to control your ColdEND32 system from the
   CNC controller with a 24V solid signal (must be connected to input #4). Please note
   that momentary switches are required!

6) Uncomment (define EXT_LED for external LEDs indicating active mist, air or spit mode.
   LEDs will be connected to first 3 pin pairs of JP5 (inner pin: +5V, outer Pin: GND).
   Please use appropriate resistors!

7) Flash your ColdEND controller and do a first check:

   Problem                                           | Solution
   --------------------------------------------------|-----------
   Pump speed range does not match requirements      | Edit MIN_RPM and/or MAX_RPM
   Flow direction of pump is wrong                   | Change FLOW_DIR
   Fast mode speed is too slow/fast                  | Edit FAST_RPM
   Spit mode speed is too slow/fast                  | Edit SPIT_RPM
   Spit time range dows not match requirements       | Edit MIN_SPIT_TIME and/or MAX_SPIT_TIME
   Spit mode does not start at times                 | Slightly increase SWITCH_DEBOUNCE
   Momentary switches do not work properly           | Slightly increase SWITCH_DEBOUNCE
   16x4 LCD rows 3 and 4 starting at wrong position  | Uncomment FIX16X4
   Display seems to be too slow                      | Decrease REFRESH_TIME
   Mist or Spit pot values are jumping               | Try lower POT_FILTER values
   Mist or Spit pot values are still jumping         | Increase POT_LOOP to 2 or 3

8) If any of these values have been altered, you need to flash the controller once again.


**HISTORY:**

- v1.2: Fix some 16x4 LCD row 3 and 4 starting at wrong positions

- v1.1: Added support for external LEDs indicating the active mode (mist, air or spit)

- v1.0: Initial firmware for ESP32 controlled pump board
