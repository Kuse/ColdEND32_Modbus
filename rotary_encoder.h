/*

  ColdEND32 v1.2 Minimum Quantity Lubrication
  https://www.end-cnc-shop.de

  Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License

  rotary-encoder.h modbus.h Written by Kuse, 2022-05-27
  Last edited by Kuse, 2022-05-27

*/

#ifdef ROTARY

// https://github.com/igorantolic/ai-esp32-rotary-encoder
#include "AiEsp32RotaryEncoder.h"
//https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
#include "Preferences.h"

#define MIST_VAL_KEY "MistVal"
#define SPIT_VAL_KEY "SpitVal"

#define ROT_ENC_STEPS 4
#define STORAGE_AFTER_LAST_CHANGE   (30*1000) //time in ms

// store and restore values from preferences
Preferences preferences;

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoderSpit = AiEsp32RotaryEncoder(IN_ROT_ENC1_A, IN_ROT_ENC1_B, -1, -1, ROT_ENC_STEPS); 
AiEsp32RotaryEncoder rotaryEncoderMist = AiEsp32RotaryEncoder(IN_ROT_ENC2_A, IN_ROT_ENC2_B, -1, -1, ROT_ENC_STEPS);


void IRAM_ATTR readEncoderMistISR()
{
  rotaryEncoderMist.readEncoder_ISR();
}

void IRAM_ATTR readEncoderSpitISR()
{
  rotaryEncoderSpit.readEncoder_ISR();
}


void rotarySetup()
{
  preferences.begin("ColdEnd", false); 

  // get mist value, if not in preference than default
  mist_pot_val = preferences.getFloat(MIST_VAL_KEY, MIN_RPM);   
  mist_val = mist_pot_val;

  // get spit value, if not in preference than default
  spit_pot_val = preferences.getFloat(SPIT_VAL_KEY, 1); 
  spit_val = spit_pot_val;
    
  //we must initialize rotary encoder
  rotaryEncoderMist.begin();
  rotaryEncoderMist.setup(readEncoderMistISR);

  rotaryEncoderSpit.begin();
  rotaryEncoderSpit.setup(readEncoderSpitISR);
  
  //set boundaries and if values should cycle or not
  rotaryEncoderMist.setBoundaries(0, MAX_RPM+90, false);     //minValue, maxValue, circleValues 
  rotaryEncoderSpit.setBoundaries(0, MAX_SPIT_TIME, false); 

  // rotary Acceleration
  rotaryEncoderMist.setAcceleration(100);  // larger number = more accelearation
  rotaryEncoderSpit.setAcceleration(0);    // disable 
}



void rotaryVals()
{
  static bool valueChanged = false;
  static unsigned long lastChangeTime = 0;
   
 
  if (rotaryEncoderMist.encoderChanged())
  {
    mist_pot_val = rotaryEncoderMist.readEncoder();
    
    if (mist_pot_val < 100)
      mist_pot_val = mist_pot_val / 10.0;
    else
      mist_pot_val = mist_pot_val - 90.0;  

    mist_val = mist_pot_val;

    lastChangeTime = millis();
    valueChanged = true;
     
    Serial.println(mist_pot_val);
  }


  if (rotaryEncoderSpit.encoderChanged())
  {
    spit_pot_val = rotaryEncoderSpit.readEncoder();
    spit_val = spit_pot_val;

    lastChangeTime = millis();
    valueChanged = true;
  }

  if ( valueChanged && ((millis() - lastChangeTime) > STORAGE_AFTER_LAST_CHANGE) )      
  {
     preferences.putFloat(MIST_VAL_KEY,mist_pot_val);
     preferences.putFloat(SPIT_VAL_KEY,spit_pot_val);
     valueChanged = false;

     Serial.println("Preferences stored in Flash");
  }
}

#endif
