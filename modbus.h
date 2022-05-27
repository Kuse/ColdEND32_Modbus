/*

  ColdEND32 v1.2 Minimum Quantity Lubrication
  https://www.end-cnc-shop.de

  Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License

  modbus.h Written by Kuse, 2022-05-27
  Last edited by Kuse, 2022-05-15

*/

#ifdef MODBUS

// #define LOG_LEVEL  LOG_LEVEL_WARNING
#include "Logging.h"

// https://emodbus.github.io/modbusclient
#include "ModbusServerRTU.h"


////////////////  Modbus Registers  /////////////////////////////////////////////////////////// 
#define MB_HOLDING_SPIT  (1 - 1)   // UINT16  0..MAX_                    RW
#define MB_HOLDING_MIST  (2 - 1)   // UINT16  MIN_RPM*10, MAX_RPM*10     RW
#define MAX_HOLDINGS     (MB_HOLDING_MIST + 1)

//coils 
#define MB_COIL_MIST_STAT     (1 - 1)   // RW
#define MB_COIL_AIR_STAT      (2 - 1)   // RW
#define MB_COIL_FAST_MODE     (3 - 1)   // RW
#define MB_COIL_COOLANT_VALVE (4 - 1)   // R
#define MB_COIL_AIR_VALVE     (5 - 1)   // R
#define MAX_COILS             (MB_COIL_AIR_VALVE + 1)   
///////////////////////////////////////////////////////////////////////////////////////////////

#define SERVER_ID           6                         // ServerID id number
#define SERIAL_TIMEOUT      100000                      // Serial timeout in ms
#define SERIAL_INTERVAL     1000                        // Serial interval in ms
#define MB_TIMEOUT          (30*1000)                   // in ms  

#define CORE_ID             -1

ModbusServerRTU MBserver(Serial2, SERIAL_TIMEOUT,-1);   // RTS pin not used


uint16_t readCoil (uint16_t addr)
{
  uint16_t val = 0;
  
  switch (addr)
  {
    case MB_COIL_MIST_STAT: 
      val = mist_stat;
      break;
    
    case MB_COIL_AIR_STAT: 
      val = air_stat;
      break;
    
    case MB_COIL_FAST_MODE:
      val = fast_mode;
      break;

    case MB_COIL_COOLANT_VALVE: 
      val = coolant_valve;
      break;

    case MB_COIL_AIR_VALVE: 
      val = air_valve;
      break;
  
    default:
      val = 0;
  }

 return (val?0xff00:0);
}


bool writeCoil (uint16_t addr, bool state)
{
   switch (addr)
  {

   case MB_COIL_MIST_STAT: 
      mist_stat = state;
      air_stat = LOW;
      break;
    
    case MB_COIL_AIR_STAT: 
      air_stat = state;
      mist_stat = LOW;
      break;
    
    case MB_COIL_FAST_MODE:
      mb_fast_mode = state;
      break;

    default:
      return false;
  }

  return true;
}

uint16_t readHolding (uint16_t addr)
{
  
  switch (addr)
  {
    case MB_HOLDING_SPIT: 
      return spit_pot_val;
      break;
    
    case MB_HOLDING_MIST: 
      return mist_pot_val * 10;
      break;

    default:
      return 0;
  }
}


bool writeHolding (uint16_t addr, uint16_t val)
{
  switch (addr)
  {
    case MB_HOLDING_SPIT:
      spit_pot_val = val;
      if (spit_pot_val > MAX_SPIT_TIME)
        spit_pot_val = MAX_SPIT_TIME;  

      spit_val = spit_pot_val;  
      break;
    
    case MB_HOLDING_MIST: 
       mist_pot_val = ((float) val)/10.0;
      if (mist_pot_val < MIN_RPM)
        mist_pot_val = MIN_RPM;
      else if (mist_pot_val > MAX_RPM)
        mist_pot_val = MAX_RPM; 
        
      mist_val =  mist_pot_val;  
      break;

    default:
      return false;
  }

 return true;
}

ModbusMessage read_coil_worker(ModbusMessage request) 
{
  ModbusMessage response;       // response message to be sent back
  uint16_t startAddress;        // requested sddress
  uint16_t numCoils;            // requested number of registers

  // get request values 
  request.get(2, startAddress, numCoils);

  if ((startAddress + numCoils) <= MAX_COILS) 
  {
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(numCoils * 2));  
    
    for (uint16_t addr=startAddress; addr<(startAddress+numCoils); addr++)
    {
      response.add(readCoil(addr));
    }
  } 
  else 
  {
    // Something was wrong with the parameters
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}


ModbusMessage write_coil_worker(ModbusMessage request) 
{
  // Request parameters are coil number and 0x0000 (OFF) or 0xFF00 (ON)
  
  ModbusMessage response;
  uint16_t startAddress = 0;
  uint16_t state = 0;
  
  // get request values 
  request.get(2, startAddress, state);

  // Is the coil number valid?
  if (startAddress < MAX_COILS) 
  {
    if (state == 0x0000 || state == 0xFF00) 
    {
      if (writeCoil (startAddress, state?true:false)) 
      {
        // All fine, coil was set.
        response = ECHO_RESPONSE;
      } 
      else
      {
        // Setting the coil failed
        response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
      }
    } 
    else 
    {
      // Wrong data parameter
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
    }
  } 
  else 
  {
    // Something was wrong with the coil number
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}


ModbusMessage read_holding_worker(ModbusMessage request) 
{
  ModbusMessage response;   // response message to be sent back
  uint16_t startAddress;    // requested register address
  uint16_t num;             // requested number of registers

  // get request values
  request.get(2, startAddress);
  request.get(4, num);

  if ( num && ((startAddress + num) <= MAX_HOLDINGS)) 
  {
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(num * 2));
  
    // Fill response with requested data
    for (uint16_t addr = startAddress; addr < startAddress + num; ++addr) 
    {
      response.add(readHolding(addr));
    }
  } 
  else 
  {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}


ModbusMessage write_holding_worker(ModbusMessage request) 
{
  ModbusMessage response;     // response message to be sent back
  uint16_t startAddress;      // requested register address
  uint16_t val;

  // get request values
  request.get(2, startAddress);
  request.get(4, val);

  // Address and words valid? 
  if (startAddress < MAX_HOLDINGS) 
  {
     writeHolding(startAddress,val);
     response = ECHO_RESPONSE;
  } 
  else 
  {
    // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  return response;
}

// Setup() - initialization happens here
void modbusSetup() 
{

  // Init Serial2 connected to the RTU Modbus
  Serial2.begin(115200, SERIAL_8N1, OUT_TXD, IN_RXD);

  MBserver.registerWorker(SERVER_ID, READ_COIL, &read_coil_worker);
  MBserver.registerWorker(SERVER_ID, WRITE_COIL, &write_coil_worker);

  // Register served function code worker for server ID, FC 0x03
  MBserver.registerWorker(SERVER_ID, READ_HOLD_REGISTER,  &read_holding_worker);
  MBserver.registerWorker(SERVER_ID, WRITE_HOLD_REGISTER, &write_holding_worker);

  // Start ModbusRTU background task
  MBserver.start(CORE_ID,SERIAL_INTERVAL);

  Serial.println("MB started"); 
}

void doModbus(void)
{
  static uint32_t messageCount = 0;
  static unsigned long lastMessageTime = 0;
  static bool mb_active = false;

  if (MBserver.getMessageCount() != messageCount)
  {
    // message received
    messageCount = MBserver.getMessageCount();
    lastMessageTime = millis();
    
    if (!mb_active)
    {
      mb_active = true;
      Serial.println("Modbus active");
    }
  }
  else
  {
    if ( mb_active && ((millis() - lastMessageTime) > MB_TIMEOUT) )
    {
      // Timeout
      Serial.println("Modbus timeout, switch all off"); 

      mb_active = false;
      // All off
      mb_fast_mode = mist_stat = air_stat = LOW;
    }
  }
}

#endif
