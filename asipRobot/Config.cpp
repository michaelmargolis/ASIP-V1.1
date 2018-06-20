
/*
 * Configuration data  
 */

#include "config.h" 
#include "utility/debug.h" // this file is in the ASIP library folder
#include <EEPROM.h>



//#define CONFIG_DEBUG
pidCfg_t configData;

void setConfigDefaults( )
{
  configData.Kp = 20;
  configData.Kd = 12;
  configData.Ki = 0;
  configData.Ko = 20;  
}
 
// write header and save core data
void saveConfig()
{
#ifdef  CONFIG_DEBUG 
  debug_printf("saving config\n");
#endif   
  int8_t *cfgPtr;  
  unsigned int offset;
  asipCfgPreamble_t preamble = {CONFIG_ID, CONFIG_VERSION};
  cfgPtr = (int8_t*)&preamble; 
 
  for(offset=0; offset < sizeof( preamble); offset++)  {
     EEPROM.write( offset,  cfgPtr[offset] );    // preamble is stored at the start of EEPROM memory space          
  } 
  cfgPtr = (int8_t*)&configData;    
  for(unsigned int i=0; i < sizeof( configData); i++)  
    EEPROM.write( offset + i , cfgPtr[i] );        
#ifdef  CONFIG_DEBUG 
   for(int i=0; i < sizeof( configData); i++)  {
     debug_printf("%d,", cfgPtr[i]);
   }
   debug_printf("\n");
#endif    
}


// returns true if valid config data can be restored from EEPROM
// will return false if eeprom has not been configured or invalid version 
bool restoreConfig()
{
#ifdef  CONFIG_DEBUG 
  debug_printf("restoring config\n");
#endif  
  bool ret = false;
  int8_t *cfgPtr; 
  unsigned int offset;
  // first check if config is valid 
  asipCfgPreamble_t preamble;
  cfgPtr = (int8_t*)&preamble;
  //debug_printf("reading: ");
  for(offset=0; offset < sizeof( preamble); offset++)  {
     cfgPtr[offset] = EEPROM.read( offset);    // preamble is stored at the start of EEPROM memory space          
   }      
   if(preamble.configId == CONFIG_ID)
   {
#ifdef  CONFIG_DEBUG     
      debug_printf("Using config from EEPROM\n");
#endif        
      if(preamble.version == CONFIG_VERSION)
      {                            
          int bodyOffset = offsetof(robotCfgData_t, body);   // note  another way to do this is to use offset from obove 
          int8_t *cfgPtr = (int8_t*)&configData;
          for(unsigned int i=0; i < sizeof( pidCfg_t); i++)  {
             cfgPtr[i] = EEPROM.read( bodyOffset + i );                    
          }                             
#ifdef  CONFIG_DEBUG 
         debug_printf("after reading eeprom");  
         debug_printf(" Kp= %d", configData.Kp );
         debug_printf(", Ki= %d", configData.Ki );
         debug_printf(", Kd= %d", configData.Kd );
         debug_printf(", Ko= %d", configData.Ko );             
#endif 
      ret = true; 
      }
      else
      {  
        debug_printf("Config version mismatch!\n");  
      }    
   } 
   else
   {  
       debug_printf("Using default config !!!\n");   
       setConfigDefaults();      
   }        
   return ret;
}

/*
 *---------------- end config ---------------
 */