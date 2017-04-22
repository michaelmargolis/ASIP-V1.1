/*
 * Configuration data
 */

#ifndef config_h
#define config_h

#include <stdint.h>

const int16_t  CONFIG_ID      = 0x132; // unique number indicating that this is valid config
const uint16_t CONFIG_VERSION = 0x1;   // version number incremented when config format changes 

typedef struct {
  int16_t  configId;            // a magic value to identify valid config data
  uint16_t version;             // the configuration version number    
} asipCfgPreamble_t;

typedef struct {
   int16_t Kp;
   int16_t Ki;
   int16_t Kd;
   int16_t Ko;      // scaling parameter
   int8_t rfu[32];    // bytes reserved for future use    
} pidCfg_t;


typedef struct {
  asipCfgPreamble_t  preamble;
  pidCfg_t     body; 
}  robotCfgData_t;


//todo replace the following by requesting pointer to config data
extern pidCfg_t configData;

//#define CONFIG_DEBUG

void saveConfig();
bool restoreConfig();
void setConfigDefaults( );
pidCfg_t    *getPidConfig();
void updateConfig(pidCfg_t *cfgPtr);
void showConfig();

 
#endif