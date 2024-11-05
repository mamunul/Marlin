#pragma once

#define ALLOW_STM32DUINO
#include "env_validate.h"

#define BOARD_INFO_NAME      "XETOZ F4"
#define DEFAULT_MACHINE_NAME "XETOZ"



//
// Limit Switches
//
// #define X_MIN_PIN                           PG9
// #define Y_MIN_PIN                           PG10
// #define Z_MIN_PIN                           PG11

// #define X_MAX_PIN                           PG12
// #define Y_MAX_PIN                           PG13
// #define Z_MAX_PIN                           PG14

//
// Limit Switches
//
#define X_STOP_PIN                          PB12
#define Y_STOP_PIN                          PB13
#define Z_STOP_PIN                          PB14



//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PC5

//
// Z Probe (when not Z_MIN_PIN)
//
#define Z_MIN_PROBE_PIN                   PG8


//
// Steppers
//
#define X_STEP_PIN                          PF7
#define X_DIR_PIN                           PF8
#define X_ENABLE_PIN                        PF6

#define Y_STEP_PIN                          PF10
#define Y_DIR_PIN                           PF11
#define Y_ENABLE_PIN                        PF9

#define Z_STEP_PIN                          PF13
#define Z_DIR_PIN                           PF14
#define Z_ENABLE_PIN                        PF12

#define E0_STEP_PIN                         PG0
#define E0_DIR_PIN                          PG1
#define E0_ENABLE_PIN                       PF15

#define E1_STEP_PIN                         PG3
#define E1_DIR_PIN                          PG4
#define E1_ENABLE_PIN                       PG2


//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // See below for activation of thermistor readings
#define TEMP_1_PIN                          PC1   // See below for activation of thermistor readings
#define TEMP_BED_PIN                        PC3


//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA0
#define HEATER_1_PIN                        PA1
#define HEATER_BED_PIN                      PA3

#define FAN0_PIN                            PA15  // heater 0 fan 1
#define FAN1_PIN                            PB10  // heater 1 fan 2


//
// SD support
//
#define SDCARD_CONNECTION            ONBOARD
#define ONBOARD_SDIO
#define SDIO_CLOCK                       4800000
#define SD_DETECT_PIN                       PG15

//#define SPI_EEPROM
//#define I2C_EEPROM

#if ENABLED(SPI_EEPROM)                           // SPI EEPROM Winbond W25Q128 (128Mbits) https://www.pjrc.com/teensy/W25Q128FV.pdf
  #define SPI_CHAN_EEPROM1                     1
  #define SPI_EEPROM1_CS_PIN                PB12  // datasheet: /CS pin, found with multimeter, not tested
  #define EEPROM_SCK_PIN                    PB13  // datasheet: CLK pin, found with multimeter, not tested
  #define EEPROM_MISO_PIN                   PB14  // datasheet: DO pin, found with multimeter, not tested
  #define EEPROM_MOSI_PIN                   PB15  // datasheet: DI pin, found with multimeter, not tested
  #define EEPROM_PAGE_SIZE               0x1000U  // 4K (from datasheet)
  #define MARLIN_EEPROM_SIZE 16UL * (EEPROM_PAGE_SIZE)  // Limit to 64K for now...
#elif ENABLED(I2C_EEPROM)                         // FM24CL64BG (CYP1813) 64Kbit F-RAM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C
  #define I2C_SDA_PIN                       PG13
  #define I2C_SCL_PIN                       PG14  // To be confirmed on the Lerdge S, but probably same as the K
  #define MARLIN_EEPROM_SIZE             0x2000U  // 8K
#else
  #define MARLIN_EEPROM_SIZE              0x800U  // On SD, Limit to 2K, require this amount of RAM
#endif


//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PF13  // PS-ON
#endif


  #define TFT_CS_PIN                 EXP2_04_PIN
  #define TFT_A0_PIN                 EXP2_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *          ------
   *      RX | 8  7 | 3.3V      GPIO0  PF14 ... Leave as unused (ESP3D software configures this with a pullup so OK to leave as floating)
   *   GPIO0 | 6  5 | Reset     GPIO2  PF15 ... must be high (ESP3D software configures this with a pullup so OK to leave as floating)
   *   GPIO2 | 4  3 | Enable    Reset  PG0  ... active low, probably OK to leave floating
   *     GND | 2  1 | TX        Enable PG1  ... Must be high for module to run
   *          ------
   *            W1
   */
  #define ESP_WIFI_MODULE_COM                  6  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PG0
  #define ESP_WIFI_MODULE_ENABLE_PIN        PG1
  #define ESP_WIFI_MODULE_GPIO0_PIN         PF14
  #define ESP_WIFI_MODULE_GPIO2_PIN         PF15
#endif