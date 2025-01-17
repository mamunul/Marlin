#pragma once

#define ALLOW_STM32DUINO
#include "env_validate.h"

#define BOARD_INFO_NAME      "XETOZ F4"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME



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
#define X_STOP_PIN                          PC7
#define Y_STOP_PIN                          PB1
#define Z_STOP_PIN                          PA15



//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PC6

//
// Z Probe (when not Z_MIN_PIN)
//
#define Z_MIN_PROBE_PIN                   PB9


//
// Steppers
//
#define X_STEP_PIN                          PC0
#define X_DIR_PIN                           PC2
#define X_ENABLE_PIN                        PC1

#define Y_STEP_PIN                          PB8
#define Y_DIR_PIN                           PC5
#define Y_ENABLE_PIN                        PC1

#define Z_STEP_PIN                          PB15
#define Z_DIR_PIN                           PB10
#define Z_ENABLE_PIN                        PC1

#define E0_STEP_PIN                         PA0
#define E0_DIR_PIN                          PA2
#define E0_ENABLE_PIN                       PC1

#define E1_STEP_PIN                         PC4
#define E1_DIR_PIN                          PB0
#define E1_ENABLE_PIN                       PC1


//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB5   // See below for activation of thermistor readings
// #define TEMP_1_PIN                          PC1   // See below for activation of thermistor readings
#define TEMP_BED_PIN                        PB3


//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB11
// #define HEATER_1_PIN                        PA1
#define HEATER_BED_PIN                      PC15

#define FAN0_PIN                            PC14  // heater 0 fan 1
// #define FAN1_PIN                            PB9  // heater 1 fan 2


//
// SD support
//
#define SDCARD_CONNECTION            ONBOARD
// #define SDIO
// #define HAL_SD_MODULE_ENABLED
#define ONBOARD_SDIO
// #define SDIO_CLOCK                       4800000
#define SD_DETECT_PIN                       PA8

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
  #define PS_ON_PIN                         PA4 // PS-ON
#endif

#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PA3   // Power Loss Detection: PWR-DET
#endif

#define FREEZE_FEATURE
#if ENABLED(FREEZE_FEATURE)
  #define FREEZE_PIN  PA1   // Override the default (KILL) pin here
  #define FREEZE_STATE LOW  // State of pin indicating freeze
#endif
#define KILL_PIN                            PC3

//
// Misc. Functions
//
#define LED_PIN                             PB2
#define BEEPER_PIN                          PB4


#define TFT_CS_PIN                 PB14
#define TFT_A0_PIN                 PB12
#define TFT_SCK_PIN                PA5
#define TFT_MISO_PIN               PA6
#define TFT_MOSI_PIN               PA7

// ENABLED(ESP3D_WIFISUPPORT)
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
  #define ESP_WIFI_MODULE_RESET_PIN         -1
  #define ESP_WIFI_MODULE_ENABLE_PIN        -1
  #define ESP_WIFI_MODULE_GPIO0_PIN         -1
  #define ESP_WIFI_MODULE_GPIO2_PIN         -1
#endif