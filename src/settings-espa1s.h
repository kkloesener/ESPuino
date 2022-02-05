#ifndef __ESPUINO_SETTINGS_ESPA1S_H__
#define __ESPUINO_SETTINGS_ESPA1S_H__
    #include "Arduino.h"
    #include <settings.h>

    //######################### INFOS ####################################
    /* This is a develboard-specific config-file for *AI Tinker ESP32-A1S-AudioKit*. It's highly customized and almost certainly
    not suitable for a different develboard.
    Has a lot of stuff already onboard but needs some soldering rework as there are not all GPIOs exposed.
    PCB: Not necessary.
    Infos: https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit
           https://forum.espuino.de/t/esp32-audio-kit-esp32-a1s/106
    Status: tested by kkloesner with RC522-I2C
    */


    //################## GPIO-configuration ##############################
    #ifdef SD_MMC_1BIT_MODE
        // uSD-card-reader (via SD-MMC 1Bit)
        //
        // SD_MMC uses fixed pins
        //  MOSI    15
        //  SCK     14
        //  MISO    2
    #else
        // uSD-card-reader (via SPI)
        #define SPISD_CS                    13          // GPIO for chip select (SD)
        #ifndef SINGLE_SPI_ENABLE
            #define SPISD_MOSI              15          // GPIO for master out slave in (SD) => not necessary for single-SPI
            #define SPISD_MISO               2          // GPIO for master in slave ou (SD) => not necessary for single-SPI
            #define SPISD_SCK               14          // GPIO for clock-signal (SD) => not necessary for single-SPI
        #endif
    #endif

    // RFID (via SPI; currently not supported!)
    #if defined(RFID_READER_TYPE_MFRC522_SPI)
        #define RST_PIN                         99          // Not necessary but has to be set anyway; so let's use a dummy-number
        #define RFID_CS                         21          // GPIO for chip select (RFID)
        #define RFID_MOSI                       23          // GPIO for master out slave in (RFID)
        #define RFID_MISO                       19          // GPIO for master in slave out (RFID)
        #define RFID_SCK                        18          // GPIO for clock-signal (RFID)
    #endif

    // RFID (via I2C)
    #if defined(RFID_READER_TYPE_MFRC522_I2C)
        #define MFRC522_RST_PIN                 12          // needed for initialisation -> MTDI on JTAG header
        #define MFRC522_IRQ_PIN                 13          // currently not used -> MTCK on JTAG header
    #endif

    // I2C-configuration (necessary for RC522 [only via i2c - not spi!] or port-expander)
    #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(PORT_EXPANDER_ENABLE) || defined (PORT_TOUCHMPR121_ENABLE)
        #define ext_IIC_CLK                     18          // i2c-SCL (clock) [14 pin-header]
        #define ext_IIC_DATA                     5          // i2c-SDA (data) [14 pin-header]
    #endif

    #ifdef PORT_TOUCHMPR121_ENABLE
        #define MPR121_IRQ_PIN                  13          // MTCK on JTAG header -> Review WAKEUP_BUTTON!!
        #define MPR121_I2C_ADR                0x5A          // Default address is 0x5A, if tied to 3.3V its 0x5B. If tied to SDA its 0x5C and if SCL then 0x5D
    #endif

    // I2S (DAC)
    #define I2S_DOUT                            25          // Digital out (I2S)
    #define I2S_BCLK                            27          // BCLK (I2S)
    #define I2S_LRC                             26          // LRC (I2S)

    // I2C (AC101)
    #define IIC_CLK                             32          // internal
    #define IIC_DATA                            33          // internal

    // Amp enable
    #define GPIO_PA_EN                      21          // internal

    // Rotary encoder
    #ifdef USEROTARY_ENABLE
        //#define REVERSE_ROTARY                        // To reverse encoder's direction; switching CLK / DT in hardware does the same
        #define ROTARYENCODER_CLK           5           // rotary encoder's CLK
        #define ROTARYENCODER_DT            18          // Info: Lolin D32 / Lolin D32 pro 35 are using 35 for battery-voltage-monitoring!
    #endif

    // Control-buttons (set to 99 to DISABLE; 0->39 for GPIO; 100->115 for port-expander; 200->211 for mpr121-touchports )
    #define NEXT_BUTTON                        202          // Button 0: GPIO to detect next
    #define PREVIOUS_BUTTON                    200          // Button 1: GPIO to detect previous
    #define PAUSEPLAY_BUTTON                   201          // Button 2: GPIO to detect pause/play
    #define BUTTON_4                           203          // Button 4: unnamed optional button
    #define BUTTON_5                           204          // Button 5: unnamed optional button

    // Channels of port-expander can be read cyclic or interrupt-driven. It's strongly recommended to use the interrupt-way!
    // Infos: https://forum.espuino.de/t/einsatz-des-port-expanders-pca9555/306
    #ifdef PORT_EXPANDER_ENABLE
        #define PE_INTERRUPT_PIN            99          // GPIO that is used to receive interrupts from port-expander
    #endif

    // Wake-up button => this also is the interrupt-pin if port-expander is enabled!
    // Please note: only RTC-GPIOs (0, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39, 99) can be used! Set to 99 to DISABLE.
    // Please note #2: this button can be used as interrupt-pin for port-expander. If so, all pins connected to port-expander can wake up ESPuino.
    #ifdef PORT_TOUCHMPR121_ENABLE
    #define WAKEUP_BUTTON                       MPR121_IRQ_PIN // Defines the button that is used to wake up ESPuino from deepsleep.
    #else
    #define WAKEUP_BUTTON                       MFRC522_IRQ_PIN // Defines the button that is used to wake up ESPuino from deepsleep.
    #endif
    // Power-control
    #define POWER                           GPIO_PA_EN          // GPIO used to drive transistor-circuit, that switches off peripheral devices while ESP32-deepsleep
    #ifdef POWER
        //#define INVERT_POWER                          // If enabled, use inverted logic for POWER circuit, that means peripherals are turned off by writing HIGH
    #endif

    // (optional) Neopixel
    #if defined(NEOPIXEL_ENABLE)
        #define LED_PIN                         23          // GPIO for Neopixel-signaling
    #endif

    // (optinal) Headphone-detection
    #ifdef HEADPHONE_ADJUST_ENABLE
        //#define DETECT_HP_ON_HIGH                     // Per default headphones are supposed to be connected if HT_DETECT is LOW. DETECT_HP_ON_HIGH will change this behaviour to HIGH.
        #define HP_DETECT                   39          // GPIO that detects, if there's a plug in the headphone jack or not
    #endif

    // (optional) Monitoring of battery-voltage via ADC
    #ifdef MEASURE_BATTERY_VOLTAGE
        #define VOLTAGE_READ_PIN            33          // GPIO used to monitor battery-voltage. Change to 35 if you're using Lolin D32 or Lolin D32 pro as it's hard-wired there!
        constexpr float referenceVoltage = 3.30;                  // Voltage between 3.3V and GND-pin at the develboard in battery-mode (disconnect USB!)
        constexpr float offsetVoltage = 0.1;                      // If voltage measured by ESP isn't 100% accurate, you can add an correction-value here
    #endif

    // (optional) For measuring battery-voltage a voltage-divider is necessary. Their values need to be configured here.
    #ifdef MEASURE_BATTERY_VOLTAGE
        constexpr uint16_t rdiv1 = 129;                              // Rdiv1 of voltage-divider (kOhms) (measure exact value with multimeter!)
        constexpr uint16_t rdiv2 = 389;                              // Rdiv2 of voltage-divider (kOhms) (measure exact value with multimeter!) => used to measure voltage via ADC!
    #endif

    // (Optional) remote control via infrared
    #ifdef IR_CONTROL_ENABLE
        #define IRLED_PIN                   22              // GPIO where IR-receiver is connected (only tested with VS1838B)
        #define IR_DEBOUNCE                 200             // Interval in ms to wait at least for next signal (not used for actions volume up/down)

        // Actions available. Use your own remote control and have a look at the console for "Command=0x??". E.g. "Protocol=NEC Address=0x17F Command=0x68 Repeat gap=39750us"
        // Make sure to define a hex-code not more than once as this will lead to a compile-error
        // https://forum.espuino.de/t/neues-feature-fernsteuerung-per-infrarot-fernbedienung/265
        #define RC_PLAY                     0x68            // command for play
        #define RC_PAUSE                    0x67            // command for pause
        #define RC_NEXT                     0x6b            // command for next track of playlist
        #define RC_PREVIOUS                 0x6a            // command for previous track of playlist
        #define RC_FIRST                    0x6c            // command for first track of playlist
        #define RC_LAST                     0x6d            // command for last track of playlist
        #define RC_VOL_UP                   0x1a            // Command for volume up (one step)
        #define RC_VOL_DOWN                 0x1b            // Command for volume down (one step)
        #define RC_MUTE                     0x1c            // Command to mute ESPuino
        #define RC_SHUTDOWN                 0x2a            // Command for deepsleep
        #define RC_BLUETOOTH                0x72            // Command to enable/disable bluetooth
        #define RC_FTP                      0x65            // Command to enable FTP-server
    #endif
#endif