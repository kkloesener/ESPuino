// !!! MAKE SURE TO EDIT settings.h !!!
#include <Arduino.h>
#include "settings.h" // Contains all user-relevant settings (general)

#include "i2c.h"
#include "AudioPlayer.h"
#include "Battery.h"
#include "Bluetooth.h"
#include "Button.h"
#include "Cmd.h"
#include "Common.h"
#include "Ftp.h"
#include "IrReceiver.h"
#include "Led.h"
#include "Log.h"
#include "Mqtt.h"
#include "MemX.h"
#include "Port.h"
#include "Queues.h"
#include "Rfid.h"
#include "RotaryEncoder.h"
#include "SdCard.h"
#include "System.h"
#include "Web.h"
#include "Wlan.h"
#include "revision.h"
#include "Power.h"

#ifdef PLAY_LAST_RFID_AFTER_REBOOT
    bool recoverLastRfid = true;
    bool recoverBootCount = true;
    bool resetBootCount = false;
    uint32_t bootCount = 0;
#endif

////////////

#ifdef PLAY_LAST_RFID_AFTER_REBOOT
    // If a problem occurs, remembering last rfid can lead into a boot loop that's hard to escape of.
    // That reason for a mechanism is necessary to prevent this.
    // At start of a boot, bootCount is incremented by one and after 30s decremented because
    // uptime of 30s is considered as "successful boot".
    void recoverBootCountFromNvs(void) {
        if (recoverBootCount) {
            recoverBootCount = false;
            resetBootCount = true;
            bootCount = gPrefsSettings.getUInt("bootCount", 999);

            if (bootCount == 999) {         // first init
                bootCount = 1;
                gPrefsSettings.putUInt("bootCount", bootCount);
            } else if (bootCount >= 3) {    // considered being a bootloop => don't recover last rfid!
                bootCount = 1;
                gPrefsSettings.putUInt("bootCount", bootCount);
                gPrefsSettings.putString("lastRfid", "-1");     // reset last rfid
                Log_Println((char *) FPSTR(bootLoopDetected), LOGLEVEL_ERROR);
                recoverLastRfid = false;
            } else {                        // normal operation
                gPrefsSettings.putUInt("bootCount", ++bootCount);
            }
        }

        if (resetBootCount && millis() >= 30000) {      // reset bootcount
            resetBootCount = false;
            bootCount = 0;
            gPrefsSettings.putUInt("bootCount", bootCount);
            Log_Println((char *) FPSTR(noBootLoopDetected), LOGLEVEL_INFO);
        }
    }

    // Get last RFID-tag applied from NVS
    void recoverLastRfidPlayedFromNvs(void) {
        if (recoverLastRfid) {
            if (System_GetOperationMode() == OPMODE_BLUETOOTH) { // Don't recover if BT-mode is desired
                recoverLastRfid = false;
                return;
            }
            recoverLastRfid = false;
            String lastRfidPlayed = gPrefsSettings.getString("lastRfid", "-1");
            if (!lastRfidPlayed.compareTo("-1")) {
                Log_Println((char *) FPSTR(unableToRestoreLastRfidFromNVS), LOGLEVEL_INFO);
            } else {
                char *lastRfid = x_strdup(lastRfidPlayed.c_str());
                xQueueSend(gRfidCardQueue, lastRfid, 0);
                snprintf(Log_Buffer, Log_BufferLength, "%s: %s", (char *) FPSTR(restoredLastRfidFromNVS), lastRfidPlayed.c_str());
                Log_Println(Log_Buffer, LOGLEVEL_INFO);
            }
        }
    }
#endif


void setup() {
    Log_Init();
    Queues_Init();

    // Make sure all wakeups can be enabled *before* initializing RFID, which can enter sleep immediately
    Button_Init();  // To preseed internal button-storage with values
    #ifdef PN5180_ENABLE_LPCD
        Rfid_Init();
    #endif

    System_Init();

    // I2C wird zentral behandelt
    i2c_Init();
    // Needs i2c first if port-expander is used
    Port_Init();

    // If port-expander is used, port_init has to be called first, as power can be (possibly) done by port-expander
    Power_Init();

    Battery_Init();

    // Init audio before power on to avoid speaker noise
    AudioPlayer_Init();

    // All checks that could send us to sleep are done, power up fully
    Power_PeripheralOn();

    memset(&gPlayProperties, 0, sizeof(gPlayProperties));
    gPlayProperties.playlistFinished = true;

    Led_Init();

    // Needs power first
    SdCard_Init();

    // welcome message
    Serial.println(F(""));
    Serial.println(F("  _____   ____    ____            _                 "));
    Serial.println(F(" | ____| / ___|  |  _ \\   _   _  (_)  _ __     ___  "));
    Serial.println(F(" |  _|   \\__  \\  | |_) | | | | | | | | '_ \\   / _ \\"));
    Serial.println(F(" | |___   ___) | |  __/  | |_| | | | | | | | | (_) |"));
    Serial.println(F(" |_____| |____/  |_|      \\__,_| |_| |_| |_|  \\___/ "));
    Serial.print(F(" Rfid-controlled musicplayer\n\n"));
    Serial.printf("%s\n\n", softwareRevision);
    Serial.println("ESP-IDF version: " + String(ESP.getSdkVersion()));

    // print wake-up reason
    System_ShowWakeUpReason();
	// print SD card info
    SdCard_PrintInfo();

    Ftp_Init();
    Mqtt_Init();
    #ifndef PN5180_ENABLE_LPCD
        Rfid_Init();
    #endif

    RotaryEncoder_Init();
    Wlan_Init();
    Bluetooth_Init();

    if (OPMODE_NORMAL == System_GetOperationMode()) {
        Wlan_Cyclic();
    }

    IrReceiver_Init();
    System_UpdateActivityTimer(); // initial set after boot
    
    snprintf(Log_Buffer, Log_BufferLength, "%s: %u", (char *) FPSTR(freeHeapAfterSetup), ESP.getFreeHeap());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    snprintf(Log_Buffer, Log_BufferLength, "PSRAM: %u bytes", ESP.getPsramSize());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    snprintf(Log_Buffer, Log_BufferLength, "Flash-size: %u bytes", ESP.getFlashChipSize());
    Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
/*  Will be called Cyclic
    if (Wlan_IsConnected()) {
        snprintf(Log_Buffer, Log_BufferLength, "RSSI: %d dBm", Wlan_GetRssi());
        Log_Println(Log_Buffer, LOGLEVEL_DEBUG);
    }
*/
    System_ShowUpgradeWarning();
    vTaskDelay(100);
    Led_Indicate(LedIndicatorType::BootComplete);
}

void loop() {
    if (OPMODE_BLUETOOTH == System_GetOperationMode()) {
        Bluetooth_Cyclic();
    } else {
        Wlan_Cyclic();
        Web_Cyclic();
        Ftp_Cyclic();
        RotaryEncoder_Cyclic();
        Mqtt_Cyclic();
    }

    AudioPlayer_Cyclic();
    Battery_Cyclic();
    //Port_Cyclic(); // called by button (controlled via hw-timer)
    Button_Cyclic();
    System_Cyclic();
    Rfid_PreferenceLookupHandler();

    #ifdef PLAY_LAST_RFID_AFTER_REBOOT
        recoverBootCountFromNvs();
        recoverLastRfidPlayedFromNvs();
    #endif

    IrReceiver_Cyclic();
    vTaskDelay(portTICK_RATE_MS * 5u);
}
