#include <Arduino.h>
#include "settings.h"
#include "Log.h"
#include "i2c.h"
#include <MPR121.h>
#include "ButtonMPR121.h"
#include "Button.h"

// Init MPR121 Touch-Buttons
#ifdef PORT_TOUCHMPR121_ENABLE
MPR121_type touchsensor = MPR121_type();
const int numElectrodes = 3;
t_button touchbuttons[numElectrodes -1];    // next + prev + pplay + rotEnc + button4 + button5 + dummy-button
unsigned long MPR121_LastCheckTimestamp;

static void ButtonMPR121_Task(void *parameter);

void ButtonMPR121_update() {
    touchsensor.updateTouchData();
}
#endif

void ButtonMPR121_Init() {
#ifdef PORT_TOUCHMPR121_ENABLE
/*    if (touchsensor.isInited()) {
//                 Log_Println((char *) FPSTR(mpr121_initiated), LOGLEVEL_INFO);
        Serial.println("MPR already initiated - checking for RUN");
        if (!touchsensor.isRunning()) {
//                 Log_Println((char *) FPSTR(mpr121_starting), LOGLEVEL_INFO);
            Serial.println("MPR not running - start up");
            touchsensor.run();
        } else {
//                 Log_Println((char *) FPSTR(mpr121_running), LOGLEVEL_INFO);
            Serial.println("MPR Running - nothing more to do");
        }
    } else { */
    touchsensor.clearError();
    if (!touchsensor.begin(MPR121_I2C_ADR, 40, 20, MPR121_IRQ_PIN, &i2cBusTwo)) {
            Serial.print("MPR121 ERROR: ");
        switch (touchsensor.getError()) {
            case NO_ERROR:
                Serial.println("no error");
                break;
            case ADDRESS_UNKNOWN:
                Serial.println("incorrect address");
                break;
            case READBACK_FAIL:
                Serial.println("readback failure");
                break;
            case OVERCURRENT_FLAG:
                Serial.println("overcurrent on REXT pin");
                break;
            case OUT_OF_RANGE:
                Serial.println("electrode out of range");
                break;
            case NOT_INITED:
                Serial.println("not initialised");
                break;
            default:
                Serial.println("unknown error");
                break;
            }
//                 Log_Println((char *) FPSTR(mpr121_error), LOGLEVEL_ERROR);
        while (1);
//                 Log_Println((char *) FPSTR(mpr121_initiated), LOGLEVEL_INFO);
        if (touchsensor.isInited()) {Serial.println("MPR121 initialized");}
//                 Log_Println((char *) FPSTR(mpr121_running), LOGLEVEL_INFO);
        if (touchsensor.isRunning()) {Serial.println("MPR121 running");}
//    }
    }

    touchsensor.autoSetElectrodes();  // autoset all electrode settings
    touchsensor.updateTouchData();

    xTaskCreatePinnedToCore(
        ButtonMPR121_Task,              /* Function to implement the task */
        "mpr121",                 /* Name of the task */
        1536,                   /* Stack size in words */
        NULL,                   /* Task input parameter */
        2,          /* Priority of the task */
//                1 | portPRIVILEGE_BIT,  /* Priority of the task */
        NULL,                   /* Task handle. */
        1          /* Core where the task should run */
    );

  #endif
}

#ifdef PORT_TOUCHMPR121_ENABLE
void ButtonMPR121_Task(void *parameter) {
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 250 / portTICK_RATE_MS;

		xLastWakeTime = xTaskGetTickCount();

    for (;;) {
    // Run Task only when needed
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

//                 Log_Println((char *) FPSTR(mpr121_IRQ), LOGLEVEL_DEBUG);
    if (touchsensor.touchStatusChanged()) { // temporary should be called external to wake task
        i2c_tsafe_execute(ButtonMPR121_update,50);
        if (touchsensor.getNumTouches() <=2){
            for (int i = 0; i < numElectrodes; i++) {
                if (touchsensor.isNewTouch(i)) {
//                 Log_Println((char *) FPSTR(mpr121_IRQ), LOGLEVEL_DEBUG);
                    Serial.print(i, DEC);
                    Serial.println(" was just touched");
                } else if (touchsensor.isNewRelease(i)) {
//                 Log_Println((char *) FPSTR(mpr121_IRQ), LOGLEVEL_DEBUG);
                    Serial.print(i, DEC);
                    Serial.println(" was just released");
                }
            }
        }
    }}
}
#endif

void ButtonMPR121_Cyclic() {
#ifdef PORT_TOUCHMPR121_ENABLE
    if (touchsensor.touchStatusChanged()) {
        //WakeUp Task handling the Work
    }

#endif
}

void ButtonMPR121_Exit() {
#ifdef PORT_TOUCHMPR121_ENABLE 
/*  // Only useful when not using IRQ for MPR121
    if (touchsensor.isRunning()) {
//                 Log_Println((char *) FPSTR(mpr121_stop), LOGLEVEL_INFO);
        Serial.println("MPR Running - going to sleep");
        touchsensor.stop();
    }
    */
#endif
}

/* Handling der Buttons fehlt
void  onButtonPressed(Button& btn){
    if (button1.is(btn)) { touchbuttons[0].currentState = false; Serial.println("electrode 1 released"); }
    if (button2.is(btn)) { touchbuttons[1].currentState = false; Serial.println("electrode 2 released"); }
    if (button3.is(btn)) { touchbuttons[2].currentState = false; Serial.println("electrode 3 released"); }
    if (button4.is(btn)) { touchbuttons[3].currentState = false; Serial.println("electrode 4 released"); }
    if (button5.is(btn)) { touchbuttons[4].currentState = false; Serial.println("electrode 5 released"); }
    if (button6.is(btn)) { touchbuttons[5].currentState = false; Serial.println("electrode 6 released"); }
}

void portMPR121Handler(void) {
    if(digitalRead(MPR121_IRQ_PIN) == LOW){
        Serial.println("MPRHandler called  ->  IRQ LOW");
        // Get the latest touch readings from the sensor. Do this here means we only call .touched() once instead of 6 times
        int latestTouchReading = touchSensor.touched();
        button1.update(latestTouchReading);
        button2.update(latestTouchReading);
        button3.update(latestTouchReading);
        button4.update(latestTouchReading);
        button5.update(latestTouchReading);
        button6.update(latestTouchReading);
    }
}

Temp();
#endif
*/