#include <Arduino.h>
#include "settings.h"
#include "i2c.h"

#if (HAL == 2)
    extern TwoWire i2cBusOne = TwoWire(0);
    extern AC101 ac(&i2cBusOne);
#endif

#if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(PORT_EXPANDER_ENABLE) || defined (PORT_TOUCHMPR121_ENABLE)
    extern TwoWire i2cBusTwo = TwoWire(1);
#endif

void i2c_Init() {

    #if (HAL == 2)
        i2c_clear_lines(IIC_DATA, IIC_CLK);
        i2cBusOne.begin(IIC_DATA, IIC_CLK, 40000);

        ac.begin();
        pinMode(22, OUTPUT);
        digitalWrite(22, HIGH);

        pinMode(GPIO_PA_EN, OUTPUT);
        digitalWrite(GPIO_PA_EN, HIGH);

    #endif

    #if defined(RFID_READER_TYPE_MFRC522_I2C) || defined(PORT_EXPANDER_ENABLE) || defined (PORT_TOUCHMPR121_ENABLE)
        i2c_clear_lines(ext_IIC_DATA, ext_IIC_CLK);
        i2cBusTwo.begin(ext_IIC_DATA, ext_IIC_CLK, 40000);
        delay(50);
    #endif
}

void i2c_clear_lines(int PIN_SDA, int PIN_SCL) {
    boolean stuck_transaction = false;
    uint8_t stuck_transaction_retry_count = 0;
    const uint8_t stuck_transaction_retry_MAX = 10;

    ::pinMode( PIN_SDA, INPUT_PULLUP );
    ::pinMode( PIN_SCL, INPUT_PULLUP );

    do{
        if(( ::digitalRead( PIN_SDA ) == LOW ) && ( ::digitalRead( PIN_SCL ) == HIGH )){
        stuck_transaction = true;
        ::pinMode( PIN_SCL, OUTPUT );
        ::digitalWrite( PIN_SCL, LOW );
        delay( 1 ); // this is way longer than required (would be 1.25us at 400kHz) but err on side of caution
        ::pinMode( PIN_SCL, INPUT_PULLUP );
        stuck_transaction_retry_count++;
        } else {
        stuck_transaction = false;
        }
    } while ( stuck_transaction && ( stuck_transaction_retry_count < stuck_transaction_retry_MAX ));

    // TODO: add new error code that can be handled externally
    if( stuck_transaction_retry_count > 0){
        if( stuck_transaction ){
        } else {
        }
    }
}