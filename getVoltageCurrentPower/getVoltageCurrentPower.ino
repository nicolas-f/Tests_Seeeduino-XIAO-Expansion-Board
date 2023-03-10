/*!
 *@file getVoltageCurrentPower.ino
 *@brief Get the current, voltage, and power of electronic devices.
 *@copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fengli](li.feng@dfrobot.com)
 *@version  V1.0
 *@date  2022-3-1
 *@url https://github.com/DFRobot/DFRobot_INA219
*/

#include <Wire.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include "DFRobot_INA219.h"

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/ PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

 
/**
 * @fn DFRobot_INA219_IIC
 * @brief pWire I2C controller pointer
 * @param i2caddr  I2C address
 * @n INA219_I2C_ADDRESS1  0x40   A0 = 0  A1 = 0
 * @n INA219_I2C_ADDRESS2  0x41   A0 = 1  A1 = 0
 * @n INA219_I2C_ADDRESS3  0x44   A0 = 0  A1 = 1
 * @n INA219_I2C_ADDRESS4  0x45   A0 = 1  A1 = 1	 
  */
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS4);

// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

void setup(void) 
{
    Wire.begin();

    //Initialize the sensor
    Serial.println("Initialize the sensor");
    while(ina219.begin() != true) {
        Serial.begin(115200);    
        delay(100);
        Serial.println();
        Serial.println("INA219 begin faild");

        // look for I2C
        byte error, address;
        int nDevices;
    
        Serial.println("Scanning...");
    
        nDevices = 0;
        for(address = 0; address <= 127; address++ )
        {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0)
            {
                Serial.print("I2C device found at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.print(address, HEX);
                Serial.println(" !");
                nDevices++;
            }
            else if (error==4)
            {
                Serial.print("Unknow error at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.println(address,HEX);
            }
        }
        if (nDevices == 0)
            Serial.println("No I2C devices found\n");
        else
            Serial.println("done\n");
        delay(2000);
    }

    //Linear calibration
    ina219.linearCalibrate(/*The measured current before calibration*/ina219Reading_mA, /*The current measured by other current testers*/extMeterReading_mA);
    u8x8.setFlipMode(1);   // set number from 1 to 3, the screen word will rotary 180
    u8x8.begin();
}

void loop(void)
{
    //Serial.print("BusVoltage:   ");
    //Serial.print(ina219.getBusVoltage_V(), 2);
    //Serial.println("V");
    //Serial.print("ShuntVoltage: ");
    //Serial.print(ina219.getShuntVoltage_mV(), 3);
    //Serial.println("mV");
    //Serial.print("Current:      ");
    //Serial.print(ina219.getCurrent_mA(), 1);
    //Serial.println("mA");
    //Serial.print("Power:        ");
    //Serial.print(ina219.getPower_mW(), 1);
    //Serial.println("mW");
    //Serial.println("");
    u8x8.clear();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print(String(ina219.getBusVoltage_V(), 2) + String(" V"));
    u8x8.setCursor(0, 9);
    u8x8.print(String(ina219.getCurrent_mA(), 2) + String(" mA"));
    u8x8.setCursor(0, 18);
    u8x8.print(String(ina219.getPower_mW(), 2) + String(" mW"));
    //for(int N = 1; N <= 100000; N++)
   // {
    //  int count = 0;
    //  for (int i = 2; i <= N/2; i++)
    //  {
    //    if(N%i == 0)
    //    {
    //      count++;
    //      break;
    //    }
    //  }
    //}
    delay(1000);
}
