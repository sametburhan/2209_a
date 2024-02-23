#include <Arduino.h>

#define buzzerHigh_pin GPIO_NUM_4 // DAC pin
#define voltage_pin GPIO_NUM_39   // analog pin
/**
 * Wifi çalışıyorken ADC2 çalışmıyor bütün pinler iptal!!!
 * ADC1 pinleri kullanımda
 */

bool Voltage_Measure(uint8_t set_thrust);
void Voltage_Init();

void Voltage_Init()
{
    pinMode(voltage_pin, INPUT);
    pinMode(buzzerHigh_pin, OUTPUT);
    digitalWrite(buzzerHigh_pin, LOW);
}

bool Voltage_Measure(uint8_t set_thrust)
{
    float voltage = analogRead(voltage_pin);
    // Serial.print(voltage);
    // Serial.println("v");
    if (voltage < 2100 && set_thrust == 0)
    {
        digitalWrite(buzzerHigh_pin, HIGH);
        return true;
    }
    else
    {
        digitalWrite(buzzerHigh_pin, LOW);
        return false;
    }
}