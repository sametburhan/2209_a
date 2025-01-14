#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include "PID.cpp"
#include <ESP32Servo.h>
#include "movingAvg.h"

#define Deg2rad 3.1415 / 180

void Idle(double roll, double pitch, double yaw);
void spinMotors(struct MotorPowers motorPowers);

movingAvg roll_mov(5);
movingAvg pitch_mov(5);
movingAvg yaw_mov(5);

Madgwick MadgwickFilter;
Servo frontLeftMotorPower;
Servo frontRightMotorPower;
Servo rearLeftMotorPower;
Servo rearRightMotorPower;

// Değişkenler
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MgX, MgY, MgZ;
float SF_Acc = 16384;
float SF_Gy = 131;
float SF_Mg = 500;
float SF_Tmp = 333.87;
float g2mpss = 9.80665;
float deg2rad = 3.14159265 / 180;
int i;
float ROLL, PITCH, YAW;

volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer1()
{

    portENTER_CRITICAL_ISR(&timerMux);
    timeCounter1++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void MPU_Init();
void MPU();
void output();
void read_mpu_6050_data();

void MPU_Init()
{
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);       // Send the requested starting register
    Wire.write(0x00);       // Set the requested starting register
    Wire.endTransmission(); // End the transmission
    // Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68); // Start communicating with the MPU-6050
    Wire.write(0x1C);             // Send the requested starting register
    Wire.write(0x10);             // Set the requested starting register
    Wire.endTransmission();       // End the transmission
    // Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68); // Start communicating with the MPU-6050
    Wire.write(0x1B);             // Send the requested starting register
    Wire.write(0x08);             // Set the requested starting register
    Wire.endTransmission();       // End the transmission

    Wire.beginTransmission(0x68); // Start communication with the address found during search
    Wire.write(0x1A);             // We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);             // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();       // End the transmission with the gyro
    Serial.println("MPU6050 Hazır.");

    roll_mov.begin();
    pitch_mov.begin();
    yaw_mov.begin();
}

void MPU()
{
    unsigned long time;
    time = millis();
    if (timeCounter1 > 0)
    {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);
        if (Pid_Flag == 1)
        {
            read_mpu_6050_data(); // Read the raw acc and gyro data from the MPU-6050
            MadgwickFilter.update(GyX / SF_Gy, GyY / SF_Gy, GyZ / SF_Gy, AcX / SF_Acc, AcY / SF_Acc, AcZ / SF_Acc, MgX / SF_Mg, MgY / SF_Mg, MgZ / SF_Mg);
            ROLL = roll_mov.reading(MadgwickFilter.getRoll());
            PITCH = pitch_mov.reading(MadgwickFilter.getPitch());
            YAW = yaw_mov.reading(MadgwickFilter.getYaw());
            // output();
            Idle(PITCH, ROLL, YAW);
        }
        else if (Pid_Flag == 0)
        {
            resetPidVariables();
            ROLL = 0;
            PITCH = 0;
            last_time = 0;
        }
    }
}

void read_mpu_6050_data()
{
    // her data alırken i2c yolunu kontrol et bir sorun varsa güvenle in
    //  Subroutine for reading the raw gyro and accelerometer data
    Wire.beginTransmission(0x68); // Start communicating with the MPU-6050
    Wire.write(0x3B);             // Send the requested starting register
    Wire.endTransmission();       // End the transmission
    Wire.requestFrom(0x68, 14);   // Request 14 bytes from the MPU-6050
    while (Wire.available() < 14)
        ;                                 // Wait until all the bytes are received
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void output()
{
    Serial.print(ROLL);
    Serial.print(",");
    Serial.print(PITCH);
    Serial.print(",");
    Serial.print(YAW);
    Serial.print("\n");
}

void Idle(double roll, double pitch, double yaw)
{
    struct MotorPowers motorPowers = calculateMotorPowers(roll, pitch, yaw);
    spinMotors(motorPowers);
}

/****************************
 * Hareket fonksiyonları
 ***************************/
void spinMotors(struct MotorPowers motorPowers)
{
    // Serial.print("sol ön =");
    // Serial.print(motorPowers.frontLeftMotorPower);
    // Serial.print(",");
    // Serial.print("sağ ön =");
    // Serial.print(motorPowers.frontRightMotorPower);
    // Serial.print(",");
    // Serial.print("sol arka = ");
    // Serial.print(motorPowers.rearLeftMotorPower);
    // Serial.print(",");
    // Serial.print("sağ arka = ");
    // Serial.print(motorPowers.rearRightMotorPower);
    // Serial.print(",");
    // Serial.println("\n");
    frontLeftMotorPower.writeMicroseconds(motorPowers.frontLeftMotorPower);
    frontRightMotorPower.writeMicroseconds(motorPowers.frontRightMotorPower);
    rearLeftMotorPower.writeMicroseconds(motorPowers.rearLeftMotorPower);
    rearRightMotorPower.writeMicroseconds(motorPowers.rearRightMotorPower);
}