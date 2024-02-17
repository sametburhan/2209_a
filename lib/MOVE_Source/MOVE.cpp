#include "MPU.cpp"
#include "CONTROL.cpp"

#define motor_solon_1 5
#define motor_sagon_2 23
#define motor_solarka_3 19
#define motor_sagarka_4 18

ESP32PWM pwm;

void PWM_Init();
void Thrust();
void Move();
void stopMotors();
void throttledown(void);
MotorPowers MoveMotorPowers(uint8_t up, uint8_t down, uint8_t right, uint8_t left);

/****************************
 * Variables
 ***************************/

volatile int throttle = minUs;

/****************************
 * PWM fonksiyonları
 ***************************/
void PWM_Init()
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    frontLeftMotorPower.setPeriodHertz(100);  // Standard 100hz
    frontRightMotorPower.setPeriodHertz(100); // Standard 100hz
    rearLeftMotorPower.setPeriodHertz(100);   // Standard 100hz
    rearRightMotorPower.setPeriodHertz(100);  // Standard 100hz

    frontLeftMotorPower.attach(motor_solon_1, minUs, maxUs);
    frontRightMotorPower.attach(motor_sagon_2, minUs, maxUs);
    rearLeftMotorPower.attach(motor_solarka_3, minUs, maxUs);
    rearRightMotorPower.attach(motor_sagarka_4, minUs, maxUs);

    stopMotors();
}

void throttledown(void)
{
    if (throttle > 1000)
    {
        Pid_Flag = 0;
        for (uint8_t temp = set_thrust; temp > 0; temp--)
        {
            frontLeftMotorPower.writeMicroseconds(map(temp, 0, 100, minUs, maxUs));
            frontRightMotorPower.writeMicroseconds(map(temp, 0, 100, minUs, maxUs));
            rearLeftMotorPower.writeMicroseconds(map(temp + 10, 0, 100, minUs, maxUs));
            rearRightMotorPower.writeMicroseconds(map(temp - 2, 0, 100, minUs, maxUs));
            delay(150);
        }
        throttle = 1000;
    }
}

void stopMotors()
{
    frontLeftMotorPower.write(0);
    frontRightMotorPower.write(0);
    rearLeftMotorPower.write(0);
    rearRightMotorPower.write(0);
}

void Thrust()
{
    if (RemoteXY.thrust != set_thrust)
    {
        set_thrust = RemoteXY.thrust;
        throttle = map(set_thrust, 0, 100, minUs, maxUs);
        frontLeftMotorPower.writeMicroseconds(throttle);
        frontRightMotorPower.writeMicroseconds(throttle);
        rearLeftMotorPower.writeMicroseconds(map(set_thrust + 10, 0, 100, minUs, maxUs));
        rearRightMotorPower.writeMicroseconds(map(set_thrust - 2, 0, 100, minUs, maxUs));

        if (set_thrust > 15)
        {
            Pid_Flag = 1;
        }
        else
        {
            Pid_Flag = 0;
        }
    }
}

/****************************
 * Manevra fonksiyonları
 ***************************/
struct MotorPowers MoveMotorPowers(uint8_t up, uint8_t down, uint8_t right, uint8_t left)
{
    struct MotorPowers motorPowers;
    motorPowers.frontLeftMotorPower = map(set_thrust + down + right, 0, 100, minUs, maxUs);
    motorPowers.frontRightMotorPower = map(set_thrust + down + left, 0, 100, minUs, maxUs);
    motorPowers.rearLeftMotorPower = map(set_thrust + up + right + 10, 0, 100, minUs, maxUs);
    motorPowers.rearRightMotorPower = map(set_thrust + up + left - 2, 0, 100, minUs, maxUs);
    return motorPowers;
}

void Move()
{
    if ((RemoteXY.up || RemoteXY.down || RemoteXY.left || RemoteXY.right) && Pid_Flag)
    {
        while (RemoteXY.up)
        {
            struct MotorPowers motorPowers = MoveMotorPowers(7, 0, 0, 0);
            spinMotors(motorPowers);
            Serial.println("ileri");
            Pid_Flag = 0;
            delay(1);
        }
        while (RemoteXY.down)
        {
            struct MotorPowers motorPowers = MoveMotorPowers(0, 7, 0, 0);
            spinMotors(motorPowers);
            Serial.println("geri");
            Pid_Flag = 0;
            delay(1);
        }
        while (RemoteXY.right)
        {
            struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 7, 0);
            spinMotors(motorPowers);
            Serial.println("sağ");
            Pid_Flag = 0;
            delay(1);
        }
        while (RemoteXY.left)
        {
            struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 7);
            spinMotors(motorPowers);
            Serial.println("sol");
            Pid_Flag = 0;
            delay(1);
        }
        struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 0);
        spinMotors(motorPowers);
        Pid_Flag = 1;
    }
}