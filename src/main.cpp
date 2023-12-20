/******************************************************
 * Samet Burhan
 * Quadcopter 2209-a projesi
 *
 * #include "SoftwareSerial.h"
 * çift seri port kullanılacaksa eklenecek kütüphane
 *
 * Include library
 *****************************************************/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <ESP32Servo.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "PID.cpp"

/***************************
 * Define and variable
 **************************/
#define RUNNING_CORE_0 0
#define RUNNING_CORE_1 1

#define motor_solon_1 5
#define motor_sagon_2 23
#define motor_solarka_3 19
#define motor_sagarka_4 18

#define servo_pin1 14
#define servo_pin2 27

#define KP 0.30
#define KI 0.10
#define KD 0.10

ESP32PWM pwm;
Servo frontLeftMotorPower;
Servo frontRightMotorPower;
Servo rearLeftMotorPower;
Servo rearRightMotorPower;
Servo sonarServo1;
Servo sonarServo2;

Adafruit_MPU6050 mpu;

/****************************
 * Variables
 ***************************/
volatile uint8_t set_thrust = 0;
volatile float roll, pitch;
const int minUs = 1000;
const int maxUs = 2000;

/****************************
 * Function decleration
 ***************************/
static void TaskFirst(void *pvParameters);
static void TaskSecond(void *pvParameters);
static void PWM_Init();
static void Thrust();
static void Move();
static void MPU_Init();
static void MPU_Motion();
static void Idle(float roll, float pitch);
void spinMotors(struct MotorPowers motorPowers);
void stopMotors();
void Sonar_Servo_Init();
bool Sonar_Detect();
void Pre_Collision();
void Servo_Degree();

/****************************
 * Struct
 ***************************/
struct MotorPowers
{
  int frontLeftMotorPower;
  int frontRightMotorPower;
  int rearLeftMotorPower;
  int rearRightMotorPower;
};

/////////////////////////////////////////////
//           Wifi RemoteXY Include         //
/////////////////////////////////////////////

#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "ESP_Drone"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 70 bytes
    {255, 5, 0, 0, 0, 63, 0, 16, 31, 1, 4, 0, 50, 24, 8, 51, 119, 26, 1, 0,
     4, 45, 12, 12, 2, 31, 115, 111, 108, 0, 1, 0, 18, 31, 12, 12, 119, 31, 105, 108,
     101, 114, 105, 0, 1, 0, 32, 45, 12, 12, 2, 31, 115, 97, 196, 159, 0, 1, 0, 18,
     60, 12, 12, 119, 31, 103, 101, 114, 105, 0};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  int8_t thrust; // =0..100 slider position
  uint8_t left;  // =1 if button pressed, else =0
  uint8_t up;    // =1 if button pressed, else =0
  uint8_t right; // =1 if button pressed, else =0
  uint8_t down;  // =1 if button pressed, else =0

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

/****************************
 * Başlangıç fonksiyonları
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

void Sonar_Servo_Init()
{
  sonarServo1.setPeriodHertz(100); // Standard 100hz
  sonarServo2.setPeriodHertz(100); // Standard 100hz

  sonarServo1.attach(servo_pin1, minUs, maxUs);
  sonarServo2.attach(servo_pin2, minUs, maxUs);
}

void MPU_Init()
{

  if (!mpu.begin())
  {
    while (1)
    {
      Serial.println("MPU 6050 bulunamadı");
      delay(100);
    }
  }

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  Serial.println("MPU 6050 kullanıma hazır");
}

/****************************
 * Servo fonksiyonları
 ***************************/
void Servo_Degree()
{
  uint8_t degree = 0;
  for (degree = 0; degree < 180; degree++)
  {
    sonarServo1.write(degree);
    sonarServo2.write(degree);
    if (Sonar_Detect() == true)
    {
      Pre_Collision();
      delay(7);
    }
  }
  for (degree = 180; degree > -180; degree--)
  {
    sonarServo1.write(degree);
    sonarServo2.write(degree);
    if (Sonar_Detect() == true)
    {
      Pre_Collision();
      delay(7);
    }
  }
}
/****************************
 * Hareket fonksiyonları
 ***************************/
void spinMotors(struct MotorPowers motorPowers)
{
  frontLeftMotorPower.write(motorPowers.frontLeftMotorPower);
  frontRightMotorPower.write(motorPowers.frontRightMotorPower);
  rearLeftMotorPower.write(motorPowers.rearLeftMotorPower);
  rearRightMotorPower.write(motorPowers.rearRightMotorPower);
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
    Serial.println(set_thrust);
    frontLeftMotorPower.write(set_thrust);
    frontRightMotorPower.write(set_thrust);
    rearLeftMotorPower.write(set_thrust);
    rearRightMotorPower.write(set_thrust);
  }
}

void MPU_Motion()
{
  if (mpu.getMotionInterruptStatus())
  {
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Roll - Pitch
    roll = a.acceleration.roll;
    pitch = a.acceleration.pitch;
    //---------------------------
    Serial.print("roll =");
    Serial.println(roll);
    Serial.print("pitch =");
    Serial.println(pitch);
    //---------------------------
    Idle(roll, pitch);
  }
}

void Idle(float roll, float pitch)
{
  struct MotorPowers motorPowers = calculateMotorPowers(roll, pitch);
  spinMotors(motorPowers);
}

/****************************
 * Manevra fonksiyonları
 ***************************/
struct MotorPowers MoveMotorPowers(int up, int down, int right, int left)
{
  struct MotorPowers motorPowers;
  motorPowers.frontLeftMotorPower = set_thrust + down + right;
  motorPowers.frontRightMotorPower = set_thrust + down + left;
  motorPowers.rearLeftMotorPower = set_thrust + up + right;
  motorPowers.rearRightMotorPower = set_thrust + up + left;
  return motorPowers;
}

void Move()
{
  if (RemoteXY.up || RemoteXY.down || RemoteXY.left || RemoteXY.right)
  {
    while (RemoteXY.up)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(5, 0, 0, 0);
      spinMotors(motorPowers);
      delay(1);
    }
    while (RemoteXY.down)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 5, 0, 0);
      spinMotors(motorPowers);
      delay(1);
    }
    while (RemoteXY.right)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 5, 0);
      spinMotors(motorPowers);
      // şimdilik boş
      delay(1);
    }
    while (RemoteXY.left)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 5);
      spinMotors(motorPowers);
      // şimdilik boş
      delay(1);
    }
    struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 0);
    spinMotors(motorPowers);
  }
}

/****************************
 * Engel tespit ve kaçış fonksiyonları
 ***************************/
bool Sonar_Detect()
{
  return false;
}

void Pre_Collision()
{
}

/****************************
 * Setup
 ***************************/
void setup()
{
  Serial.begin(115200);
  /********************************************************************************
   * Serial1.begin(115200);
   * Eğer 2 farklı seri port kullanılacaksa konfigüre edilecek
   *******************************************************************************/

  MPU_Init();
  PWM_Init();
  Sonar_Servo_Init();
  RemoteXY_Init();

  xTaskCreatePinnedToCore(
      TaskFirst, "TaskFirst" // Name
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 2 // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL, RUNNING_CORE_0);

  xTaskCreatePinnedToCore(
      TaskSecond, "TaskSecond" // Name
      ,
      4096 // Stack size
      ,
      NULL, 2 // Priority
      ,
      NULL, RUNNING_CORE_1);
}

void loop()
{

  // FreeRTOS
}

/****************************
 * Task 1
 ***************************/
void TaskFirst(void *pvParameters) //
{
  (void)pvParameters;
  for (;;)
  {
    RemoteXY_Handler();
    delay(7);
    Servo_Degree();
  }
}

/****************************
 * Task 2
 ***************************/
void TaskSecond(void *pvParameters) //
{
  (void)pvParameters;
  for (;;)
  {
    Thrust();
    delay(7);
    Move();
    delay(7);
    MPU_Motion();
    delay(7);
  }
}