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

#define servo_pin1 12
#define servo_pin2 13

// üst sonar
#define trigPin1 14
#define echoPin1 27
#define up_sonar 0
// alt sonar
#define trigPin2 25
#define echoPin2 26
#define down_sonar 1

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
volatile float roll, pitch;
const int minUs = 1000;
const int maxUs = 3000;

volatile long duration1, duration2;
volatile uint8_t distance1, distance2;

/****************************
 * Function decleration
 ***************************/
static void TaskFirst(void *pvParameters);
static void TaskSecond(void *pvParameters);
static void TaskServo(void *pvParameters);
static void PWM_Init();
static void Thrust();
static void Move();
static void MPU_Init();
static void MPU_Motion();
static void Idle(float roll, float pitch);
static void spinMotors(struct MotorPowers motorPowers);
static struct MotorPowers MoveMotorPowers(uint8_t up, uint8_t down, uint8_t right, uint8_t left);
static void stopMotors();
static void Sonar_Servo_Init();
static bool Sonar_Detect(int degree);
static void Pre_Collision();
static void Servo_Degree();
static void upOrDown(bool upOrDown);

struct DetectObject
{
  bool up_down; // 0 üst 1 alt
  int degree;
};
struct DetectObject object;

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
  sonarServo1.setPeriodHertz(50); // Standard 100hz
  sonarServo2.setPeriodHertz(50); // Standard 100hz

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
  int degree;
  for (degree = -180; degree < 180; degree++)
  {
    sonarServo1.write(degree);
    sonarServo2.write(degree);
    delay(2);
    if (degree % 20 == 0)
    {
      if (Sonar_Detect(degree) == true)
      {
        Pre_Collision();
      }
    }
  }
  delay(7);
  for (degree = 180; degree > -180; degree--)
  {
    sonarServo1.write(degree);
    sonarServo2.write(degree);
    delay(2);
    if (degree % 20 == 0)
    {
      if (Sonar_Detect(degree) == true)
      {
        Pre_Collision();
      }
    }
  }
  delay(7);
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
struct MotorPowers MoveMotorPowers(uint8_t up, uint8_t down, uint8_t right, uint8_t left)
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
      struct MotorPowers motorPowers = MoveMotorPowers(7, 0, 0, 0);
      spinMotors(motorPowers);
      Serial.println("ileri");
      delay(1);
    }
    while (RemoteXY.down)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 7, 0, 0);
      spinMotors(motorPowers);
      Serial.println("geri");
      delay(1);
    }
    while (RemoteXY.right)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 7, 0);
      spinMotors(motorPowers);
      Serial.println("sağ");
      delay(1);
    }
    while (RemoteXY.left)
    {
      struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 7);
      spinMotors(motorPowers);
      Serial.println("sol");
      delay(1);
    }
    struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 0);
    spinMotors(motorPowers);
  }
}

/****************************
 * Engel tespit ve kaçış fonksiyonları
 ***************************/
bool Sonar_Detect(int degree)
{
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;

  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;
  if (distance1 < 12)
  {
    object.degree = degree;
    object.up_down = up_sonar;
    return true;
  }
  else if (distance2 < 12)
  {
    object.degree = -1 * degree;
    object.up_down = down_sonar;
    return true;
  }
  else
  {
    return false;
  }
}

/********************************************************************************
 * Engel aşağıdan gelirse hafif yükselerek geri manevra yap
 * Yukarıdan gelirse hafif alçalarak geri manevra yap
 * Engel sağdan gelirse hafif sola git
 * Soldan gelirse hafif sağa git
 *******************************************************************************/
void Pre_Collision()
{
  // sağ veya soldan gelen engel
  if (object.degree > 90)
  {
    upOrDown(object.up_down);
  }
  else if (object.degree < 90)
  {
    upOrDown(object.up_down);
  }
  else
  {
    upOrDown(object.up_down);
    // dik gelen engel
  }
  Serial.println("kaçtık");
}

void upOrDown(bool upOrDown)
{
  if (upOrDown)
  {
  }
  else if (!upOrDown)
  {
  }
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
  /****************************
   * sonar init
   ***************************/
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT);  // Sets the echoPin as an Input

  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT);  // Sets the echoPin as an Input
  /****************************
   * sonar
   ***************************/
  RemoteXY_Init();
  while (RemoteXY.connect_flag)
  {
    Serial.println("BAğlantı sağlanmadı...");
    RemoteXY_Handler();
    delay(50);
  }
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

  xTaskCreatePinnedToCore(
      TaskServo, "TaskServo",
      1024 // Stack size
      ,
      NULL, 1 // Priority
      ,
      NULL, RUNNING_CORE_0);
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

/****************************
 * Task servo
 ***************************/
void TaskServo(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    Servo_Degree();
  }
}