/******************************************************
 * Samet Burhan
 * Quadcopter 2209-a projesi
 *
 * #include "SoftwareSerial.h"
 * çift seri port kullanılacaksa eklenecek kütüphane
 *
 *
 *
 * Include library
 *****************************************************/
#include "sdkconfig.h"
#include "esp_system.h"
#include "SERVO.cpp"
#include "VOLTAGE.cpp"

/***************************
 * Define and variable
 **************************/
#define RUNNING_CORE_0 0
#define RUNNING_CORE_1 1

/****************************
 * Function decleration
 ***************************/
static void TaskFirst(void *pvParameters);
static void TaskSecond(void *pvParameters);
static void TaskServo(void *pvParameters);
static void TaskPreCollision(void *pvParameters);

void setup()
{
  /********************************************************************************
   * Serial1.begin(115200);
   * Eğer 2 farklı seri port kullanılacaksa konfigüre edilecek
   *******************************************************************************/
  Serial.begin(115200);
  MPU_Init();
  Voltage_Init();

  // MadgwickFilter
  MadgwickFilter.begin(10); // filtering frequency 100Hz
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 100000, true);
  timerAlarmEnable(timer1);
  PWM_Init();
  resetPidVariables();

  /****************************
   * sonar init
   ***************************/
  Sonar_Servo_Init();
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
      NULL, RUNNING_CORE_1);

  xTaskCreatePinnedToCore(
      TaskPreCollision, "TaskCollision",
      1024 // Stack size
      ,
      NULL, 3 // Priority
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
    delay(1);
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
    Move();
    // long int t1 = micros();
    MPU();
    // long int t2;
    // Serial.print("Time taken by the task: ");
    // Serial.print(t1 - t2);
    // Serial.println(" microseconds");
    // t2 = micros();
    delay(1);
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
    if (set_thrust != 0)
    {
      Servo_Degree();
    }
    else
    {
      sonarServo1.write(90);
      sonarServo1.write(90);
    }
    delay(2);
    if (Voltage_Measure(set_thrust))
    {
      throttledown();
    }
  }
}

/****************************
 * Task pre collision
 ***************************/
void TaskPreCollision(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    if (Collision_Flag)
    {
      Pre_Collision();
      Collision_Flag = false;
    }
    delay(1);
  }
}