#include "MOVE.cpp"

Servo sonarServo1;
Servo sonarServo2;

struct DetectObject
{
    uint8_t up_down; // 0 üst 1 alt
    int degree;
};
struct DetectObject object;

/****************************
 * Variables
 ***************************/
volatile bool Collision_Flag = false;

volatile long duration1, duration2;
volatile uint8_t distance1, distance2;

/****************************
 * Sonar Pinler
 ***************************/
// üst sonar
#define trigPin1 14
#define echoPin1 27
#define up_sonar 0
// alt sonar
#define trigPin2 25
#define echoPin2 26
#define down_sonar 1

#define all_sonar 2

// 0 üst 1 alt
#define object_up 0u
#define object_down 1u

#define servo_pin1 12
#define servo_pin2 13

void Sonar_Servo_Init();
bool Sonar_Detect(int degree);
void Pre_Collision();
void Servo_Degree();
void stopMotors();
void upOrDown(uint8_t upOrDown);

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
                Collision_Flag = true;
            }
        }
    }
    delay(2);
    for (degree = 180; degree > -180; degree--)
    {
        sonarServo1.write(degree);
        sonarServo2.write(degree);
        delay(2);
        if (degree % 20 == 0)
        {
            if (Sonar_Detect(degree) == true)
            {
                Collision_Flag = true;
            }
        }
    }
}

void Sonar_Servo_Init()
{
    sonarServo1.setPeriodHertz(50); // Standard 100hz
    sonarServo2.setPeriodHertz(50); // Standard 100hz

    sonarServo1.attach(servo_pin1, minUs, maxUs);
    sonarServo2.attach(servo_pin2, minUs, maxUs);
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
        // engel yukarıdan geldi
        object.up_down = up_sonar;
        return true;
    }
    if (distance2 < 12)
    {
        object.degree = -1 * degree;
        if (object.up_down == up_sonar)
        {
            // engel hem üstten hem alttan geldi
            object.up_down == all_sonar;
        }
        else
        {
            // engel aşağıdan geldi
            object.up_down = down_sonar;
        }
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
 * Karşıdan gelirse geri git
 *******************************************************************************/
void Pre_Collision()
{
    /****************************
     * Sağ veya soldan gelen engel
     ***************************/
    if (object.degree > 90 && object.up_down != all_sonar)
    {
        upOrDown(object.up_down);
        // sağa git
        struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 7, 0);
        spinMotors(motorPowers);
    }
    else if (object.degree < 90 && object.up_down != all_sonar)
    {

        upOrDown(object.up_down);
        // sola git
        struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 7);
        spinMotors(motorPowers);
    }
    /****************************
     * Karşıdan gelen engel
     ***************************/
    else
    {
        // doğrudan geri gel
        struct MotorPowers motorPowers = MoveMotorPowers(0, 7, 0, 0);
        spinMotors(motorPowers);
    }

    delay(200); // todo: 200ms çok olabilir
    struct MotorPowers motorPowers = MoveMotorPowers(0, 0, 0, 0);
    spinMotors(motorPowers);
    Thrust();
    Serial.println("kaçtık");
}

void upOrDown(uint8_t upOrDown)
{
    if (upOrDown == object_up)
    {
        /**
         * engel yukarıdan geldi biraz alçal "thrust =- 5"
         */
        set_thrust = -5;
        frontLeftMotorPower.write(set_thrust);
        frontRightMotorPower.write(set_thrust);
        rearLeftMotorPower.write(set_thrust);
        rearRightMotorPower.write(set_thrust);
    }
    else if (!upOrDown == object_down)
    {
        /**
         * engel aşağıdan geldi biraz yüksel "thrust =+ 5"
         */
        set_thrust = +5;
        frontLeftMotorPower.write(set_thrust);
        frontRightMotorPower.write(set_thrust);
        rearLeftMotorPower.write(set_thrust);
        rearRightMotorPower.write(set_thrust);
    }
}