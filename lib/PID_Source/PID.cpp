#include <Arduino.h>
#include <ESP32Servo.h>

double error_sum_pitch = 0;
double error_sum_roll = 0;
unsigned long last_time = 0.0;
double last_error_pitch = 0;
double last_error_roll = 0;

void motor_control_pid(int set_thrust, Servo mot1, Servo mot2, Servo mot3, Servo mot4, double error_calc_pitch, double error_calc_roll)
{
    int calc_thrust_pitch = set_thrust;
    int calc_thrust_roll = set_thrust;

    calc_thrust_pitch = calc_thrust_pitch * abs(error_calc_pitch);
    // calc_thrust_pitch = map(calc_thrust_pitch, 0, 10000, 0, 200);

    calc_thrust_roll = calc_thrust_roll * abs(error_calc_roll);
    // calc_thrust_roll = map(calc_thrust_roll, 0, 10000, 0, 200);
    if (error_calc_pitch < -0.3)
    {
        mot1.write(calc_thrust_pitch);
        mot2.write(calc_thrust_pitch);
    }
    else if (error_calc_pitch > 0.3)
    {
        mot3.write(calc_thrust_pitch);
        mot4.write(calc_thrust_pitch);
    }
    Serial.print("calc thrust pitch = ");
    Serial.println(calc_thrust_pitch);

    if (error_calc_roll < -0.3)
    {
        mot1.write(calc_thrust_roll);
        mot3.write(calc_thrust_roll);
    }
    else if (error_calc_roll > 0.3)
    {
        mot2.write(calc_thrust_roll);
        mot4.write(calc_thrust_roll);
    }
    Serial.print("calc thrust roll = ");
    Serial.println(calc_thrust_roll);
}

void pid_calculate(double kp, double ki, double kd, double input_pitch, double input_roll, double setpoint, double output_pitch, double output_roll)
{

    unsigned long now = millis();
    double time_change = (double)(now - last_time);

    /*
    ###########    PITCH    #############
    */
    double error_pitch = (double)(input_pitch - setpoint); // error

    error_sum_pitch += error_pitch * time_change; // integral error

    double diff_error_pitch = (error_pitch - last_error_pitch) / time_change; // diferantial error

    output_pitch = kp * error_sum_pitch + ki * error_sum_pitch + kd * diff_error_pitch;

    last_error_pitch = error_pitch;

    /*
    ###########    ROLL    #############
    */
    double error_roll = (double)(input_roll - setpoint); // error

    error_sum_roll += error_roll * time_change; // integral error

    double diff_error_roll = (error_roll - last_error_roll) / time_change; // diferantial error

    output_roll = kp * error_sum_roll + ki * error_sum_roll + kd * diff_error_roll;

    last_error_roll = error_roll;

    last_time = now;
}
/*
    pitch -> 12 , -12
    roll -> 12 , -12
*/
void full_pid(int set_thrust, Servo mot1, Servo mot2, Servo mot3, Servo mot4, double kp, double ki, double kd, double input_pitch, double input_roll, double setpoint)
{
    double output_pitch = 0.0;
    double output_roll = 0.0;
    pid_calculate(kp, ki, kd, input_pitch, input_roll, setpoint, output_pitch, output_roll);
    delay(10);
    motor_control_pid(set_thrust, mot1, mot2, mot3, mot4, output_pitch, output_roll);
    delay(10);
}
