#include <Arduino.h>

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

double delta_time_in_seconds;
unsigned long last_time = 0;
double getControlSignal(double error, double kp, double ki, double kd, double &pid_i, double &last_error, double delta_time_in_seconds);
struct MotorPowers reduceMotorPowers(MotorPowers motorPowers);
double calculateYawError(double yaw, double DeltaTimeInSeconds);
double fix360degrees(double val);

//----------- PID CONFIGURATION-----------
double KP_roll_pitch = 0.10;
double KI_roll_pitch = 0.050;
double KD_roll_pitch = 0.015;

double KP_yaw = 0.40;
double KI_yaw = 0.50;
double KD_yaw = 0.00;

float PreviousOrientation = 0;
double QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND = 180.00;

volatile uint8_t set_thrust = 0;

double roll_pid_i, pitch_pid_i, yaw_pid_i, roll_last_error, pitch_last_error, yaw_last_error;
double roll_control_signal, pitch_control_signal, yaw_control_signal;

double QUADCOPTER_MAX_TILT_ANGLE = 30.00; // roll, pitch tilt angle limit in degrees
double ROLL_PITCH_CONTROL_SIGNAL_LIMIT = KP_roll_pitch * QUADCOPTER_MAX_TILT_ANGLE * 2;

struct MotorPowers calculateMotorPowers(double roll, double pitch, double yaw)
{
    unsigned long current_time = millis();
    unsigned long delta_time_in_milliseconds = current_time - last_time;
    double delta_time_in_seconds = (double)delta_time_in_milliseconds / 1000.0;
    // calculate orientation errors (error: difference between desired orientation and actual orientation)
    double rollError = -roll;   // receiverCommands.RollAngle
    double pitchError = -pitch; // receiverCommands.PitchAngle
    double yawError = 0;        // calculateYawError(yaw, delta_time_in_seconds); // imu_values);

    // calculate control gains based on errors
    roll_control_signal = getControlSignal(rollError, KP_roll_pitch, KI_roll_pitch, KD_roll_pitch, roll_pid_i, roll_last_error, delta_time_in_seconds);
    pitch_control_signal = getControlSignal(pitchError, KP_roll_pitch, KI_roll_pitch, KD_roll_pitch, pitch_pid_i, pitch_last_error, delta_time_in_seconds);
    yaw_control_signal = getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, yaw_pid_i, yaw_last_error, delta_time_in_seconds);

    last_time = current_time;
    // limit roll-pitch control signals
    roll_control_signal = constrain(roll_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
    pitch_control_signal = constrain(pitch_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);

    // calculate power for each motor
    struct MotorPowers motorPowers;
    motorPowers.frontLeftMotorPower = round(set_thrust + roll_control_signal + pitch_control_signal - yaw_control_signal);
    motorPowers.frontRightMotorPower = round(set_thrust - roll_control_signal + pitch_control_signal + yaw_control_signal);
    motorPowers.rearLeftMotorPower = round(set_thrust + roll_control_signal - pitch_control_signal + yaw_control_signal);
    motorPowers.rearRightMotorPower = round(set_thrust - roll_control_signal - pitch_control_signal - yaw_control_signal);

    motorPowers = reduceMotorPowers(motorPowers);

    return motorPowers;
}

double getControlSignal(double error, double kp, double ki, double kd, double &pid_i, double &last_error, double delta_time_in_seconds)
{
    double pid_p = error;
    double pid_d = (error - last_error) / delta_time_in_seconds;
    pid_i += error * delta_time_in_seconds;

    double control_signal = (kp * pid_p) + (ki * pid_i) + (kd * pid_d);
    last_error = error;
    return control_signal;
}

/**
 * Oppppsiyonel
 */
void resetPidVariables()
{
    roll_pid_i = 0;
    roll_last_error = 0;
    pitch_pid_i = 0;
    pitch_last_error = 0;
}

double calculateYawError(double yaw, double DeltaTimeInSeconds)
{
    double CurrentOrientation = yaw;
    double imuYawAngleChangeInDeltaTime = fix360degrees(CurrentOrientation - PreviousOrientation);
    double imuYawAngleChangePerSecond = imuYawAngleChangeInDeltaTime / DeltaTimeInSeconds;
    double yawError = -imuYawAngleChangePerSecond;
    yawError = constrain(yawError, -QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND, QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND);
    PreviousOrientation = CurrentOrientation;
    return yawError;
}

double fix360degrees(double val)
{
    if (val > 180)
    {
        return val - 360;
    }
    else if (val < -180)
    {
        return val + 360;
    }
    else
    {
        return val;
    }
}

/**
 * Optimizasyon opppsiyonell
 */
struct MotorPowers reduceMotorPowers(MotorPowers motorPowers)
{ // to preserve balance if throttle limit exceeds the max value (180)
    int maxMotorPower = max(max(motorPowers.frontLeftMotorPower, motorPowers.frontRightMotorPower), max(motorPowers.rearLeftMotorPower, motorPowers.rearRightMotorPower));
    if (maxMotorPower > 100)
    {
        double power_reduction_rate = (double)maxMotorPower / (double)100;
        motorPowers.frontLeftMotorPower = round((double)motorPowers.frontLeftMotorPower / power_reduction_rate);
        motorPowers.frontRightMotorPower = round((double)motorPowers.frontRightMotorPower / power_reduction_rate);
        motorPowers.rearLeftMotorPower = round((double)motorPowers.rearLeftMotorPower / power_reduction_rate);
        motorPowers.rearRightMotorPower = round((double)motorPowers.rearRightMotorPower / power_reduction_rate);
    }
    return motorPowers;
}