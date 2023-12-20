#include <Arduino.h>

//----------- PID CONFIGURATION-----------
double KP_roll_pitch = 0.30;
double KI_roll_pitch = 0.10;
double KD_roll_pitch = 0.10;
/*
struct MotorPowers calculateMotorPowers(float roll, float pitch)
{
    // calculate orientation errors (error: difference between desired orientation and actual orientation)
    double rollError = 0.0 - roll;   // receiverCommands.RollAngle
    double pitchError = 0.0 - pitch; // receiverCommands.PitchAngle
    // calculate control gains based on errors
    roll_control_signal = getControlSignal(rollError, KP_roll_pitch, KI_roll_pitch, KD_roll_pitch, roll_pid_i, roll_last_error, imu_values.DeltaTimeInSeconds);
    pitch_control_signal = getControlSignal(pitchError, KP_roll_pitch, KI_roll_pitch, KD_roll_pitch, pitch_pid_i, pitch_last_error, imu_values.DeltaTimeInSeconds);

    // limit roll-pitch control signals
    roll_control_signal = constrain(roll_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
    pitch_control_signal = constrain(pitch_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);

    // calculate power for each motor
    struct MotorPowers motorPowers;
    motorPowers.frontLeftMotorPower = round(set_thrust + roll_control_signal + pitch_control_signal);
    motorPowers.frontRightMotorPower = round(set_thrust - roll_control_signal + pitch_control_signal);
    motorPowers.rearLeftMotorPower = round(set_thrust + roll_control_signal - pitch_control_signal);
    motorPowers.rearRightMotorPower = round(set_thrust - roll_control_signal - pitch_control_signal);

    motorPowers = reduceMotorPowers(motorPowers);

    return motorPowers;
}
*/
double getControlSignal(double error, double kp, double ki, double kd, double &pid_i, double &last_error, double delta_time_in_seconds)
{
    double pid_p = error;
    double pid_d = (error - last_error) / delta_time_in_seconds;
    pid_i += error * delta_time_in_seconds;

    double control_signal = (kp * pid_p) + (ki * pid_i) + (kd * pid_d);
    last_error = error;
    return control_signal;
}