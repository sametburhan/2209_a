/*
void PIDcal(void)
{
    if (millis() - tcount > 3)
    {
        yawReq -= (constants[4] - 512) * 0.0006;
        pitchErrNow = pitchReq + ((float(constants[1])) / 1023 * 30) - 15 - (ypr[1] * 180 / M_PI);
        rollErrNow = rollReq + ((float(constants[2])) / 1023 * 30) - 15 - (ypr[2] * 180 / M_PI);

        yawNow = ypr[0] * 180 / M_PI;
        if (yawNow - yawPrev > 280)
        {
            yawReq += 360;
        }
        else if (yawNow - yawPrev < -280)
        {
            yawReq -= 360;
        }
        yawErrNow = yawReq - yawNow;
        yawErrIgt += yawErrNow;
        rollErrIgt += rollErrNow;
        pitchErrIgt += pitchErrNow;

        yaw = (yawKp * yawErrNow) + (yawKd * (yawErrNow - yawErrPrev)) + (yawErrIgt * yawKi);
        pitch = (pitchKp * pitchErrNow) + pitchKd * (-(ypr[1] * 180 / M_PI) + pitchErrPrev) + pitchKi * (pitchErrIgt);
        roll = (rollKp * rollErrNow) + rollKd * (-(ypr[2] * 180 / M_PI) + rollErrPrev) + rollKi * (rollErrIgt);

        yawPrev = yawNow;
        yawErrPrev = yawErrNow;
        pitchErrPrev = (ypr[1] * 180 / M_PI);
        rollErrPrev = (ypr[2] * 180 / M_PI);
        tcount = millis();
    }
}*/