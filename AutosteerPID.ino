void calcSteeringPID(void)
{
	//Proportional only
	pValue = steerSettings.Kp * steerAngleError;
	pwmDrive = (int16_t)pValue;

	errorAbs = abs(steerAngleError);
	int16_t newMax = 0;

	if (errorAbs < LOW_HIGH_DEGREES)
		newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
	else
		newMax = steerSettings.highPWM;

	//add min throttle factor so no delay from motor resistance.
	if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
	else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

	//Serial.print(newMax); //The actual steering angle in degrees
	//Serial.print(",");

	//limit the pwm drive
	if (pwmDrive > newMax) pwmDrive = newMax;
	if (pwmDrive < -newMax) pwmDrive = -newMax;

	if (steerConfig.MotorDriveDirection) pwmDrive *= -1;
}
