

void CAN_setup(void)
{
	// can3 is the bus we find the steering on, and it will act as one side of the bridge to can2
	can3.begin();
	can3.setBaudRate(250000);
	can3.enableFIFO();
	// low volume, can we handle it all?
	// will have to if it's acting as a bridge
	// can3.setFIFOFilter(REJECT_ALL);
	if (Brand != 0)
	{
		can3.setFIFOFilter(0, 0xCFF0501, EXT); // Claas Curve Data & Valve State Message
		CANBUS_ModuleID = 0x1E;
	}

	// Claim can3 Address - no claiming just now, is this necessary
	if (Brand != 0)
	{
		CAN_message_t msgCAN3;
		msgCAN3.id = 0x18EEFF1E;
		msgCAN3.flags.extended = true;
		msgCAN3.len = 8;
		msgCAN3.buf[0] = 0x00;
		msgCAN3.buf[1] = 0x00;
		msgCAN3.buf[2] = 0xC0;
		msgCAN3.buf[3] = 0x0C;
		msgCAN3.buf[4] = 0x00;
		msgCAN3.buf[5] = 0x17;
		msgCAN3.buf[6] = 0x02;
		msgCAN3.buf[7] = 0x20;
		can3.write(msgCAN3);
	}
	delay(500);

	// can2 - use as one side of the bridge?
	can2.begin();
	can2.setBaudRate(250000);
	can2.enableFIFO();
	// can2.setFIFOFilter(REJECT_ALL); // if bridging, don't filter

	if (Brand != 0)
	{
		// message to claim, not used as Brand > 0
		CAN_message_t msgCAN2;
		msgCAN2.id = 0x18EEFF1E;
		msgCAN2.flags.extended = true;
		msgCAN2.len = 8;
		msgCAN2.buf[0] = 0x00;
		msgCAN2.buf[1] = 0x00;
		msgCAN2.buf[2] = 0xC0;
		msgCAN2.buf[3] = 0x0C;
		msgCAN2.buf[4] = 0x00;
		msgCAN2.buf[5] = 0x17;
		msgCAN2.buf[6] = 0x02;
		msgCAN2.buf[7] = 0x20;
		can2.write(msgCAN2);
	}

	delay(500);

	// can1 - no need - we can bridge between 2 and 3

	//can1.begin();
	//can1.setBaudRate(250000);
	//can1.enableFIFO();
	//// can1.setFIFOFilter(REJECT_ALL); // if bridging, don't filter
	//if (Brand == 1) // not used
	//{
	//	can1.setFIFOFilter(0, 0xCFF2621, EXT); // MF engage button
	//	can1.setFIFOFilter(1, 0x203, EXT);		// MF check valve is on K-bus - not used //
	//}
	//delay(300);

} // End CAN SETUP

//---Send can3 message

void can3Send()
{
	if (!Autosteer_running) {
		return;
	}

	CAN_message_t msgCAN3;
	msgCAN3.id = 0x0CAD131E;
	msgCAN3.flags.extended = true;
	msgCAN3.len = 8;
	msgCAN3.buf[0] = lowByte(setCurve);
	msgCAN3.buf[1] = highByte(setCurve);
	if (Autosteer_running)
		msgCAN3.buf[2] = 253;
	msgCAN3.buf[3] = 0;
	msgCAN3.buf[4] = 0;
	msgCAN3.buf[5] = 0;
	msgCAN3.buf[6] = 0;
	msgCAN3.buf[7] = 0;
	can3.write(msgCAN3);
}

//---Receive can3 message

void can3Receive()
{
	CAN_message_t can3ReceiveData;
	if (can3.read(can3ReceiveData))
	{
		if (can3ReceiveData.id == 0x253 && !Autosteer_running) {
			// nothing to do here, bridge only
			can2.write(can3ReceiveData);
		}
		else if (can3ReceiveData.id == 0xCFF1401) {
			hmsEngaged = can3ReceiveData.buf[0] & 1;
		}
		else if (can3ReceiveData.id == 0x18FF8902) {
			if (can3ReceiveData.buf[4] & 1)
				joystickSteerDirection = JoystickSteerDirection::UpDown;
			else
				joystickSteerDirection = JoystickSteerDirection::LeftRight;
		}
		else if (can3ReceiveData.id == 0xCFF0501)
		{
			estCurve = (can3ReceiveData.buf[0]); // Until clarity on Angle/256 means - decimal perhaps?
			Time = millis();
			digitalWrite(engageLED, HIGH);
			relayTime = ((millis() + 1000));
		}
		if (ShowCANData == 1)
		{
			Serial.print(Time);
			Serial.print(", CAN3");
			Serial.print(", ID: 0x");
			Serial.print(", DATA: ");
			for (uint8_t i = 0; i < 8; i++)
			{
				Serial.print(can3ReceiveData.buf[i], HEX);
				Serial.print(", ");
			}
			if (can3ReceiveData.id == 0xCFF0501) {
				Serial.print(" angle: ");
				Serial.print(estCurve);
			}

		}
	}
} // can3receivedata

//---Receive can2 message
void can2Receive()
{
	CAN_message_t can2ReceiveData;
	if (can2.read(can2ReceiveData))
	{
		Time = millis();
		// read and throw?
		can3.write(can2ReceiveData);
		// acting as one-half the bridge, let's not echo
		//unsigned long PGN;
		//byte priority;
		//byte srcaddr;
		//byte destaddr;
//		j1939_decode(can2ReceiveData.id, &PGN, &priority, &srcaddr, &destaddr);
		//if (ShowCANData == 1)
		//{
		//	Serial.print(Time);
		//	Serial.print(", CAN2");
		//	Serial.print(", MB: ");
		//	Serial.print(can2ReceiveData.mb);
		//	Serial.print(", ID: 0x");
		//	Serial.print(can2ReceiveData.id, HEX);
		//	Serial.print(", PGN: ");
		//	Serial.print(PGN);
		//	Serial.print(", Priority: ");
		//	Serial.print(priority);
		//	Serial.print(", SA: ");
		//	Serial.print(srcaddr);
		//	Serial.print(", DA: ");
		//	Serial.print(destaddr);
		//	Serial.print(", EXT: ");
		//	Serial.print(can2ReceiveData.flags.extended);
		//	Serial.print(", LEN: ");
		//	Serial.print(can2ReceiveData.len);
		//	Serial.print(", DATA: ");
		//	for (uint8_t i = 0; i < 8; i++)
		//	{
		//		Serial.print(can2ReceiveData.buf[i]);
		//		Serial.print(", ");
		//	}
		//	Serial.println("");
		//} // End Show Data
	}
}

////---Receive can1 message
//void can1Receive()
//{
//	CAN_message_t can1ReceiveData;
//	if (can1.read(can1ReceiveData))
//	{
//
//		if (ShowCANData == 1)
//		{
//			Serial.print(Time);
//			Serial.print(", CAN1");
//			Serial.print(", MB: ");
//			Serial.print(can1ReceiveData.mb);
//			Serial.print(", ID: 0x");
//			Serial.print(can1ReceiveData.id, HEX);
//			Serial.print(", EXT: ");
//			Serial.print(can1ReceiveData.flags.extended);
//			Serial.print(", LEN: ");
//			Serial.print(can1ReceiveData.len);
//			Serial.print(", DATA: ");
//			for (uint8_t i = 0; i < 8; i++)
//			{
//				Serial.print(can1ReceiveData.buf[i]);
//				Serial.print(", ");
//			}
//
//			Serial.println("");
//		} // End Show Data
//	}
//}

// AgOpen CAN module
void canConfig()
{

	CAN_message_t config251;
	config251.id = 0x19EF131C;
	config251.flags.extended = true;
	config251.len = 8;
	config251.buf[0] = 251;
	config251.buf[1] = uint8_t(steerSettings.wasOffset);
	config251.buf[2] = uint8_t(steerSettings.wasOffset >> 8);
	uint8_t sett0 = 0;
	if (steerConfig.InvertWAS == 1)
		bitSet(sett0, 0);
	if (steerConfig.IsRelayActiveHigh == 1)
		bitSet(sett0, 1);
	if (steerConfig.MotorDriveDirection == 1)
		bitSet(sett0, 2);
	if (steerConfig.SingleInputWAS == 1)
		bitSet(sett0, 3);
	if (steerConfig.CytronDriver == 1)
		bitSet(sett0, 4);
	if (steerConfig.SteerSwitch == 1)
		bitSet(sett0, 5);
	if (steerConfig.SteerButton == 1)
		bitSet(sett0, 6);
	if (steerConfig.ShaftEncoder == 1)
		bitSet(sett0, 7);
	config251.buf[3] = sett0;
	config251.buf[4] = steerConfig.PulseCountMax;
	uint8_t sett1 = 0;
	if (steerConfig.IsDanfoss == 1)
		bitSet(sett1, 0);
	if (steerConfig.PressureSensor == 1)
		bitSet(sett1, 1);
	if (steerConfig.CurrentSensor == 1)
		bitSet(sett1, 2);
	config251.buf[5] = sett1;
	config251.buf[6] = 0;
	config251.buf[7] = 0;
	can3.write(config251);

	CAN_message_t config252;
	config252.id = 0x19EF131C;
	config252.flags.extended = true;
	config252.len = 8;
	config252.buf[0] = 252;
	config252.buf[1] = uint8_t(steerSettings.Kp);
	config252.buf[2] = uint8_t(steerSettings.highPWM);
	config252.buf[3] = uint8_t(steerSettings.lowPWM);
	config252.buf[4] = uint8_t(steerSettings.minPWM);
	config252.buf[5] = uint8_t(steerSettings.steerSensorCounts);
	config252.buf[6] = uint8_t(steerSettings.AckermanFix * 100);
	config252.buf[7] = 0;
	can3.write(config252);
}
