#define debugCAN 1

void CAN_setup(void)
{
#ifdef debugCAN
	Serial.println("Extra CAN debugging enabled");
#endif
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
	// this would be a steering command (Joy2, in correct orientation) or Joy2 button (to recenter)
	CAN_message_t msgCAN3;
	msgCAN3.id = 0x253;
	msgCAN3.flags.extended = false;
	msgCAN3.len = 8;
	msgCAN3.buf[0] = 0;
	msgCAN3.buf[1] = 0;
	msgCAN3.buf[2] = 0;
	msgCAN3.buf[3] = 0;
	msgCAN3.buf[4] = 0;
	msgCAN3.buf[5] = 0;
	if (joystickSteerDirection == JoystickSteerDirection::LeftRight) {
		if (pwmDrive < 0)
			msgCAN3.buf[6] |= 4; // Joy2Left
		else
			msgCAN3.buf[6] |= 8; // Joy2Right
	}
	else {
		if (pwmDrive < 0)
			msgCAN3.buf[6] |= 2; // Joy2Down
		else
			msgCAN3.buf[6] |= 1; // Joy2Up
	}
	msgCAN3.buf[7] = 0;
	can3.write(msgCAN3);
}

void can3Receive()
{
	CAN_message_t can3ReceiveData;
	if (can3.read(can3ReceiveData))
	{
		if (can3ReceiveData.id == 0x253 && !Autosteer_running) { // we send our own 253s if autosteer is running
			// nothing to do here, bridge only
			if (can3ReceiveData.buf[7] & 4) // but do update this. Not that it'll change once up and running, but we do need to take care of it
				moduleRev = moduleRevision::New;
			else
				moduleRev = moduleRevision::Old;
			can2.write(can3ReceiveData);
		}
		// update some important variables as the info flies by
		else if (can3ReceiveData.id == 0x18FF7C02) {
			flowControl = can3ReceiveData.buf[7];
			flowControlHMS = can3ReceiveData.buf[6];
		}
		else if (can3ReceiveData.id == 0xCFF1401)
			hmsEngaged = can3ReceiveData.buf[0] & 1;
		else if (can3ReceiveData.id == 0x18FF8902) {
			if (can3ReceiveData.buf[4] & 1) {
#ifdef debugCAN
				Serial.println("Joystick orientation is UpDown");
#endif
				joystickSteerDirection = JoystickSteerDirection::UpDown;
			}
			else {
#ifdef debugCAN
				joystickSteerDirection = JoystickSteerDirection::LeftRight;
#endif
				Serial.println("Joystick orientation is LeftRight");
			}
		}
		else if (can3ReceiveData.id == 0xCFF0501)
		{
			estCurve = ((can3ReceiveData.buf[1] << 8) + can3ReceiveData.buf[0]);
			Time = millis();
			digitalWrite(engageLED, HIGH);
			relayTime = ((millis() + 1000));
		}
		if (ShowCANData == 1)
		{
			Serial.print(Time);
			Serial.print(", CAN3, ID: ");
			Serial.print(can3ReceiveData.id, HEX);
			Serial.print(", DATA: ");
			for (uint8_t i = 0; i < 8; i++)
			{
				Serial.print(can3ReceiveData.buf[i], HEX);
				Serial.print(", ");
			}
			if (can3ReceiveData.id == 0x253 && can3ReceiveData.buf[6] <= 8) {
				switch (can3ReceiveData.buf[6]) {
				case 1:
					Serial.print("Joy2 Up,");
					break;
				case 2:
					Serial.print("Joy2 Down,");
					break;
				case 4:
					Serial.print("Joy2 Left,");
					break;
				case 8:
					Serial.print("Joy2 Right,");
					break;
				}
				if (joystickSteerDirection == JoystickSteerDirection::UpDown)
					Serial.print(", SteerUD,");
				else
					Serial.print(", SteerLR,");
			}
			if (can3ReceiveData.id == 0xCFF0501) {
				Serial.print(" angle: ");
				Serial.print(estCurve);
			}
			Serial.println();
		}
	}
}

void can2Receive()
{
	CAN_message_t can2ReceiveData;
	if (can2.read(can2ReceiveData))
	{
		Time = millis();
		// read and forward
		can3.write(can2ReceiveData);
#pragma region Commented
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
#pragma endregion
	}
}
#pragma region Commented
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
#pragma endregion

