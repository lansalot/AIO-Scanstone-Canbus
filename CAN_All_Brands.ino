#define debugCAN 1

void CAN_setup(void)
{
#ifdef debugCAN
	Serial.println("Extra CAN debugging enabled");
#endif

	//Teensy FlexCAN_T4 setup
	CAN2_Joystick.begin();
	CAN2_Joystick.setBaudRate(250000);
	CAN2_Joystick.setMaxMB(32);
	CAN2_Joystick.enableFIFO();
	CAN2_Joystick.onReceive(CAN2_Joystick_Handler);
	CAN2_Joystick.enableFIFOInterrupt();

	CAN3_MachineECU.begin();
	CAN3_MachineECU.setBaudRate(250000);
	CAN3_MachineECU.setMaxMB(32);
	CAN3_MachineECU.enableFIFO();
	CAN3_MachineECU.onReceive(CAN3_MachineECU_Handler);
	CAN3_MachineECU.enableFIFOInterrupt();

} // End CAN SETUP

//------------------------------------------------------------------------------------------------------

void CAN2_Joystick_Handler(const CAN_message_t& orig_frame)
{
	//copy frame so we can modify it
	CAN_message_t frame = orig_frame;
	got_frame(frame.id, frame.flags.extended, frame.len, (BytesUnion*)frame.buf, orig_frame.bus);
	frame.seq = 1;
	CAN3_MachineECU.write(frame);
}

//------------------------------------------------------------------------------------------------------

void CAN3_MachineECU_Handler(const CAN_message_t& orig_frame)
{
	//copy frame so we can modify it
	CAN_message_t frame = orig_frame;
	got_frame(frame.id, frame.flags.extended, frame.len, (BytesUnion*)frame.buf, orig_frame.bus);
	frame.seq = 1;
	CAN2_Joystick.write(frame);
}

//------------------------------------------------------------------------------------------------------

void got_frame(uint32_t id, uint8_t extended, uint8_t length, BytesUnion* data, int fromBus)
{
	if (ShowCANData == 1 && fromBus == 2)
	{
		Serial.print(systick_millis_count);
		Serial.print(", CAN2_Joystick");
		Serial.print(", ID: 0x"); Serial.print(id, HEX);
		Serial.print(", LEN: "); Serial.print(length);
		Serial.print(", DATA: \t");
		for (uint8_t i = 0; i < length; i++)
		{
			Serial.print(data->bytes[i], DEC);
			Serial.print(",\t");
		}
		Serial.println();
	}

	else if (ShowCANData == 1 && fromBus == 3)
	{
		Serial.print(systick_millis_count);
		Serial.print(", CAN3_Machine");
		Serial.print(", ID: 0x"); Serial.print(id, HEX);
		Serial.print(", LEN: "); Serial.print(length);
		Serial.print(", DATA: \t");
		for (uint8_t i = 0; i < length; i++)
		{
			Serial.print(data->bytes[i], DEC);
			Serial.print(",\t");
		}
		Serial.println();
	}

	// Read and/or modfiy messages as needed here
	// 
	// From Joystick Side (CAN 2)
	if (fromBus == 2)
	{
		// NOTES TO DO
		// - Check the joystick, if operator moved joystick, steervalve ready = 80, else = 16
		// - If intend to steer mod the joystick wheels data leave everything else unmofified, else leave alone

		// Example
		//	data->bytes[3] = 0x46; 
	}

	// From Machine ECU Side (CAN 3)
	else if (fromBus == 3)
	{
		// NOTES TO DO
		// - Read the wheel angle to estcurve, as is 0 - 65000ish (16bit)
	}

}



/*
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

*/