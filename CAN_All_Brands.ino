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

	// From Joystick Side (CAN 2)
	if (fromBus == 2)
	{
		if (id == 0x253)
		{
			if (bitRead(data->bytes[7], 2))			// To do update this. Not that it'll change once up and running, but we do need to take care of it
				moduleRev = moduleRevision::New;
			else if (bitRead(data->bytes[7], 0) && bitRead(data->bytes[7], 1))
				moduleRev = moduleRevision::Old;
		}

		else if (id == 0x254)
		{
			// Check if operator is steering
			if (joystickSteerDirection == JoystickSteerDirection::LeftRight)
			{
				if (bitRead(data->bytes[6], 2) || bitRead(data->bytes[6], 3))
				{
					intendToSteer = 0;
					steeringValveReady = 80;
				}
			}
			else
			{
				if (bitRead(data->bytes[6], 0) || bitRead(data->bytes[6], 1))
				{
					intendToSteer = 0;
					steeringValveReady = 80;
				}
			}

			// Joystick - Modify data if we intend to steer
			if (intendToSteer)
			{
				if (joystickSteerDirection == JoystickSteerDirection::LeftRight)
				{
					if (steerAngleError < -0.1)
					{
						bitClear(data->bytes[6], 2);	// Joy2 Left
						bitSet(data->bytes[6], 3);		// Joy2 Right
					}
					else if (steerAngleError > 0.1)
					{
						bitClear(data->bytes[6], 3);	// Joy2 Right
						bitSet(data->bytes[6], 2);		// Joy2 Left
					}
					else
					{
						bitClear(data->bytes[6], 3);	// Joy2 Right
						bitClear(data->bytes[6], 2);	// Joy2 Left
					}
				}
				else
				{
					if (steerAngleError < -0.1)
					{
						bitClear(data->bytes[6], 0);	// Joy2 Up
						bitSet(data->bytes[6], 1);		// Joy2 Right
					}
					else if (steerAngleError > 0.1)
					{
						bitClear(data->bytes[6], 0);	// Joy2 Up
						bitSet(data->bytes[6], 1);		// Joy2 Down
					}
					else
					{
						bitClear(data->bytes[6], 0);	// Joy2 Up
						bitClear(data->bytes[6], 1);	// Joy2 Down
					}
				}
			}
		}

		else if (id == 0x18FF8002)
		{

		}

		else if (id == 0x18FF7C02)
		{
			flowControl = data->bytes[7];
			flowControlHMS = data->bytes[6];
		}

		else if (id == 0x18FF8902)
		{
			if (bitRead(data->bytes[4], 0)) {
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
	}

	// From Machine ECU Side (CAN 3)
	else if (fromBus == 3)
	{
		if (id == 0xCFF0501)
		{
			// Wheel Angle
			estCurve = ((data->bytes[1] << 8) + data->bytes[0]);
		}

		else if (id == 0xCFF1401)
		{
			// Headland Seq is running on machine ??
			if (bitRead(data->bytes[0], 0))
			{
				hmsEngaged = true;
			}
			else
			{
				hmsEngaged = false;
			}
		}
	}

}
