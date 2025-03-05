
// Danfoss PVED-CL Service Tool for use with AgOpenGPS
// The Danfoss PVED-CL parts are not in here yet, but will be copied over soon
void Service_Tool (void) 
{
  Serial.println("\r\nScanStone CANBUS Service Tool Mode:");
  Help();
  
  while (Service == 1) 
  {
      if (Serial.available())   // Read Data From Serail Monitor 
      {    
        byte b = Serial.read();
        if ( b == '?') Help();          
        else if ( b == 'X') Service = 0; //Exit Service Mode
        else if ( b == '0') ScanStone();
        else if ( b == 'R') ReadCAN();
        else if ( b == 'S') StopCAN();
        else if ( b == 'f') gpsModeOne();
        else if ( b == 'F') gpsModeTwo();
        else if ( b == 'p') gpsModeThree();
        else if ( b == 'P') gpsModeFour();

        else
        {
          Serial.println("No command, send ? for help");
          Serial.println(" ");
          delay(50);
        }

        while (Serial.available())
        {
        Serial.read();                //Clear the serial buffer
        }
      }

      if (tempChecker > 10000)
      {
          tempChecker = 0;
          float temp = tempmonGetTemp();
          Serial.print(temp, 2);
          Serial.println(" degC CPU Temp");
      }
  }
}

//**************************************************************************************
void Help()
{
  Serial.println("? = Help");
  Serial.println("X = Exit Service Mode");
  Serial.println("0 = Set Brand as ScanStone");
  Serial.println("R = Show CAN Data");
  Serial.println("S = Stop Data");
  Serial.println("    **GPS options**");
  Serial.println("Forwarding Mode: f = 115200, F = 460800");
  Serial.println("Panda Mode: p = 115200, P = 460800\r\n");
}

//**************************************************************************************
void ScanStone()
{
  EEPROM.update(70,0); 
  Brand = EEPROM.read(70);
  Serial.println("Brand Set ScanStone, Restarting Teensy");
  delay(1000);
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
  Serial.println(" ");
}
//**************************************************************************************
void ReadCAN()
{
ShowCANData = 1;
  Serial.println("CAN Data ON, Send X To Exit Service Tool");
  Serial.println(" ");
}
//**************************************************************************************
void StopCAN()
{
ShowCANData = 0;
  Serial.println("CAN Data OFF, Send X To Exit Service Tool");
  Serial.println(" ");
}
//**************************************************************************************
void gpsModeOne()
{
    EEPROM.update(72, 1);
    Serial.println("GPS Forwarding @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeTwo()
{
    EEPROM.update(72, 2);
    Serial.println("GPS Forwarding @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeThree()
{
    EEPROM.update(72, 3);
    Serial.println("GPS Panda @ 115200, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
//**************************************************************************************
void gpsModeFour()
{
    EEPROM.update(72, 4);
    Serial.println("GPS Panda @ 460800, Restarting Teensy");
    delay(1000);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
    Serial.println(" ");
}
