#define isAllInOneBoard
//Tool Steer
bool useToolSteer = 1;

float toolXTE = 0;
float tractorXTE = 0;
float toolSteerAngleSetPoint = 0;

int8_t integralGain = 5;
float integral = 0;
float pivotDistanceError = 0;
float pivotDistanceErrorLast = 0;
int16_t integralCounter = 0;
float pivotDerivative = 0;
bool Autosteer_running = true; //Auto set off in autosteer setup

//Scanstone
bool hmsEngaged = false;

enum JoystickSteerDirection
{
	UpDown = 0,
	LeftRight = 1
};
JoystickSteerDirection joystickSteerDirection = JoystickSteerDirection::LeftRight;

enum moduleRevision
{
	New = 0,
	Old = 1
};
moduleRevision moduleRev = moduleRevision::New;

int flowControl = 0;
int flowControlHMS = 0;



String inoVersion = ("\r\nAgOpenGPS Scanstone ToolSteer 05.03.2025\r\n");

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

#ifdef isAllInOneBoard
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37
#else
#define STEERSW_PIN 6 //PD6
#define WORKSW_PIN 7  //PD7
#define REMOTE_PIN 8  //PB0
#endif

 /////////////////////////////////////////////

 // if not in eeprom, overwrite 
#define EEP_Ident 0x5455

//--------------------------- Switch Input Pins ------------------------
#ifdef isAllInOneBoard
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37
#else
#define STEERSW_PIN 6 //PD6
#define WORKSW_PIN 7  //PD7
#define REMOTE_PIN 8  //PB0
#endif

#define CONST_180_DIVIDED_BY_PI 57.2957795130823
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

#include <Wire.h>
#include <EEPROM.h> 
#include "zNMEAParser.h"
#include "BNO08x_AOG.h"

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype
extern float tempmonGetTemp(void);
elapsedMillis tempChecker;



//----Teensy 4.1 Ethernet--Start---------------------
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
	uint8_t ipOne = 192;
	uint8_t ipTwo = 168;
	uint8_t ipThree = 1;
};  ConfigIP networkAddress;   //3 bytes

// Module IP Address / Port
IPAddress ip = { 0,0,0,126 };
unsigned int localPort = 8888;
unsigned int NtripPort = 2233;

// AOG IP Address / Port
static uint8_t ipDestination[] = { 0,0,0,255 };
unsigned int AOGPort = 9999;

//MAC address
byte mac[] = { 0x00,0x00,0x56,0x00,0x00,0x7E };

enum PGNs {
	SteerData = 0xFE,
	AgIOHello = 0xC8,
	Destoner = 0xE9,
	MachineData = 0xEF,
	MachineSettings = 0xEE,
	SteerConfig = 0xFB,
	SteerSettings = 0xFC,
	SubnetRequest = 0xCA,
	SubnetChange = 0xC9
};

// Buffer For Receiving UDP Data
union _udpPacket {
	byte udpData[512];    // Incoming Buffer
	struct {
		uint16_t AOGID;
		byte MajorPGN;
		byte MinorPGN;
		byte data[508];
	};
};;

_udpPacket udpPacket;

byte NtripData[512];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetUDP NtripUdp;

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

#include <FlexCAN_T4.h>
#include "canframe.h"
FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> CAN1_Tractor_ISOBUS;	//For another day
FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN2_Joystick;
FlexCAN_T4<CAN3, RX_SIZE_1024, TX_SIZE_1024> CAN3_MachineECU;

#ifdef isAllInOneBoard
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = No RTK)
#define GPSGREEN_LED 10           //Green (ON = RTK)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green
#else
#define ledPin 5        //Option for LED, CAN Valve Ready To Steer.
#define engageLED 24    //Option for LED, to see if Engage message is recived.
#define steeringLED 9    //Option for LED, to see if Engage message is recived.
#endif

uint8_t gpsMode = 4;
uint8_t Brand = 0;              //Variable to set brand via serial monitor.
uint8_t CANBUS_ModuleID = 0x1C; //Used for the Module CAN ID

uint32_t Time;                  //Time Arduino has been running
uint32_t relayTime;             //Time to keep "Button Pressed" from CAN Message
boolean engageCAN = 0;          //Variable for Engage from CAN
boolean workCAN = 0;            //Variable for Workswitch from CAN
boolean Service = 0;            //Variable for Danfoss Service Tool Mode
boolean ShowCANData = 1;        //Variable for Showing CAN Data

uint16_t setCurve = 32128;       //Variable for Set Curve to CAN
uint16_t estCurve = 32128;       //Variable for WAS from CAN

//WAS Calabration
float inputWAS[] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };  //Input WAS do not adjust
float outputWAS[] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };
float outputWASFendt[] = { -60.00, -54.0, -48.0, -42.3, -36.1, -30.1, -23.4, -17.1, -11.0, -5.5, 0, 5.5, 11.0, 17.1, 23.4, 30.1, 36.1, 42.3, 48.0, 54.0, 60.0 };  //Fendt 720 SCR, CPD = 80

boolean sendCAN = 0;              //Send CAN message every 2nd cycle (If needed ?)
uint8_t steeringValveReady = 0;   //Variable for Steering Valve State from CAN
boolean intendToSteer = 0;        //Do We Intend to Steer?

//----Teensy 4.1 CANBus--End-----------------------

  //Main loop time variables in microseconds  
const uint16_t LOOP_TIME = 40;	//25hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//IMU data                            
const uint16_t GYRO_LOOP_TIME = 20;   //50Hz IMU 
uint32_t lastGyroTime = GYRO_LOOP_TIME;
uint32_t IMU_currentTime;

bool blink;

//IMU data
float roll = 0;
float pitch = 0;
float yaw = 0;

//Roomba Vac mode for BNO085 and data
#include "BNO_RVC.h"
BNO_rvc rvc = BNO_rvc();
BNO_rvcData bnoData;
elapsedMillis bnoTimer;
bool bnoTrigger = false;
HardwareSerial* SerialIMU = &Serial5;   //IMU BNO-085

// booleans to see what mode BNO08x
bool useBNO08x = false;
bool useBNO08xRVC = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A,0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

//show life in AgIO - v5.5
uint8_t helloAgIO[] = { 0x80,0x81, 0x7f, 0xC7, 1, 0, 0x47 };
uint8_t helloCounter = 0;

//Heart beat hello AgIO - v5.6
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t AOG[] = { 0x80,0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int16_t AOGSize = sizeof(AOG);

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 0x7f, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
uint8_t pressureReading;
uint8_t currentReading;

//EEPROM
int16_t EEread = 0;

//Relays
uint8_t relay = 0, relayHi = 0, tram = 0, hydLift = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t previousStatus = 0;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings  
struct Storage
{
	uint8_t Kp = 15;  //proportional gain
	uint8_t lowPWM = 5;  //band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 1;
	uint8_t highPWM = 250;//max PWM value
	float steerSensorCounts = 80;
	float AckermanFix = 1;     //sent as percent
};  Storage steerSettings;  //11 bytes

//Variables for settings - 0 is false  
struct Setup
{
	uint8_t InvertWAS = 0;
	uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
	uint8_t MotorDriveDirection = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;
	uint8_t SteerSwitch = 0;  //1 if switch selected
	uint8_t SteerButton = 0;  //1 if button selected
	uint8_t ShaftEncoder = 0;
	uint8_t PressureSensor = 0;
	uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;
	uint8_t IsUseY_Axis = 1;     //0 = use X Axis, 1 = use Y axis
};  Setup steerConfig;          //9 bytes

//Variables for config - 0 is false - Machine Config
struct Config
{
	uint8_t raiseTime = 2;
	uint8_t lowerTime = 4;
	uint8_t enableToolLift = 0;
	uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

};  Config aogConfig;   //4 bytes

//*******************************************************************************

void setup()
{
	Serial.println(inoVersion);
#ifdef isAllInOneBoard
	Serial.println("All In One Board");
#else
	Serial.println("CANBUS board");
#endif
	delay(500);                         //Small delay so serial can monitor start up

	set_arm_clock(600000000);           //Set CPU speed to 450mhz
	Serial.print("CPU speed set to: ");
	Serial.println(F_CPU_ACTUAL);

	//keep pulled high and drag low to activate, noise free safe   

#ifdef isAllInOneBoard
	pinMode(Power_on_LED, OUTPUT);
	digitalWrite(Power_on_LED, HIGH);
	pinMode(Ethernet_Active_LED, OUTPUT);
	pinMode(GPSRED_LED, OUTPUT);
	pinMode(GPSGREEN_LED, OUTPUT);
	pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
	pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);
#endif 
	pinMode(WORKSW_PIN, INPUT_PULLUP);
	pinMode(STEERSW_PIN, INPUT_PULLUP);
	pinMode(REMOTE_PIN, INPUT_PULLUP);
	pinMode(13, OUTPUT);


	//set up communication
	Wire.begin();
	Serial.begin(460800);

	delay(2000);


	SerialIMU->begin(115200);
	rvc.begin(SerialIMU);

	// Check for i2c BNO08x
	uint8_t error;

	static elapsedMillis rvcBnoTimer = 0;
	Serial.println("\r\nChecking for serial BNO08x");
	while (rvcBnoTimer < 1000)
	{
		//check if new bnoData
		if (rvc.read(&bnoData))
		{
			useBNO08xRVC = true;
			Serial.println("Serial BNO08x Good To Go :-)");
			imuHandler();
			break;
		}
	}
	if (!useBNO08xRVC)  Serial.println("No Serial BNO08x not Connected or Found");

	EEPROM.get(0, EEread);     // read identifier

	if (EEread != EEP_Ident)   // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(6, aogConfig); //Machine
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(60, networkAddress);
		EEPROM.update(70, Brand);
		EEPROM.update(72, gpsMode);
		//EEPROM.put(80, outputWAS);
	}
	else
	{
		EEPROM.get(6, aogConfig); //Machine
		EEPROM.get(10, steerSettings);     // read the Settings
		EEPROM.get(40, steerConfig);
		EEPROM.get(60, networkAddress);
		//EEPROM.get(80, outputWAS);
		Brand = 0; // EEPROM.read(70);
		gpsMode = 4; // EEPROM.read(72);
	}

	// for PWM High to Low interpolator
	highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;

	//----Teensy 4.1 Ethernet--Start---------------------

	delay(500);

	if (Ethernet.linkStatus() == LinkOFF)
	{
		Serial.println("\r\nEthernet cable is not connected - Who cares we will start ethernet anyway.");
	}

	Ethernet.begin(mac, 0);          // Start Ethernet with IP 0.0.0.0

	//grab the ip from EEPROM
	ip[0] = networkAddress.ipOne;
	ip[1] = networkAddress.ipTwo;
	ip[2] = networkAddress.ipThree;

	ipDestination[0] = networkAddress.ipOne;
	ipDestination[1] = networkAddress.ipTwo;
	ipDestination[2] = networkAddress.ipThree;

	Ethernet.setLocalIP(ip);  // Change IP address to IP set by user
	Serial.println("\r\nEthernet status OK");
	Serial.print("IP set Manually: ");
	Serial.println(Ethernet.localIP());

	Udp.begin(localPort);
	NtripUdp.begin(NtripPort);

	GPS_setup();

	//----Teensy 4.1 Ethernet--End---------------------

	//----Teensy 4.1 CANBus--Start---------------------
#ifdef isAllInOneBoard
	pinMode(AUTOSTEER_STANDBY_LED, LOW);
	pinMode(AUTOSTEER_ACTIVE_LED, LOW);
#else
	pinMode(ledPin, OUTPUT);    //CAN Valve Ready LED
	digitalWrite(ledPin, LOW);

	pinMode(engageLED, OUTPUT);  //CAN engage LED
	digitalWrite(engageLED, LOW);

	pinMode(steeringLED, OUTPUT);  //Steering LED
	digitalWrite(steeringLED, LOW);
#endif

	Serial.println("\r\nStarting CAN-Bus Ports");
	Serial.println("Brand = SCANSTONE, forwarding GPS at 460800");

	Serial.println("\r\nStarting CAN-Bus Ports");
	if (Brand == 0) Serial.println("Brand = SCANSTONE (Set Via Service Tool)");
	else Serial.println("No Tractor Brand Set, Set Via Service Tool");

	Serial.println("\r\nGPS Mode:");
	if (gpsMode == 1) Serial.println("GPS Forwarding @ 115200 (Set Via Service Tool)");
	else if (gpsMode == 2) Serial.println("GPS Forwarding @ 460800 (Set Via Service Tool)");
	else if (gpsMode == 3) Serial.println("Panda Mode @ 115200 (Set Via Service Tool)");
	else if (gpsMode == 4) Serial.println("Panda Mode @ 460800 (Set Via Service Tool)");
	else Serial.println("No GPS mode selected - Set Via Service Tool");

	delay(3000);
	CAN_setup();   //Run the Setup void (CAN page)

	//----Teensy 4.1 CANBus--End---------------------

	Serial.print(inoVersion);
	Serial.println("\r\nSetup complete, waiting for AgOpenGPS");
	Serial.println("\r\nTo Start AgOpenGPS CANBUS Service Tool Enter 'S'");

	unsigned long start = micros();
	Serial.available();  // Polling time measurement
	unsigned long end = micros();

	Serial.print("Time taken: ");
	Serial.println(end - start); // Should be ~1-2 µs on AVR boards
}
// End of Setup

void loop()
{
	currentTime = millis();

	//--Main Timed Loop----------------------------------   
	if (currentTime - lastTime >= LOOP_TIME)
	{
		lastTime = currentTime;

		//reset debounce
		encEnable = true;

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250)
		{
			watchdogTimer = WATCHDOG_FORCE_VALUE;
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
		}

		//CANBus     
		if (steeringValveReady == 20 || steeringValveReady == 16)
		{
#ifdef isAllInOneBoard
			digitalWrite(AUTOSTEER_STANDBY_LED, HIGH);
			digitalWrite(AUTOSTEER_ACTIVE_LED, LOW);
#else
			digitalWrite(ledPin, HIGH);
#endif
		}
		else
		{
#ifdef isAllInOneBoard
			digitalWrite(AUTOSTEER_STANDBY_LED, LOW);
			digitalWrite(AUTOSTEER_ACTIVE_LED, LOW);
#else
			digitalWrite(ledPin, LOW);
#endif
		}

		//read all the switches
		workSwitch = digitalRead(WORKSW_PIN);     // read work switch (PCB pin)
		if (workCAN == 1) workSwitch = 0;         // If CAN workswitch is on, set workSwitch ON

		//Engage steering via 1 PCB Button or 2 Tablet or 3 CANBUS

		// 1 PCB Button pressed?
		reading = digitalRead(STEERSW_PIN);

		// 2 Has tablet button been pressed?
		if (previousStatus != guidanceStatus)
		{
			if (guidanceStatus == 1)    //Must have changed Off >> On
			{
#ifdef isAllInOneBoard
				digitalWrite(AUTOSTEER_ACTIVE_LED, HIGH);
				digitalWrite(AUTOSTEER_STANDBY_LED, LOW);
#else
				digitalWrite(engageLED, HIGH);
#endif
				engageCAN = 1;
				relayTime = ((millis() + 1000));

				currentState = 1;
			}
			else
			{
				currentState = 1;
				steerSwitch = 1;
			}

			previousStatus = guidanceStatus;
		}

		// 3 Has CANBUS button been pressed?
		if (engageCAN == 1) reading = 0;              //CAN Engage is ON (Button is Pressed)

		// Arduino software switch code
		if (reading == LOW && previous == HIGH)
		{
			if (currentState == 1)
			{
				if (Brand == 0) steeringValveReady = 16;  //Scanstone Ready To Steer 
				currentState = 0;
				steerSwitch = 0;
			}
			else
			{
				currentState = 1;
				steerSwitch = 1;
			}
		}
		previous = reading;

		//--------CAN CutOut--------------------------
		if (steeringValveReady != 20 && steeringValveReady != 16)
		{
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
			previous = HIGH;
		}

		remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
		switchByte = 0;
		switchByte |= (remoteSwitch << 2);  //put remote in bit 2
		switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
		switchByte |= workSwitch;

		//get steering position       

		//DETERMINE ACTUAL STEERING POSITION  *********From CAN-Bus************

			// TODO Sort out the steering curve here
		if (intendToSteer == 0) setCurve = estCurve;  //Not steering so setCurve = estCurve
		else steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;

		//Ackerman fix
		if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

		//Map WAS
		float mappedWAS;
		mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
		steerAngleActual = mappedWAS;

		if (useToolSteer) steerAngleError = steerAngleActual - toolSteerAngleSetPoint;   //calculate the steering error
		else steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error


		if (watchdogTimer < WATCHDOG_THRESHOLD)
		{
#ifdef isAllInOneBoard

#else
			//We are good to steer
			digitalWrite(steeringLED, 1);
#endif

			//calcSteeringPID(); //do the pid
			intendToSteer = 1; //CAN Curve Inteeded for Steering
		}
		else
		{
			//we've lost the comm to AgOpenGPS, or just stop request
			//****** If CAN engage is ON (1), don't turn off saftey valve ******

			intendToSteer = 0; //CAN Curve NOT Inteeded for Steering   
			pwmDrive = 0; //turn off steering motor
			pulseCount = 0;
#ifdef isAllInOneBoard

#else
			//We are good to steer
			digitalWrite(steeringLED, 0);
#endif
		}

		//-------CAN Set Curve ---------------

		//send empty pgn to AgIO to show activity
		if (++helloCounter > 10)
		{
			Udp.beginPacket(ipDestination, AOGPort);
			Udp.write(helloAgIO, sizeof(helloAgIO));
			Udp.endPacket();
			helloCounter = 0;
		}
	} //end of main timed loop


	if ((millis()) > relayTime) {
#ifdef isAllInOneBoard
		digitalWrite(AUTOSTEER_STANDBY_LED, HIGH);
		digitalWrite(AUTOSTEER_ACTIVE_LED, LOW);
#else
		digitalWrite(engageLED, LOW);
#endif
		engageCAN = 0;
	}

	//Service Tool
	if (Serial.available())
	{
		byte b = Serial.read();

		while (Serial.available()) {
			Serial.read();              //Clear the serial buffer
		}

		if (b == 'S') {
			Service = 1;
			Service_Tool();
		}
	}

	//--CAN--End-----

	//RVC BNO08x
	if (rvc.read(&bnoData)) useBNO08xRVC = true;

	if (useBNO08xRVC && bnoTimer > 40 && bnoTrigger)
	{
		bnoTrigger = false;
		imuHandler();   //Get IMU data ready
	}

	Panda_GPS();

	Forward_Ntrip();

	//Check for UDP Packet
	int packetSize = Udp.parsePacket();
	if (packetSize) {
		udpSteerRecv(packetSize);
	}

	if (encEnable)
	{
		thisEnc = digitalRead(REMOTE_PIN);
		if (thisEnc != lastEnc)
		{
			lastEnc = thisEnc;
			if (lastEnc) EncoderFunc();
		}
	}

} // end of main loop

//********************************************************************************

void udpSteerRecv(int sizeToRead)
{
	if (sizeToRead > 128) sizeToRead = 128;
	IPAddress src_ip = Udp.remoteIP();
	Udp.read(udpPacket.udpData, sizeToRead);

	if (udpPacket.AOGID == 0x8180 && udpPacket.MajorPGN == 0x7F) //Data
	{
		if (udpPacket.MinorPGN == PGNs::SteerData)  //254 // the ALL IMPORTANT steer data !!
		{
			gpsSpeed = ((float)(udpPacket.udpData[5] | udpPacket.udpData[6] << 8)) * 0.1;

			guidanceStatus = udpPacket.udpData[7];

			//Bit 8,9    set point steer angle * 100 is sent
			steerAngleSetPoint = ((float)(udpPacket.udpData[8] | ((int8_t)udpPacket.udpData[9]) << 8)) * 0.01; //high low bytes

			//Serial.println(gpsSpeed); 

			if ((bitRead(guidanceStatus, 0) == 0) || (steerSwitch == 1))        //AgOpen or Steer switch off
			{
				watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
			}
			else          //valid conditions to turn on autosteer
			{
				watchdogTimer = 0;  //reset watchdog
			}

			//Bit 10 Tram 
			tram = udpPacket.udpData[10];

			//Bit 11
			relay = udpPacket.udpData[11];

			//Bit 12
			relayHi = udpPacket.udpData[12];

			//----------------------------------------------------------------------------
			//Serial Send to agopenGPS

			int16_t sa = (int16_t)(steerAngleActual * 100);

			AOG[5] = (uint8_t)sa;
			AOG[6] = sa >> 8;

			//heading         
			AOG[7] = (uint8_t)9999;
			AOG[8] = 9999 >> 8;

			//roll
			AOG[9] = (uint8_t)8888;
			AOG[10] = 8888 >> 8;

			AOG[11] = switchByte;
			AOG[12] = (uint8_t)pwmDisplay;

			//checksum
			int16_t CK_A = 0;
			for (uint8_t i = 2; i < AOGSize - 1; i++)
				CK_A = (CK_A + AOG[i]);

			AOG[AOGSize - 1] = CK_A;

			//off to AOG
			Udp.beginPacket(ipDestination, 9999);
			Udp.write(AOG, AOGSize);
			Udp.endPacket();

			//Steer Data 2 -------------------------------------------------
			if (aog2Count++ > 2)
			{
				//Send fromAutosteer2
				if (steerConfig.CurrentSensor) PGN_250[5] = (byte)currentReading;
				else if (steerConfig.PressureSensor) PGN_250[5] = (byte)pressureReading;
				else PGN_250[5] = 0;

				//add the checksum for AOG2
				CK_A = 0;
				for (uint8_t i = 2; i < PGN_250_Size; i++)
				{
					CK_A = (CK_A + PGN_250[i]);
				}
				PGN_250[PGN_250_Size] = CK_A;

				Udp.beginPacket(ipDestination, 9999);
				Udp.write(PGN_250, sizeof(PGN_250));
				Udp.endPacket();
				aog2Count = 0;
			}

			// Stop sending the helloAgIO message
			helloCounter = 0;

			if (blink)
				digitalWrite(13, HIGH);
			else digitalWrite(13, LOW);
			blink = !blink;

			//Serial.println(steerAngleActual); 
			//--------------------------------------------------------------------------    
		}

		// Tool Steer
		else if (udpPacket.MinorPGN == PGNs::Destoner && Autosteer_running)  //233   SCANSTONE
		{
			int16_t temp_int16;

			temp_int16 = (int16_t)(udpPacket.udpData[8] | ((int8_t)udpPacket.udpData[9]) << 8);
			if (temp_int16 < 29000) tractorXTE = (float)temp_int16 * 0.001;
			else tractorXTE = 0.00;

			Serial.print("XTE = ");
			Serial.print(tractorXTE, 3);

			//integral slider is set to 0
			if (integralGain != 0)
			{
				pivotDistanceError = tractorXTE * 0.2 + pivotDistanceError * 0.8;

				if (integralCounter++ > 4)
				{
					pivotDerivative = pivotDistanceError - pivotDistanceErrorLast;
					pivotDistanceErrorLast = pivotDistanceError;
					integralCounter = 0;
					pivotDerivative *= 2;
				}

				if (steerSwitch == 0 && abs(pivotDerivative) < (0.1) && gpsSpeed > 0.5)
				{
					//if over the line heading wrong way, rapidly decrease integral
					if ((integral < 0 && tractorXTE < 0) || (integral > 0 && tractorXTE > 0))
					{
						integral += pivotDistanceError * integralGain * -0.04;
					}
					else
					{
						if (abs(tractorXTE) > 0.02)
						{
							integral += pivotDistanceError * integralGain * -0.02;
							if (integral > 0.2) integral = 0.2;
							else if (integral < -0.2) integral = -0.2;
						}
					}
				}
				else
				{
					integral *= 0.95;
				}
			}
			else integral = 0;

			Serial.print("\tInt = ");
			Serial.print(integral, 3);

			toolSteerAngleSetPoint = (tractorXTE + -integral) * -20;
			if (toolSteerAngleSetPoint > 20.0) toolSteerAngleSetPoint = 20.0;
			else if (toolSteerAngleSetPoint < -20.0) toolSteerAngleSetPoint = -20.0;

			Serial.print("\tSet = ");
			Serial.println(toolSteerAngleSetPoint, 1);

			Serial.print("Act = ");
			Serial.print(steerAngleActual, 1);
			Serial.print("\tErr = ");
			Serial.println(steerAngleError, 1);
		}

		else if (udpPacket.MinorPGN == PGNs::AgIOHello) //200 Hello from AgIO
		{
			int16_t sa = (int16_t)(steerAngleActual * 100);

			helloFromAutoSteer[5] = (uint8_t)sa;
			helloFromAutoSteer[6] = sa >> 8;

			helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
			helloFromAutoSteer[8] = helloSteerPosition >> 8;
			helloFromAutoSteer[9] = switchByte;


			Udp.beginPacket(ipDestination, 9999);
			Udp.write(helloFromAutoSteer, sizeof(helloFromAutoSteer));
			Udp.endPacket();

			if (useBNO08x || useBNO08xRVC)
			{
				Udp.beginPacket(ipDestination, 9999);
				Udp.write(helloFromIMU, sizeof(helloFromIMU));
				Udp.endPacket();
			}
		}


		//Machine Data
		else if (udpPacket.MinorPGN == PGNs::MachineData)  //239 Machine Data
		{
			hydLift = udpPacket.udpData[7];

			//reset for next pgn sentence
			isHeaderFound = isPGNFound = false;
			pgn = dataLength = 0;
		}

		//Machine Settings
		else if (udpPacket.MinorPGN == PGNs::MachineSettings) //238 Machine Settings 
		{
			aogConfig.raiseTime = udpPacket.udpData[5];
			aogConfig.lowerTime = udpPacket.udpData[6];
			//aogConfig.enableToolLift = udpPacket.udpData[7]; //This is wrong AgOpen is putting enable in sett,1

			//set1 
			uint8_t sett = udpPacket.udpData[8];  //setting0     
			if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
			if (bitRead(sett, 1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;

			//crc
			//udpPacket.udpData[13];        //crc

			//save in EEPROM and restart
			EEPROM.put(6, aogConfig);

			//reset for next pgn sentence
			isHeaderFound = isPGNFound = false;
			pgn = dataLength = 0;
		}

		else if (udpPacket.MinorPGN == PGNs::SteerSettings)  //252 steer settings
		{
			//PID values
			steerSettings.Kp = udpPacket.udpData[5];   // read Kp from AgOpenGPS

			steerSettings.highPWM = udpPacket.udpData[6]; // read high pwm

			steerSettings.lowPWM = udpPacket.udpData[8]; //udpPacket.udpData[7];   // read lowPWM from AgOpenGPS              

			steerSettings.minPWM = 1; //udpPacket.udpData[8]; //read the minimum amount of PWM for instant on

			steerSettings.steerSensorCounts = udpPacket.udpData[9]; //sent as setting displayed in AOG

			steerSettings.wasOffset = (udpPacket.udpData[10]);  //read was zero offset Lo

			steerSettings.wasOffset |= (((int8_t)udpPacket.udpData[11]) << 8);  //read was zero offset Hi

			steerSettings.AckermanFix = (float)udpPacket.udpData[12] * 0.01;

			//crc
			//udpPacket.udpData[13];

			//store in EEPROM
			EEPROM.put(10, steerSettings);

			// for PWM High to Low interpolator
			highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
		}

		else if (udpPacket.MinorPGN == PGNs::SteerConfig) //251 FB - SteerConfig
		{
			uint8_t sett = udpPacket.udpData[5]; //setting0

			if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
			if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
			if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
			if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
			if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
			if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
			if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
			if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

			steerConfig.PulseCountMax = udpPacket.udpData[6];

			//was speed
			//udpPacket.udpData[7]; 

			sett = udpPacket.udpData[8]; //setting1 - Danfoss valve etc

			if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
			if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
			if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
			if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

			//crc
			//udpPacket.udpData[13];        

			EEPROM.put(40, steerConfig);

			//reset for next pgn sentence
			isHeaderFound = isPGNFound = false;
			pgn = dataLength = 0;

		}//end FB

		else if (udpPacket.MinorPGN == PGNs::SubnetChange) // 201 Subnet change and restart
		{
			//make really sure this is the subnet pgn
			if (udpPacket.udpData[4] == 5 && udpPacket.udpData[5] == 201 && udpPacket.udpData[6] == 201)
			{
				networkAddress.ipOne = udpPacket.udpData[7];
				networkAddress.ipTwo = udpPacket.udpData[8];
				networkAddress.ipThree = udpPacket.udpData[9];

				//save in EEPROM and restart
				EEPROM.put(60, networkAddress);
				SCB_AIRCR = 0x05FA0004; //Teensy Reset
			}
		}//end C9

		else if (udpPacket.MinorPGN == PGNs::SubnetRequest) // 202 Who Am I ?
		{
			//make really sure this is the reply pgn
			if (udpPacket.udpData[4] == 3 && udpPacket.udpData[5] == 202 && udpPacket.udpData[6] == 202)
			{
				//hello from AgIO
				uint8_t scanReply[] = { 128, 129, 126, 203, 7,
				ip[0], ip[1], ip[2], 126, src_ip[0], src_ip[1], src_ip[2], 23 };

				//checksum
				int16_t CK_A = 0;
				for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
				{
					CK_A = (CK_A + scanReply[i]);
				}
				scanReply[sizeof(scanReply) - 1] = CK_A;

				static uint8_t ipDest[] = { 255,255,255,255 };
				uint16_t portDest = 9999; //AOG port that listens

				//off to AOG
				Udp.beginPacket(ipDest, portDest);
				Udp.write(scanReply, sizeof(scanReply));
				Udp.endPacket();

				Serial.print("\r\nAdapter IP: ");
				Serial.print(src_ip[0]); Serial.print(" . ");
				Serial.print(src_ip[1]); Serial.print(" . ");
				Serial.print(src_ip[2]); Serial.print(" . ");
				Serial.print(src_ip[3]);

				Serial.print("\r\nModule  IP: ");
				Serial.print(ip[0]); Serial.print(" . ");
				Serial.print(ip[1]); Serial.print(" . ");
				Serial.print(ip[2]); Serial.print(" . ");
				Serial.print(ip[3]); Serial.println();

				Serial.println(inoVersion); Serial.println();

				Serial.println(" --------- ");

			}
		}//end 202

	} //end if 80 81 7F

} //end udp callback


//ISR Steering Wheel Encoder

void EncoderFunc()
{
	if (encEnable)
	{
		pulseCount++;
		encEnable = false;
	}
}

//Rob Tillaart, https://github.com/RobTillaart/MultiMap
template<typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
	// take care the value is within range
	// value = constrain(value, _in[0], _in[size-1]);
	if (value <= _in[0]) return _out[0];
	if (value >= _in[size - 1]) return _out[size - 1];

	// search right interval
	uint8_t pos = 1;  // _in[0] already tested
	while (value > _in[pos]) pos++;

	// this will handle all exact "points" in the _in array
	if (value == _in[pos]) return _out[pos];

	// interpolate in the right segment for the rest
	return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
