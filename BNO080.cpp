/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017
  Edited by James Ligon for ECEN 403

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

	SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.
*/

#include "SparkFun_BNO080_Arduino_Library.h"

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
bool BNO080::begin(uint8_t deviceAddress, TwoWire &wirePort, uint8_t intPin)
{
	_deviceAddress = deviceAddress; //If provided, store the I2C address from user
	_i2cPort = &wirePort;			//Grab which port the user wants us to use
	_int = intPin;					//Get the pin that the user wants to use for interrupts. By default, it's 255 and we'll not use it in dataAvailable() function.
	if (_int != 255)
	{
		pinMode(_int, INPUT_PULLUP);
	}

	//We expect caller to begin their I2C port, with the speed of their choice external to the library
	//But if they forget, we start the hardware here.
	//_i2cPort->begin();

	//Begin by resetting the IMU
	softReset();

	//Check communication with device
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket() == true)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			if (_printDebug == true)
			{
				_debugPort->print(F("SW Version Major: 0x"));
				_debugPort->print(shtpData[2], HEX);
				_debugPort->print(F(" SW Version Minor: 0x"));
				_debugPort->print(shtpData[3], HEX);
				uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
				_debugPort->print(F(" SW Part Number: 0x"));
				_debugPort->print(SW_Part_Number, HEX);
				uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
				_debugPort->print(F(" SW Build Number: 0x"));
				_debugPort->print(SW_Build_Number, HEX);
				uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
				_debugPort->print(F(" SW Version Patch: 0x"));
				_debugPort->println(SW_Version_Patch, HEX);
			}
			return (true);
		}
	}

	return (false); //Something went wrong
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void BNO080::enableDebugging(Stream &debugPort)
{
	_debugPort = &debugPort;
	_printDebug = true;
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
	return (getReadings() != 0);
}

uint16_t BNO080::getReadings(void)
{
	//If we have an interrupt pin connection available, check if data is available.
	//If int pin is not set, then we'll rely on receivePacket() to timeout
	//See issue 13: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/13
	if (_int != 255)
	{
		if (digitalRead(_int) == HIGH)
			return 0;
	}

	if (receivePacket() == true)
	{
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
		}
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      return parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
    }
	}
	return 0;
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t BNO080::parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
uint16_t BNO080::parseInputReport(void)
{
	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= 4; //Remove the header bytes from the data count

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
	if(shtpHeader[2] == CHANNEL_GYRO) {
		rawQuatI = (uint16_t)shtpData[1] << 8 | shtpData[0];
		rawQuatJ = (uint16_t)shtpData[3] << 8 | shtpData[2];
		rawQuatK = (uint16_t)shtpData[5] << 8 | shtpData[4];
		rawQuatReal = (uint16_t)shtpData[7] << 8 | shtpData[6];
		rawFastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
		rawFastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
		rawFastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

		return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	uint16_t data4 = 0;
	uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

	if (dataLength - 5 > 9)
	{
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11)
	{
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
	{
		accelAccuracy = status;
		rawAccelX = data1;
		rawAccelY = data2;
		rawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
	{
		accelLinAccuracy = status;
		rawLinAccelX = data1;
		rawLinAccelY = data2;
		rawLinAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
	{
		gyroAccuracy = status;
		rawGyroX = data1;
		rawGyroY = data2;
		rawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
	{
		magAccuracy = status;
		rawMagX = data1;
		rawMagY = data2;
		rawMagZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
	{
		quatAccuracy = status;
		rawQuatI = data1;
		rawQuatJ = data2;
		rawQuatK = data3;
		rawQuatReal = data4;

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		rawQuatRadianAccuracy = data5;
	}
	else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR)
	{
		tapDetector = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
	{
		stepCount = data3; //Bytes 8/9
	}
	else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
	{
		stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
	}
	else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
	{
		activityClassifier = shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
			_activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
	{
		memsRawAccelX = data1;
		memsRawAccelY = data2;
		memsRawAccelZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
	{
		memsRawGyroX = data1;
		memsRawGyroY = data2;
		memsRawGyroZ = data3;
	}
	else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
	{
		memsRawMagX = data1;
		memsRawMagY = data2;
		memsRawMagZ = data3;
	}
	else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		if (_printDebug == true)
		{
			_debugPort->println(F("!"));
		}
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE)
		{
			if (_printDebug == true)
			{
				_debugPort->println(F("ME Cal report found!"));
			}
			calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
		return 0;
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return shtpData[5];
}


//Gets the full gyro vector
//x,y,z output floats
/*void BNO080::getGyro(float &x, float &y, float &z, uint8_t &accuracy)
{
	x = qToFloat(rawGyroX, gyro_Q1);
	y = qToFloat(rawGyroY, gyro_Q1);
	z = qToFloat(rawGyroZ, gyro_Q1);
	accuracy = gyroAccuracy;
}
*/
//Return the gyro component
float BNO080::getGyroX()
{
	float gyro = qToFloat(rawGyroX, gyro_Q1);
	return (gyro);
}

//Return the step count
uint16_t BNO080::getStepCount()
{
	return (stepCount);
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080::softReset(void)
{
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
void BNO080::modeOn(void)
{
	shtpData[0] = 2; //On

	//Attempt to start communication with sensor
	sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
	delay(50);
	while (receivePacket() == true)
		; //delay(1);
}

// Indicates if we've received a Reset Complete packet. Once it's been read, 
// the state will reset to false until another Reset Complete packet is found. 
bool BNO080::hasReset() {
	if (_hasReset) {
		_hasReset = false;
		return true;
	}
	return false; 
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

//Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080::enableStepCounter(uint16_t timeBetweenReports)
{
	setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
	setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;								   //Feature flags
	shtpData[3] = 0;								   //Change sensitivity (LSB)
	shtpData[4] = 0;								   //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;								   //Batch Interval (LSB)
	shtpData[10] = 0;								   //Batch Interval
	shtpData[11] = 0;								   //Batch Interval
	shtpData[12] = 0;								   //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(CHANNEL_CONTROL, 12);
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
bool BNO080::waitForI2C()
{
	for (uint8_t counter = 0; counter < 100; counter++) //Don't got more than 255
	{
		if (_i2cPort->available() > 0)
			return (true);
		delay(1);
	}

	if (_printDebug == true)
		_debugPort->println(F("I2C timeout"));
	return (false);
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080::receivePacket(void)
{
	if (_i2cPort == NULL) //Do SPI
	{
		if (digitalRead(_int) == HIGH)
			return (false); //Data is not available

		//Old way: if (waitForSPI() == false) return (false); //Something went wrong

		//Get first four bytes to find out how much data we need to read

		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _spiPort->transfer(0);
		uint8_t packetMSB = _spiPort->transfer(0);
		uint8_t channelNumber = _spiPort->transfer(0);
		uint8_t sequenceNumber = _spiPort->transfer(0); //Not sure if we need to store this or not

		//Store the header info
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit
		if (dataLength == 0)
		{
			//Packet is empty
			printHeader();
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		//Read incoming data into the shtpData array
		for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
		{
			uint8_t incoming = _spiPort->transfer(0xFF);
			if (dataSpot < MAX_PACKET_SIZE)	//BNO080 can respond with upto 270 bytes, avoid overflow
				shtpData[dataSpot] = incoming; //Store data into the shtpData array
		}

		digitalWrite(_cs, HIGH); //Release BNO080

		_spiPort->endTransaction();
		printPacket();
	}
	else //Do I2C
	{
		_i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)4); //Ask for four bytes to find out how much data we need to read
		if (waitForI2C() == false)
			return (false); //Error

		//Get the first four bytes, aka the packet header
		uint8_t packetLSB = _i2cPort->read();
		uint8_t packetMSB = _i2cPort->read();
		uint8_t channelNumber = _i2cPort->read();
		uint8_t sequenceNumber = _i2cPort->read(); //Not sure if we need to store this or not

		//Store the header info.
		shtpHeader[0] = packetLSB;
		shtpHeader[1] = packetMSB;
		shtpHeader[2] = channelNumber;
		shtpHeader[3] = sequenceNumber;

		//Calculate the number of data bytes in this packet
		uint16_t dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
		dataLength &= ~(1 << 15); //Clear the MSbit.
		//This bit indicates if this package is a continuation of the last. Ignore it for now.
		//TODO catch this as an error and exit

		// if (_printDebug == true)
		// {
		// 	_debugPort->print(F("receivePacket (I2C): dataLength is: "));
		// 	_debugPort->println(dataLength);
		// }

		if (dataLength == 0)
		{
			//Packet is empty
			return (false); //All done
		}
		dataLength -= 4; //Remove the header bytes from the data count

		getData(dataLength);
	}

	// Quickly check for reset complete packet. No need for a seperate parser.
	// This function is also called after soft reset, so we need to catch this
	// packet here otherwise we need to check for the reset packet in multiple
	// places.
	if (shtpHeader[2] == CHANNEL_EXECUTABLE && shtpData[0] == EXECUTABLE_RESET_COMPLETE) 
	{
		_hasReset = true;
	} 

	return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool BNO080::getData(uint16_t bytesRemaining)
{
	uint16_t dataSpot = 0; //Start at the beginning of shtpData array

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0)
	{
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
			numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

		_i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)(numberOfBytesToRead + 4));
		if (waitForI2C() == false)
			return (0); //Error

		//The first four bytes are header bytes and are throw away
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();
		_i2cPort->read();

		for (uint8_t x = 0; x < numberOfBytesToRead; x++)
		{
			uint8_t incoming = _i2cPort->read();
			if (dataSpot < MAX_PACKET_SIZE)
			{
				shtpData[dataSpot++] = incoming; //Store data into the shtpData array
			}
			else
			{
				//Do nothing with the data
			}
		}

		bytesRemaining -= numberOfBytesToRead;
	}
	return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	if (_i2cPort == NULL) //Do SPI
	{
		//Wait for BNO080 to indicate it is available for communication
		if (waitForSPI() == false)
			return (false); //Something went wrong

		//BNO080 has max CLK of 3MHz, MSB first,
		//The BNO080 uses CPOL = 1 and CPHA = 1. This is mode3
		_spiPort->beginTransaction(SPISettings(_spiPortSpeed, MSBFIRST, SPI_MODE3));
		digitalWrite(_cs, LOW);

		//Send the 4 byte packet header
		_spiPort->transfer(packetLength & 0xFF);			 //Packet length LSB
		_spiPort->transfer(packetLength >> 8);				 //Packet length MSB
		_spiPort->transfer(channelNumber);					 //Channel number
		_spiPort->transfer(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_spiPort->transfer(shtpData[i]);
		}

		digitalWrite(_cs, HIGH);
		_spiPort->endTransaction();
	}
	else //Do I2C
	{
		//if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

		_i2cPort->beginTransmission(_deviceAddress);

		//Send the 4 byte packet header
		_i2cPort->write(packetLength & 0xFF);			  //Packet length LSB
		_i2cPort->write(packetLength >> 8);				  //Packet length MSB
		_i2cPort->write(channelNumber);					  //Channel number
		_i2cPort->write(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

		//Send the user's data packet
		for (uint8_t i = 0; i < dataLength; i++)
		{
			_i2cPort->write(shtpData[i]);
		}

		uint8_t i2cResult = _i2cPort->endTransmission();

		if (i2cResult != 0)
		{
			if (_printDebug == true)
			{
				_debugPort->print(F("sendPacket(I2C): endTransmission returned: "));
				_debugPort->println(i2cResult);
			}
			return (false);
		}
	}

	return (true);
}
