#include <Modbus_P2P.h>
#include <board_def.h>
#include <settings.h>

// LoRa modul serial
HardwareSerial LoRaserial(2);
// Modbus over Software serial
HardwareSerial ModbusSerial(1);

// Define the array of leds
CRGB leds[NUM_LEDS];
// instantiate Logger obejct
logging::Logger logger;
// instantiate LoRa Radio object
LoRa_E220 e220ttl(&LoRaserial, ebyte_AUX, ebyte_M0,ebyte_M1);
// instantiate ModbusMaster object
ModbusMaster node; 

byte b_tx_payload [24], b_payload_save[24], b_rx_payload [24];
byte DataToTransmitIsValid;
bool TxOnChange = false;
int nackCounter;  
unsigned long Elapsed; // To calc the execution time
unsigned long lastMillisRead;

// How often the Inputs is Read
const unsigned long DataSampleRate = ReadPeriod*1000UL;



//----------------------------------------------------------------------
// Print the parameters the E220 Radiomodule
//----------------------------------------------------------------------
void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
	Serial.println(F(" "));
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
	Serial.println(F(" "));
	Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
	Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
	Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
	Serial.println(F(" "));
	Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
	Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
	Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
	Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


	Serial.println("----------------------------------------");
}
//----------------------------------------------------------------------
// Print the module information the E220 Radiomodul
//----------------------------------------------------------------------
void printModuleInformation(struct ModuleInformation moduleInformation) {
	Serial.println("----------------------------------------");
	Serial.print(F("HEAD: "));  Serial.print(moduleInformation.COMMAND, HEX);Serial.print(" ");Serial.print(moduleInformation.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(moduleInformation.LENGHT, DEC);

	Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");
}
//----------------------------------------------------------------------
// Gather the module information and parameters from the E220 Radiomodule
//----------------------------------------------------------------------
void GetSettings_E220(){
	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);
	printParameters(configuration);
	ResponseStructContainer cMi;
	cMi = e220ttl.getModuleInformation();
	// It's important get information pointer before all other operation
	ModuleInformation mi = *(ModuleInformation*)cMi.data;
	Serial.println(cMi.status.getResponseDescription());
	Serial.println(cMi.status.code);
	printModuleInformation(mi);
	c.close();
	cMi.close();
}
//----------------------------------------------------------------------
// Profiling routines
//----------------------------------------------------------------------
void MarkTime()
{
  Elapsed=millis();
}
//----------------------------------------------------------------------
void ShowTime()
{
  // Calcs the time
  Elapsed=millis()-Elapsed;
  logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "Process", "Job time (ms) : %ld ",Elapsed);   
}
//----------------------------------------------------------------------
// Hex String to Byte array
//----------------------------------------------------------------------
void strToBin(byte bin[], char const str[]) {
  for (size_t i = 0; str[i] and str[i + 1]; i += 2) {
    char slice[] = {0, 0, 0};
    strncpy(slice, str + i, 2);
    bin[i / 2] = strtol(slice, nullptr, 16);
  }
}
//----------------------------------------------------------------------
// Copy Src array to dst array. Size have to be the same
//----------------------------------------------------------------------
void copyArray(byte* src, byte* dst, int len) {
    for (int i = 0; i < len; i++) {
        *dst++ = *src++;
    }
}
//----------------------------------------------------------------------
// Set the Color of the Diag Neopixel
//----------------------------------------------------------------------
void SetDiagLED(CRGB color){
	leds[0] = color;
	FastLED.show();
}
//----------------------------------------------------------------------
// Write WellPro Modbus Modul Digital outputs 4DO
//----------------------------------------------------------------------
bool WP_WriteModbus4DO(byte slaveId, byte data){
	uint8_t result;
  node.setSlaveId(slaveId);
	node.setTransmitBuffer(0, data);
	result = node.writeMultipleCoils(0x0000, 4);
		if (result == node.ku8MBSuccess)
		{	
			#ifdef Debug
			logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "Modbus", "Write outputs Successfull!");
			#endif
			return true;

		}
		else 
		{
			logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "Modbus", "Error at writing outputs! Error Nr.: %i ",result);
			return false;
		}
}
//----------------------------------------------------------------------
// Read WellPro Modbus Modul Digital inputs 1 byte
//----------------------------------------------------------------------
bool WP_ReadModbus8DI(byte slaveId, byte &data){
	uint8_t result;
  node.setSlaveId(slaveId);
	result = node.readDiscreteInputs(0x00, 8);
		if (result == node.ku8MBSuccess)
		{
			data = node.getResponseBuffer(0x00);
			#ifdef Debug
			logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "Modbus", "Read inputs Successfull! Data: %i ",data);
			#endif
			return true;
		}
		else 
		{
			logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "Modbus", "Error at reading inputs! Error Nr.: %i ",result);
			data = 0;
			return false;
		}
}
//----------------------------------------------------------------------
// Check Radio Module config
//----------------------------------------------------------------------
void CheckRadioConfig(){
	ResponseStructContainer c;
	c = e220ttl.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);

	ResponseStructContainer cMi;
	cMi = e220ttl.getModuleInformation();
	// It's important get information pointer before all other operation
	ModuleInformation mi = *(ModuleInformation*)cMi.data;

	Serial.println(cMi.status.getResponseDescription());
	Serial.println(cMi.status.code);

	printModuleInformation(mi);

	c.close();
	cMi.close();
}
//----------------------------------------------------------------------
// Send Payload over radio (inkl. waiting Ack)
//----------------------------------------------------------------------
void Transmitbytes(const byte* data, uint8_t size){
    char msgBuffer[size*2 + 1];

    char buffer[3];
    for (unsigned i=0; i<size; i++)
    {
      sprintf(buffer, "%02X", data[i]);
      memcpy(&msgBuffer[i*2], &buffer, sizeof(buffer));
    }
    String dataToTx(msgBuffer);
	// Send message
	ResponseStatus rs = e220ttl.sendMessage(dataToTx);
	// Check If there is some problem of succesfully send
	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Transmission result: %s",rs.getResponseDescription().c_str());

}
//----------------------------------------------------------------------
// Send Acknowledge
//----------------------------------------------------------------------
void SendAck(String ack_str){
	// Send message
	ResponseStatus rs = e220ttl.sendMessage(ack_str);
	// Check If there is some problem of succesfully send
	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Acknowledge sent with result: %s",rs.getResponseDescription().c_str());
}
//----------------------------------------------------------------------
// Receive Acknowledge
//----------------------------------------------------------------------
bool ReceiveAck(unsigned long timeout){
	unsigned long t = millis();
	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0) {
		t = 0;
	}
	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Starting to listen for Acknowledge... ");
	while (!e220ttl.available()) {
		if ((millis() - t) > timeout){
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Received nothing");
        return false;
		}
	}
        if (e220ttl.available()>1) {
			ResponseContainer rc = e220ttl.receiveMessage();
			if (rc.status.code!=1){
				logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "RADIO", "%s",rc.status.getResponseDescription().c_str());
				return false;
			}
			else {
        	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Received data: ");
        	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", rc.data.c_str());
        	// Packet Filter
        	if (rc.data.equals(AckString)) {
          		return true;
        	}
        	else {
          		logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "RADIO", "Received foreign payload!");
          		return false;
          	}
        }
		}
		else{
			logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Received nothing");
        	return false;	
		}
    }
//----------------------------------------------------------------------
// Send Payload over radio (inkl. waiting Ack)
//----------------------------------------------------------------------
bool SendPayload(){
    // Debug payload
    Serial.print("TX Payload: ");
    for (int i = 0; i < sizeof(b_tx_payload); i++) Serial.print(b_tx_payload[i]);
    logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Transmitting!!");
	SetDiagLED(CRGB::Aqua);
    Transmitbytes(b_tx_payload, sizeof(b_tx_payload));
    // Delay Added to Avoid Radio Busy in higher SF's
    delay(200);
    nackCounter = 0;
    while(!ReceiveAck(ReceiveAckTimeout) && nackCounter <= 5){
	  SetDiagLED(CRGB::Gold);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "RADIO", "No Acknowladge received after %i retransmissions!",nackCounter);
      delay(1000);
      Transmitbytes(b_tx_payload, sizeof(b_tx_payload));
      // Delay Added to Avoid Radio Busy in higher SF's
      delay(200);
      nackCounter++;
      }
    if (nackCounter >= 5) {
	  SetDiagLED(CRGB::FireBrick);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "RADIO", "MESSAGE LOST!");
      delay(100);
	  return false;
      } 
      else {
	  SetDiagLED(CRGB::DarkBlue);
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Transmission Acknowladged!");
	  return true;
      }
    
}
//----------------------------------------------------------------------
// Receive Data over Radio
//----------------------------------------------------------------------
bool Receive(unsigned long timeout) {
	unsigned long t = millis();
	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}
	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Starting to listen ... ");
	while (!e220ttl.available()) {
		if ((millis() - t) > timeout){
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Received nothing");
        return false;
		}
	}
	ResponseContainer rc = e220ttl.receiveMessage();
	if (rc.status.code!=1){
		logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "RADIO", "%s",rc.status.getResponseDescription().c_str());
		return false;
		}
	else {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RADIO", "Received data: %s",rc.data.c_str());
      // Packet Filter
      if (rc.data.substring(0,4).equals(HeaderString)){
        strToBin(b_rx_payload, rc.data.c_str());
        return true;
		}
      else{
        logger.log(logging::LoggerLevel::LOGGER_LEVEL_WARN, "RADIO", "Received foreign payload!");
        return false;
        }
	}
  }

void setup() {
	Serial.begin(115200);
	// Wait a maximum of 10s for Serial Monitor
	while (!Serial && millis() < 10000);
	logger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "MAIN", "Logger Started");
  	LoRaserial.begin(9600, SERIAL_8N1, ebyte_RX, ebyte_TX);
	ModbusSerial.begin(Modbus_Baudrate, SERIAL_8N1, rs485_RX, rs485_TX);
  	// Modbus slave ID 1
	node.begin(1, ModbusSerial);
	// Startup all pins and UART
	e220ttl.begin();	// Start up Diag LED
	FastLED.addLeds<WS2812B, Neopixel>(leds, NUM_LEDS);
  	FastLED.setBrightness(24);
	// Initialize digital pin Onboard Relay as an output.
	pinMode(onboard_relay, OUTPUT);
	// Initialise Input pins
	pinMode(DI_1, INPUT);
	pinMode(DI_2, INPUT);
	// Initialise Header bytes in tx payload for payload validation
	b_tx_payload [0] = Header0;
	b_tx_payload [1] = Header1;
	// Analog input pin 36 is unconnected, random analog
  	// noise will cause the call to randomSeed() to generate
  	// different seed numbers each time the sketch runs.
  	// randomSeed() will then shuffle the random function.
  	randomSeed(analogRead(36));
}

void loop() {
    if ((millis() - lastMillisRead >= DataSampleRate) || lastMillisRead == 0) {
		if(WP_ReadModbus8DI(1,b_tx_payload [2])){
			SetDiagLED(CRGB::DarkGreen);
			lastMillisRead = millis();  //get ready for the next iteration
      		// Compare saved data with the newly read
      		if (b_tx_payload[2] != b_payload_save[2] ) {
        		TxOnChange = true;
        		copyArray(b_tx_payload, b_payload_save, (sizeof(b_tx_payload) / sizeof(b_tx_payload[0])));
				} 
		}
		else{
			SetDiagLED(CRGB::Crimson);
			delay(250);
		}
  
    }
	else{
		SetDiagLED(CRGB::Black);
	}
	if (TxOnChange) {
		MarkTime();
		if(SendPayload()){
			TxOnChange = false;
		}
		else {
			delay(random(1000,5000));
		}
		ShowTime();
	}
	if (Receive(ReceiveTimeout)){
		SendAck(AckString);
		WP_WriteModbus4DO(1,b_rx_payload[2]);
	}
}