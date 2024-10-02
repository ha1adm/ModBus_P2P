#define Modbus_Baudrate 9600
// Message Header
const byte Header0 = 0x42;
const byte Header1 = 0x55;

const char HeaderString [] ="4255";

// Acknowledge String
char AckString [] = "DEADBEEF";

// Timeout value for Receive in ms
const unsigned long ReceiveTimeout = 2000UL;
// Timeout value for Receive Acknowlegdement in ms
const unsigned long ReceiveAckTimeout = 5000UL;

// Cyclic Input read period in seconds
const unsigned long ReadPeriod = 5UL;
