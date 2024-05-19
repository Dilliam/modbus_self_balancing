#include <SPI.h>
#include <PortentaEthernet.h>
#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
// The IP address will be dependent on your local network:
IPAddress ip(192, 168, 1, 177);

EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient);

IPAddress server(192, 168, 1, 10); // update with the IP Address of your Modbus server

int receivedChar = 0;
String str;

unsigned long startTime;
unsigned long endTime;
char strBuf[50];
int status = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) {
    ;
  }

  Ethernet.begin(ip);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

}

void loop() {
  main_loop();

}

void main_loop()
{
  Serial.println("Attempting to connect to Modbus TCP server");
  if (!modbusTCPClient.connected()){
    if (!modbusTCPClient.begin(server)) {
      Serial.println("Modbus TCP Client failed to connect!");
      modbusTCPClient.stop();
    } else {
      Serial.println("Modbus TCP Client connected");
    }
  }
  delay(1000);
  while( modbusTCPClient.connected() ) {

    if (!writeCoil(0x01)){
      break;
    }

    delay(1000);
    
    if (!writeCoil(0x00)){
      break;
    }

    delay(1000);
  }
}

bool writeCoil(byte value)
{
  bool status = modbusTCPClient.coilWrite(0x00, value);
  if (!status) {
    Serial.println("Failed to write coil");
    modbusTCPClient.stop();
  }
  return status;
}