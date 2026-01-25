#Device Name: Pector 
#Full Form: Portable Electrical Conductivity Test Observer and Reader.

#Device Specification: ESP32 based Soil Sensor Gateway using Modbus RS485, TCP server, OTA updates and LCD User Interface
#Used Modules: 4x(ESP32-S3-DevKitC-1), 4x(MAX485 RS-485 Module), 1x(12V Adapter), 1x(12V to 5V DC-DC converter), 4x(LCD Display)

## Features
- ESP32 WiFi connectivity
- RS485 Modbus
- TCP server 
    - Port 3131 for Pector 1
    - Port 3132 for Pector 2
    - Port 3133 for Pector 3
    - Port 3134 for Pector 4
- OTA update support
- mDNS discovery 
    - pector_1.local
    - pector_2.local
    - pector_3.local
    - pector_4.local
- 16x2 I2C LCD display
    - Display outputs: Moisture, Electrical Conductivity, pH as ( Moist | EC | pH )
    - Pector 2 will show pH value '0' as the sensor does not support pH readings
- Soil sensor readings:
  - Moisture
  - Temperature
  - EC
  - pH
  - NPK (Nitrogen, Phosphorus, Potassium)

## Configuration for Pector 1
// TCP server and Port allocation
static const char* MDNS_NAME = "pector_1";
static const uint16_t TCP_PORT = 3131;
## Configuration for Pector 2
// TCP server and Port allocation
static const char* MDNS_NAME = "pector_2";
static const uint16_t TCP_PORT = 3132;
## Configuration for Pector 3
// TCP server and Port allocation
static const char* MDNS_NAME = "pector_3";
static const uint16_t TCP_PORT = 3133;
## Configuration for Pector 4
// TCP server and Port allocation
static const char* MDNS_NAME = "pector_4";
static const uint16_t TCP_PORT = 3134;

## OTA Port : 3232
## OTA Password : pector1234

#Device Using Guidlines:
1. Connect the device to the AC power outlet and turn on the switch
2. The LCD Displays will show the specific results

## To check the data through Wireless Connectivity, User have to use the E_LAB_BS_Terminal application
#BS Terminal Using Guidlines: (Monitor through Wireless Connectivity)
1. Open the BS Terminal Application, 
    - Set TCP mode "Client"
    - Host "pector_<serial_number>.local" (Use 1,2,3,4 for specific Pectors)
    - Port "313<serial_number>" (Use 1,2,3,4 for specific Pectors)
    - Hit Start TCP
2. If all the pectors are properly connected to the WIFI the data will be shown in the Terminal

## Standard Operations Procedure for soil tests are given below

1. Make sure the soil is evenly mixed
2. Make sure the soil’s hardness remains same, all over the sample
3. Clean the sensor probes every time with distilled water before changing sample points
4. If the device gives unexpected value. Restart the device and try again
5. Try to take the data when the reading is stable
6. If the sensor data reading fluctuates from 10-20 differences, no need to worry, write the one which seems in between.
7. At every point of checking, make sure the sensor probes have proper contact with the soil sample.