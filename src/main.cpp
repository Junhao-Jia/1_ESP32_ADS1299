#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

#include <ESPmDNS.h>

#include <ArduinoJson.h>

#include <Wire.h>
#include "u8g2Display.h"

#define ADS1299_PIN_RESET 25
#define ADS1299_PIN_DRDY 27    //data-ready output

#define ADS1299_PIN_SCK 14
#define ADS1299_PIN_MISO 12
#define ADS1299_PIN_MOSI 13
#define ADS1299_PIN_SS 15  //17

#define OPENBCI_DATA_BUFFER_SIZE 50

#define OPENBCI_NAME "OpenBCI-FFFF"
#define OPENBCI_VERSION "v2.0.5"

#define SOFT_AP_SSID "OpenBCI WiFi AP"
#define SOFT_AP_PASSWORD "12345678"

#define JSON_BUFFER_SIZE 1024
#define ADS_ID  0x3E  // product ID for ADS1299
#define ID_REG  0x00  // this register contains ADS_ID
#define BOARD_ADS  15  // ADS chip select  
#define _SDATAC 0x11 // Stop Read Data Continuous modeff
#define _RDATA 0x12 // Read data by command supports multiple read back
#define _RESET 0x06 // Reset the device registers to default
// Register Addresses
#define ID         0x00
#define CONFIG1    0x01
#define CONFIG2    0x02
#define CONFIG3    0x03
#define LOFF       0x04
#define CH1SET     0x05
#define CH2SET     0x06
#define CH3SET     0x07
#define CH4SET     0x08
#define CH5SET     0x09
#define CH6SET     0x0A
#define CH7SET     0x0B
#define CH8SET     0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP  0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO       0x14
#define MISC1      0x15
#define MISC2      0x16
#define CONFIG4    0x17

// Test Signal Choices - p41
#define ADS1299_TEST_INT              0x10 //(0b00010000)
#define ADS1299_TESTSIGNAL_AMP_1X     0x00 //(0b00000000)
#define ADS1299_TESTSIGNAL_AMP_2X     0x40 //(0b00000100)
#define ADS1299_TESTSIGNAL_PULSE_SLOW 0x00 //(0b00000000)
#define ADS1299_TESTSIGNAL_PULSE_FAST 0x01 //(0b00000001)
#define ADS1299_TESTSIGNAL_DCSIG      0x03 //(0b00000011)
#define ADS1299_TESTSIGNAL_NOCHANGE   0xff //(0b11111111)
// SPI Command Definitions (Datasheet, 35)
#define _WAKEUP  0x02 // Wake-up from standby mode
#define _STANDBY 0x04 // Enter Standby mode
#define _RESET   0x06 // Reset the device registers to default
#define _START   0x08 // Start and restart (synchronize) conversions
#define _STOP    0x0A // Stop conversion
#define _RDATAC  0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC  0x11 // Stop Read Data Continuous mode
#define _RDATA   0x12 // Read data by command; supports multiple read back
#define _RREG    0x20 // Read Register
#define _WREG    0x40 // Write to Register

// Gains
#define ADS1299_PGA_GAIN01 0x00 //(0b00000000)
#define ADS1299_PGA_GAIN02 0x10 //(0b00010000)
#define ADS1299_PGA_GAIN04 0x20 //(0b00100000)
#define ADS1299_PGA_GAIN06 0x30 //(0b00110000)
#define ADS1299_PGA_GAIN08 0x40 //(0b01000000)
#define ADS1299_PGA_GAIN12 0x50 //(0b01010000)
#define ADS1299_PGA_GAIN24 0x60 //(0b01100000)

// Input Modes - Channels

#define ADS1299_INPUT_PWR_DOWN   0x80 //(0b10000000)
#define ADS1299_INPUT_PWR_UP     0x00 //(0b00000000)

#define ADS1299_INPUT_NORMAL     0x00 //(0b00000000)
#define ADS1299_INPUT_SHORTED    0x01 //(0b00000001)
#define ADS1299_INPUT_MEAS_BIAS  0x02 //(0b00000010)
#define ADS1299_INPUT_SUPPLY     0x03 //(0b00000011)
#define ADS1299_INPUT_TEMP       0x04 //(0b00000100)
#define ADS1299_INPUT_TESTSIGNAL 0x05 //(0b00000101)
#define ADS1299_INPUT_SET_BIASP  0x06 //(0b00000110)
#define ADS1299_INPUT_SET_BIASN  0x07 //(0b00000111)

//Lead-off Signal Choices
#define LOFF_MAG_6NA      0x00 //(0b00000000)
#define LOFF_MAG_24NA     0x04 //(0b00000100)
#define LOFF_MAG_6UA      0x08 //(0b00001000)
#define LOFF_MAG_24UA     0x0c //(0b00001100)
#define LOFF_FREQ_DC      0x00 //(0b00000000)
#define LOFF_FREQ_7p8HZ   0x01 //(0b00000001)
#define LOFF_FREQ_31p2HZ  0x02 //(0b00000010)
#define LOFF_FREQ_FS_4    0x03 //(0b00000011)
#define PCHAN (1)
#define NCHAN (2)
#define BOTHCHAN (3)
byte    regData[24] = {0}; // array is used to mirror register data
volatile int     boardStat = 0; //

/* u8g2 constructer */
u8g2Display u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping


/*
enum ads1299_command : uint8_t 表示定义的枚举类型 ads1299_command 的底层存储类型为 uint8_t，
  即无符号 8 位整数类型。这意味着枚举常量的值将被存储为 8 位无符号整数，通常取值范围是 0 到 255。
*/
enum ads1299_command : uint8_t
{
  //datasheet, p.40
    ads1299_command_start = 0x08,
    ads1299_command_stop = 0x0A,

    ads1299_command_rdatac = 0x10,
    ads1299_command_sdatac = 0x11,

    ads1299_command_rreg = 0x20,
    ads1299_command_wreg = 0x40
};

typedef struct ads1299_register_packet
{
    uint8_t id;

    uint8_t config1;
    uint8_t config2;
    uint8_t config3;

    uint8_t loff;

    uint8_t chnset[8];
    
    uint8_t bias_sensp;
    uint8_t bias_sensn;

    uint8_t loff_sensp;
    uint8_t loff_sensn;

    uint8_t loff_flip;

    uint8_t loff_statp;
    uint8_t loff_statn;

    uint8_t gpio;

    uint8_t misc1;
    uint8_t misc2;
    
    uint8_t config4;
} __attribute__ ((packed)) ads1299_register_packet;

typedef struct ads1299_data_packet
{
    uint32_t stat : 24; //24表示stat成员变量分配了24位的存储空间

     uint8_t channel_data[24];
} __attribute__ ((packed)) ads1299_data_packet;

typedef struct openbci_data_packet
{
    uint8_t header;

    uint8_t sample_number;

    uint8_t channel_data[24];

    uint8_t auxiliary_data[6];

    uint8_t footer;
} __attribute__ ((packed)) openbci_data_packet;

ads1299_register_packet ads1299_register_buffer = {};
ads1299_data_packet ads1299_data_buffer = {};

openbci_data_packet openbci_data_buffer[OPENBCI_DATA_BUFFER_SIZE] = {{}};//OPENBCI_DATA_BUFFER_SIZE equals 50

uint16_t openbci_data_buffer_head = 0;
uint16_t openbci_data_buffer_tail = 0;

uint8_t channel_setting_buffer[8] = {0};

uint8_t sample_counter = 0;

bool streaming_enabled = false;

uint8_t* tcp_transfer_buffer = NULL;

IPAddress local_ip(192, 168, 4, 1);
IPAddress network_gateway(192, 168, 4, 1);
IPAddress subnet_mask(255, 255, 255, 0);

WebServer web_server(80);
WiFiClient tcp_client;

size_t wifi_latency = 0;
//SPI communication method
byte xfer(byte _data)
{
  byte inByte;
  inByte = SPI.transfer(_data);
  return inByte;
}
void csHigh(int SS)
{ // deselect SPI slave
  switch (SS)
  {
  case BOARD_ADS:
    digitalWrite(BOARD_ADS, HIGH);
    break;
  default:
    break;
  }
}
//SPI chip select method
void csLow(int SS)
{ // select an SPI slave to talk to
  switch (SS)
  {
  case BOARD_ADS:
    //SPI.setMode(DSPI_MODE1);
    //SPI.setSpeed(4000000);
    //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    digitalWrite(BOARD_ADS, LOW);
    break;
  default:
    break;
  }
}
byte RREG(byte _address, int targetSS)
{                                 //  reads ONE register at _address
  byte opcode1 = _address + 0x20; //  RREG expects 001rrrrr where rrrrr = _address
  csLow(targetSS);                //  open SPI
  xfer(opcode1);                  //  opcode1
  xfer(0x00);                     //  opcode2
  regData[_address] = xfer(0x00); //  update mirror location with returned byte
  csHigh(targetSS);               //  close SPI
  return regData[_address]; // return requested register value
}
void WREG(byte _address, byte _value )
{   
  int target_SS=BOARD_ADS;    //  Write ONE register at _address
  byte opcode1 = _address + 0x40; //  WREG expects 010rrrrr where rrrrr = _address
  csLow(target_SS);               //  open SPI
  xfer(opcode1);                  //  Send WREG command & address
  xfer(0x00);                     //  Send number of registers to read -1
  xfer(_value);                   //  Write the value to the register
  csHigh(target_SS);              //  close SPI
  regData[_address] = _value;     //  update the mirror array
}

void SDATAC(int targetSS)
{
  csLow(targetSS);
  xfer(_SDATAC);
  csHigh(targetSS);
  delayMicroseconds(10); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}
void RDATAC(int targetSS)
{
  csLow(targetSS);
  xfer(_RDATAC);
  csHigh(targetSS);
  delayMicroseconds(10); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}
void START(int targetSS)
{
  csLow(targetSS);
  xfer(_START);
  csHigh(targetSS);
  delayMicroseconds(10); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}
byte ADS_getDeviceID(int targetSS)
{ // simple hello world com check
  byte data = RREG(ID_REG, targetSS);
  return data;
}


//  THIS NEEDS CLEANING AND UPDATING TO THE NEW FORMAT
void RDATA()
{  
  int targetSS=BOARD_ADS; //  use in Stop Read Continuous mode when DRDY goes low
  byte inByte;     //  to read in one sample of the channels
  csLow(targetSS); //  open SPI
  xfer(_RDATA);    //  send the RDATA command
  for (int i = 0; i < 3; i++)
  { //  read in the status register and new channel data
    inByte = xfer(0x00);
    boardStat = (boardStat << 8) | inByte; //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
  }
  if (targetSS == BOARD_ADS)
  {
    for (int i = 0; i < 8; i++)
    {
      for (int j = 0; j < 3; j++)
      { //  read in the new channel data
        inByte = xfer(0x00);
        ads1299_data_buffer.channel_data[i*3+j] =  inByte;
      }
    }
  }
  csHigh(targetSS); //  close SPI
  delayMicroseconds(10); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}

void IRAM_ATTR ads1299_read_buffer(void* input_buffer, size_t buffer_size)
{
    spiTransferBytesNL(SPI.bus(), NULL, (uint8_t*)input_buffer, buffer_size);
}

void IRAM_ATTR ads1299_write_byte(uint8_t byte_to_write)
{
    spiWriteByteNL(SPI.bus(), byte_to_write);
}

void IRAM_ATTR ads1299_write_buffer(void* output_buffer, size_t buffer_size)
{
    spiWriteNL(SPI.bus(), (uint8_t*)output_buffer, buffer_size);
}

void ads1299_load_registers()
{
    ads1299_write_byte(ads1299_command_rreg);
    ads1299_write_byte(sizeof(ads1299_register_packet) - 1);

    ads1299_read_buffer(&ads1299_register_buffer, sizeof(ads1299_register_packet));
}

void ads1299_flush_registers()
{
    ads1299_write_byte(ads1299_command_wreg);
    ads1299_write_byte(sizeof(ads1299_register_packet) - 1);
  
    ads1299_write_buffer(&ads1299_register_buffer, sizeof(ads1299_register_packet));
}

void ads1299_pwr_up_seq()
{
    delay(40);
    digitalWrite(ADS1299_PIN_RESET, LOW);
    delayMicroseconds(2);
    digitalWrite(ADS1299_PIN_RESET, HIGH);
    delayMicroseconds(12);
}
void IRAM_ATTR ads1299_drdy_interrupt()
{
     RDATA();
    if (streaming_enabled)
    {
        openbci_data_buffer[openbci_data_buffer_tail].header = 0xA0;
        openbci_data_buffer[openbci_data_buffer_tail].sample_number = sample_counter++;
      
        memcpy(&openbci_data_buffer[openbci_data_buffer_tail].channel_data, &ads1299_data_buffer.channel_data, sizeof(ads1299_data_buffer.channel_data));
     
        memset(&openbci_data_buffer[openbci_data_buffer_tail].auxiliary_data, 0x00, sizeof(openbci_data_buffer[openbci_data_buffer_tail].auxiliary_data));
      
        openbci_data_buffer[openbci_data_buffer_tail].footer = 0xC0;

        if (++openbci_data_buffer_tail >= 50) openbci_data_buffer_tail = 0;
    }   
}

size_t get_sampling_rate()
{
    return 16000 >> (ads1299_register_buffer.config1 & 0b111);
}

size_t get_sample_delay()
{
    return 1000000 / get_sampling_rate();
}

size_t gain_from_channel(uint8_t channel_index)
{
    uint8_t gain = (ads1299_register_buffer.chnset[channel_index] >> 4) & 0b111;
    
    switch (gain)
    {
        case 0b000:
          return 1;
        case 0b001:
          return 2;
        case 0b010:
          return 4;
        case 0b011:
          return 6;
        case 0b100:
          return 8;
        case 0b101:
          return 12;
        case 0b110:
          return 24;
        default:
          return 0;
    }
}

IPAddress ip_from_string(String ip_string)
{
    IPAddress ip_address(0, 0, 0, 0);

    ip_address.fromString(ip_string);

    return ip_address;
}

uint8_t digit_from_char(char digit_char)
{
    return digit_char - '0';
}

void get_system_info()
{
    Serial.println("IN get_system_info\n");
    u8g2.u8g2DisplayBottomUp("IN get_system_info");
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    JsonObject json_object = json_document.to<JsonObject>();
  
    json_object["board_connected"] = true;
    json_object["heap"] = ESP.getFreeHeap();
    json_object["ip"] = WiFi.softAPIP().toString();
    json_object["latency"] = wifi_latency;
    json_object["mac"] = WiFi.softAPmacAddress();
    json_object["name"] = OPENBCI_NAME;
    json_object["num_channels"] = 8;
    json_object["version"] = OPENBCI_VERSION;
  
    String json_string = "";
  
    serializeJson(json_document, json_string);
    Serial.println(json_string);
    u8g2.u8g2DisplayBottomUp(json_string);
    
    web_server.send(200, "text/json", json_string);
}

void get_board_info()
{
    Serial.println("IN get_board_info\n");
    u8g2.u8g2DisplayBottomUp("IN get_board_info");
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
  
    JsonObject json_object = json_document.to<JsonObject>();
  
    json_object["board_connected"] = true;
    json_object["board_type"] = "cyton";
    
    JsonArray gains = json_object.createNestedArray("gains");
  
    for (size_t channel_index = 0; channel_index < 8; channel_index++) gains.add(gain_from_channel(channel_index));

    json_object["num_channels"] = 8;
  
    String json_string = "";
  
    serializeJson(json_document, json_string);
    Serial.println(json_string);
    u8g2.u8g2DisplayBottomUp(json_string);
    web_server.send(200, "text/json", json_string);
}

void process_command()
{    
    Serial.println("IN process_command\n");
    u8g2.u8g2DisplayBottomUp("IN process_command");

    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    deserializeJson(json_document, web_server.arg(0));
  
    JsonObject json_object = json_document.as<JsonObject>();

    String command = json_object["command"];

    String return_message = "";

    bool streaming_state = streaming_enabled;
    
    streaming_enabled = false;

    ads1299_write_byte(ads1299_command_sdatac);
    
    delayMicroseconds(50);
    
    if (command[0] == '~')
    {
      Serial.println("command[0]\n");
      u8g2.u8g2DisplayBottomUp("command[0]");
        uint8_t sampling_rate = digit_from_char(command[1]);

        ads1299_register_buffer.config1 &= ~(0b111);
        ads1299_register_buffer.config1 |= sampling_rate;
        
        return_message = "Success: Sample rate is now ";
        return_message += get_sampling_rate();
        return_message += "Hz";
    }
    else if (command == "1") ads1299_register_buffer.chnset[0] = 0b10000001;
    else if (command == "2") ads1299_register_buffer.chnset[1] = 0b10000001;
    else if (command == "3") ads1299_register_buffer.chnset[2] = 0b10000001;
    else if (command == "4") ads1299_register_buffer.chnset[3] = 0b10000001;
    else if (command == "5") ads1299_register_buffer.chnset[4] = 0b10000001;
    else if (command == "6") ads1299_register_buffer.chnset[5] = 0b10000001;
    else if (command == "7") ads1299_register_buffer.chnset[6] = 0b10000001;
    else if (command == "8") ads1299_register_buffer.chnset[7] = 0b10000001;
    else if (command == "!") ads1299_register_buffer.chnset[0] = channel_setting_buffer[0];
    else if (command == "@") ads1299_register_buffer.chnset[1] = channel_setting_buffer[1];
    else if (command == "#") ads1299_register_buffer.chnset[2] = channel_setting_buffer[2];
    else if (command == "$") ads1299_register_buffer.chnset[3] = channel_setting_buffer[3];
    else if (command == "%") ads1299_register_buffer.chnset[4] = channel_setting_buffer[4];
    else if (command == "^") ads1299_register_buffer.chnset[5] = channel_setting_buffer[5];
    else if (command == "&") ads1299_register_buffer.chnset[6] = channel_setting_buffer[6];
    else if (command == "*") ads1299_register_buffer.chnset[7] = channel_setting_buffer[7];
    else if (command[0] == 'x')
    {
      Serial.println("command[0] == 'x'");
      u8g2.u8g2DisplayBottomUp("command[0] == 'x'");
       uint8_t channel_index = digit_from_char(command[1]) - 1;

       uint8_t channel_power_down = digit_from_char(command[2]);
       uint8_t channel_gain = digit_from_char(command[3]);
       uint8_t channel_source = digit_from_char(command[4]);
       uint8_t channel_bias_enabled = digit_from_char(command[5]);
       uint8_t channel_srb2_enabled = digit_from_char(command[6]);
 
       uint8_t channel_setting = (channel_power_down << 7) | (channel_gain << 4) | (channel_srb2_enabled << 3) | channel_source;

       channel_setting_buffer[channel_index] = channel_setting;
       ads1299_register_buffer.chnset[channel_index] = channel_setting;

       ads1299_register_buffer.bias_sensp &= ~(1 << channel_index);
       ads1299_register_buffer.bias_sensp |= (channel_bias_enabled << channel_index);

       ads1299_register_buffer.bias_sensn &= ~(1 << channel_index);
       ads1299_register_buffer.bias_sensn |= (channel_bias_enabled << channel_index);

       uint8_t srb1_enabled = digit_from_char(command[7]);

       ads1299_register_buffer.misc1 &= ~(0b00100000);
       ads1299_register_buffer.misc1 |= (srb1_enabled << 5);
    }
    else if (command == "b") streaming_state = true;
    else if (command == "s") streaming_state = false;

    ads1299_flush_registers();
    
    ads1299_write_byte(ads1299_command_rdatac);
  
    delayMicroseconds(50);

    streaming_enabled = streaming_state;

    web_server.send(200, "text/plain", return_message);
}

void start_streaming()
{
    streaming_enabled = true;
    Serial.println("start_streaming!");
    u8g2.u8g2DisplayBottomUp("start_streaming!");
    web_server.send(200);
}

void stop_streaming()
{
    streaming_enabled = false;
    Serial.println("stop_streaming!");
    u8g2.u8g2DisplayBottomUp("stop_streaming!");
    web_server.send(200);
}

void switch_raw_output()
{
    Serial.println("switch_raw_output!");
    u8g2.u8g2DisplayBottomUp("switch_raw_output!");
    web_server.send(200, "text/plain", "Output mode configured to raw");
}

void get_tcp_config()
{
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    JsonObject json_object = json_document.to<JsonObject>();
    Serial.println("IN get_tcp_config\n");
    json_object["connected"] = (tcp_client.connected() != 0) ? true : false;
    json_object["delimiter"] = false;
    json_object["ip_address"] = tcp_client.remoteIP().toString();
    json_object["output"] = "raw";
    json_object["port"] = tcp_client.remotePort();
    json_object["latency"] = wifi_latency;
  
    String json_string = "";
    serializeJson(json_document, json_string);
    Serial.println(json_string);
    u8g2.u8g2DisplayBottomUp(json_string);
    web_server.send(200, "text/json", json_string);
}

void set_tcp_config()
{
    Serial.println("IN set_tcp_config\n");
    u8g2.u8g2DisplayBottomUp("IN set_tcp_config");
    streaming_enabled = false;
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    deserializeJson(json_document, web_server.arg(0));
  
    JsonObject json_object = json_document.as<JsonObject>();
  
    String tcp_client_ip = json_object["ip"];
    wifi_latency = json_object["latency"];
    uint16_t tcp_client_port = json_object["port"];
    
    tcp_client.stop();
    
    tcp_client.connect(ip_from_string(tcp_client_ip), tcp_client_port);

    tcp_client.setNoDelay(1);
    
    get_tcp_config();
}

void stop_tcp_connection()
{
    streaming_enabled = false;
    Serial.println("IN stop_tcp_connection\n");
    u8g2.u8g2DisplayBottomUp("IN stop_tcp_connection");
    tcp_client.stop();

    get_tcp_config();
}

void invalid_request()
{
    Serial.println("IN invalid_request\n");
    u8g2.u8g2DisplayBottomUp("IN invalid_request");
    web_server.send(404, "text/plain", "Invalid Request!");
}

void setup()
{ 
    Serial.begin(115200);
    /******************************************************************显示屏初始化*******************************************************/
    u8g2.u8g2Init();

    pinMode(ADS1299_PIN_RESET, OUTPUT);
    pinMode(ADS1299_PIN_DRDY, INPUT);
    pinMode(ADS1299_PIN_SS, OUTPUT);
    digitalWrite(ADS1299_PIN_SS, HIGH);

    SPI.begin(ADS1299_PIN_SCK, ADS1299_PIN_MISO, ADS1299_PIN_MOSI, ADS1299_PIN_SS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    delay(500);
    Serial.println("Starting Power up sequence...");
    u8g2.u8g2DisplayBottomUp("Starting Power up sequence...");
    ads1299_pwr_up_seq();
    delay(1000);
    ads1299_pwr_up_seq();
    delay(1000);
    Serial.println("Sequence completed\n");
    u8g2.u8g2DisplayBottomUp("Sequence completed");
    SDATAC(ADS1299_PIN_SS);
    delay(500);
    Serial.println("ADS id:");
    u8g2.u8g2DisplayBottomUp("ADS id:");
    Serial.print(ADS_getDeviceID(BOARD_ADS), HEX); 
    u8g2.u8g2DisplayBottomUp(u8g2.byteToString(ADS_getDeviceID(BOARD_ADS)));
    Serial.println("");
    Serial.println("Start configure!");
    u8g2.u8g2DisplayBottomUp("Start configure!");
    //1
    WREG(CONFIG1, 0x96 );
    //2  
    WREG(CONFIG2, 0xC0 | ADS1299_TEST_INT | ADS1299_TESTSIGNAL_PULSE_SLOW);
    Serial.println("CONFIG2 id:");
    u8g2.u8g2DisplayBottomUp("CONFIG2 id:");
    Serial.print(RREG(CONFIG2, BOARD_ADS), HEX);
    u8g2.u8g2DisplayBottomUp(u8g2.byteToString(ADS_getDeviceID(RREG(CONFIG2, BOARD_ADS))));
    Serial.println("");
    //3
    WREG(CONFIG3, 0x60|(1 << 7) | (1 << 2) | (1 << 3));
    //4
    WREG(CH1SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH2SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH3SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH4SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH5SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH6SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH7SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    WREG(CH8SET, ADS1299_PGA_GAIN12 | ADS1299_INPUT_NORMAL | ADS1299_INPUT_PWR_UP);
    Serial.println("CH8SET id:");
    u8g2.u8g2DisplayBottomUp("CH8SET id:");
    Serial.print(RREG(CH8SET, BOARD_ADS), HEX);
    u8g2.u8g2DisplayBottomUp(u8g2.byteToString(ADS_getDeviceID(RREG(CH8SET, BOARD_ADS))));
    Serial.println("");
    WREG(BIAS_SENSN, 0xFF);
    WREG(BIAS_SENSP, 0xFF);
    
    WREG(LOFF, LOFF_FREQ_FS_4); // ads1299_write_reg(LOFF_SENSP, 0xFF);
    WREG(MISC1, 1 << 5);// Connect SRB1 to all inverting outputs
    
    START(ADS1299_PIN_SS);// ads1299_write_byte(ads1299_command_start);
    delayMicroseconds(10);
    //SDATAC(ADS1299_PIN_SS);
    RDATAC(ADS1299_PIN_SS);//ads1299_write_byte(ads1299_command_rdatac);
    delayMicroseconds(10);
    Serial.println("configuration completed");
    u8g2.u8g2DisplayBottomUp("configuration completed");
    /***********************************************************************显示屏显示*****************************************************************/
    u8g2.setCursor(0, 15);
    u8g2.print("WIFI is connecting ...");
    u8g2.u8g2DisplayBottomUp("WIFI is connecting ...");
    

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SOFT_AP_SSID, SOFT_AP_PASSWORD);
    delay(250); 
    WiFi.softAPConfig(local_ip, network_gateway, subnet_mask); 
    delay(250);
    MDNS.begin("openbci");
    Serial.println("WiFi:tcp_transfer_buffer");
    u8g2.u8g2DisplayBottomUp("WiFi:tcp_transfer_buffer");

    tcp_transfer_buffer = (uint8_t*)malloc(sizeof(openbci_data_buffer));

    web_server.on("/all", HTTP_GET, get_system_info);
    web_server.on("/board", HTTP_GET, get_board_info);
    web_server.on("/command", HTTP_POST, process_command);
    web_server.on("/stream/start", HTTP_GET, start_streaming);
    web_server.on("/stream/stop", HTTP_GET, stop_streaming);
    web_server.on("/output/raw", HTTP_GET, switch_raw_output);    
    web_server.on("/tcp", HTTP_GET, get_tcp_config);
    web_server.on("/tcp", HTTP_POST, set_tcp_config);
    web_server.on("/tcp", HTTP_DELETE, stop_tcp_connection);   
    web_server.onNotFound(invalid_request);     
     
       
    web_server.onNotFound(invalid_request); 
    MDNS.addService("http", "tcp", 80);
    web_server.begin();
    Serial.println("web_server.begin"); 
    u8g2.u8g2DisplayBottomUp("web_server.begin");  
    attachInterrupt(digitalPinToInterrupt(ADS1299_PIN_DRDY), ads1299_drdy_interrupt, FALLING);
    /*----------------------------------------------------------------------------------------------------------------------------------------------------------
    Arduino 中的外部中断配置函数 attachInterrupt(digitalPinToInterrupt(pin),ISR,mode)`包括3个参数:
    1 pin: GPIO端口号;
    2 ISR: 中断服务程序,没有参数与返回值的函数;
    3 mode: 中断触发的方式，支持以下触发方式
        LOW：低电平触发
        HIGH：高电平触发
        RISING:上升沿触发
        FALLING:下降沿触发
        CHANGE:电平变化触发
    -------------------------------------------------------------------------------------------------------------------------------------------------------------*/
}

uint64_t last_micros = 0;

void loop()
{
  
    if (streaming_enabled == true)
    {
        uint64_t current_micros = micros();
        Serial.println("loop.begin"); 
        u8g2.u8g2DisplayBottomUp("loop.begin");  
        size_t tcp_write_size = wifi_latency / get_sample_delay(); //wifi_latency = 0;

        int16_t packets_to_write = openbci_data_buffer_tail - openbci_data_buffer_head; //0-0

        if (packets_to_write < 0) 
          packets_to_write += OPENBCI_DATA_BUFFER_SIZE; //OPENBCI_DATA_BUFFER_SIZE = 50
        
        if ((last_micros + wifi_latency <= current_micros) || (packets_to_write >= tcp_write_size))
        {              
            if (openbci_data_buffer_head + packets_to_write >= OPENBCI_DATA_BUFFER_SIZE)
            { 
               size_t wrap_size = OPENBCI_DATA_BUFFER_SIZE - openbci_data_buffer_head;

               memcpy(tcp_transfer_buffer, &openbci_data_buffer[openbci_data_buffer_head], wrap_size * sizeof(openbci_data_packet));
               memcpy(tcp_transfer_buffer + (wrap_size * sizeof(openbci_data_packet)), &openbci_data_buffer, (packets_to_write - wrap_size) * sizeof(openbci_data_packet));
            }

            else memcpy(tcp_transfer_buffer, &openbci_data_buffer[openbci_data_buffer_head], packets_to_write * sizeof(openbci_data_packet));
            
            tcp_client.write(tcp_transfer_buffer, packets_to_write * sizeof(openbci_data_packet));   

            openbci_data_buffer_head = openbci_data_buffer_tail;

            last_micros = current_micros;
            
        }
        
    }
    
    web_server.handleClient();
    
}
