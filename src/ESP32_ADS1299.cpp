
#include "ESP32_ADS1299.h"

bool streaming_enabled = false;
volatile int     boardStat = 0; //
byte    regData[24] = {0}; // array is used to mirror register data
ads1299_data_packet ads1299_data_buffer = {};
ads1299_register_packet ads1299_register_buffer = {};

openbci_data_packet openbci_data_buffer[OPENBCI_DATA_BUFFER_SIZE] = {{}};//OPENBCI_DATA_BUFFER_SIZE equals 50

uint16_t openbci_data_buffer_head = 0;
uint16_t openbci_data_buffer_tail = 0;
uint8_t channel_setting_buffer[8] = {0};
uint8_t sample_counter = 0;
uint8_t* tcp_transfer_buffer = NULL;

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

void IRAM_ATTR ads1299_read_buffer(void* input_buffer, size_t buffer_size)
{
    spiTransferBytesNL(SPI.bus(), NULL, (uint8_t*)input_buffer, buffer_size);
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
    return 16000 >> (ads1299_register_buffer.config1 & 0b111);//Config1寄存器低三位：000 -> fMOD / 64，001 -> fMOD / 128
}

size_t get_sample_delay()
{
    return 1000000 / get_sampling_rate();
}

size_t gain_from_channel(uint8_t channel_index)
{
    uint8_t gain = (ads1299_register_buffer.chnset[channel_index] >> 4) & 0b111;//右移四位并去掉最高位，读到PGA增益值
    
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
    
    web_server.send(200, "text/json", json_string);
}

void get_board_info()
{
    Serial.println("IN get_board_info\n");
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
    web_server.send(200, "text/json", json_string);
}

void process_command()//从接收json，提取指令，对ADS1299进行操控
{    
    Serial.println("IN process_command\n");
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    //将 web_server.arg(0)中的数据进行解析，将解析后的结果存储到json_document中
    deserializeJson(json_document, web_server.arg(0));
  
    JsonObject json_object = json_document.as<JsonObject>();

    String command = json_object["command"];//把JSOn中的command提取出来

    String return_message = "";

    bool streaming_state = streaming_enabled;//记录先前状态
    
    streaming_enabled = false;//暂时关闭流

    ads1299_write_byte(ads1299_command_sdatac);//停止连续读取
    
    delayMicroseconds(50);
    
    if (command[0] == '~')
    {
      Serial.println("command[0]\n");
        uint8_t sampling_rate = digit_from_char(command[1]);//采样率来自command1提取采样频率

        ads1299_register_buffer.config1 &= ~(0b111);//清空config1的采样频率位（低三位）
        ads1299_register_buffer.config1 |= sampling_rate;//按位或写入采样频率
        
        return_message = "Success: Sample rate is now ";
        return_message += get_sampling_rate();
        return_message += "Hz";
    }
    else if (command == "1") ads1299_register_buffer.chnset[0] = 0b10000001;//关闭通道，PGA增益为1，与SRB2连接，配置通道1为输入短路（用于偏移量或噪声测量）
    else if (command == "2") ads1299_register_buffer.chnset[1] = 0b10000001;//同上
    else if (command == "3") ads1299_register_buffer.chnset[2] = 0b10000001;
    else if (command == "4") ads1299_register_buffer.chnset[3] = 0b10000001;
    else if (command == "5") ads1299_register_buffer.chnset[4] = 0b10000001;
    else if (command == "6") ads1299_register_buffer.chnset[5] = 0b10000001;
    else if (command == "7") ads1299_register_buffer.chnset[6] = 0b10000001;
    else if (command == "8") ads1299_register_buffer.chnset[7] = 0b10000001;
    else if (command == "!") ads1299_register_buffer.chnset[0] = channel_setting_buffer[0];//使用单片机里缓存的配置
    else if (command == "@") ads1299_register_buffer.chnset[1] = channel_setting_buffer[1];
    else if (command == "#") ads1299_register_buffer.chnset[2] = channel_setting_buffer[2];
    else if (command == "$") ads1299_register_buffer.chnset[3] = channel_setting_buffer[3];
    else if (command == "%") ads1299_register_buffer.chnset[4] = channel_setting_buffer[4];
    else if (command == "^") ads1299_register_buffer.chnset[5] = channel_setting_buffer[5];
    else if (command == "&") ads1299_register_buffer.chnset[6] = channel_setting_buffer[6];
    else if (command == "*") ads1299_register_buffer.chnset[7] = channel_setting_buffer[7];
    else if (command[0] == 'x')//提取其后的索引、掉电值、增益设置、源、偏置使能等
    {
      Serial.println("command[0] == 'x'");
       uint8_t channel_index = digit_from_char(command[1]) - 1;

       uint8_t channel_power_down = digit_from_char(command[2]);
       uint8_t channel_gain = digit_from_char(command[3]);
       uint8_t channel_source = digit_from_char(command[4]);
       uint8_t channel_bias_enabled = digit_from_char(command[5]);
       uint8_t channel_srb2_enabled = digit_from_char(command[6]);
 
       uint8_t channel_setting = (channel_power_down << 7) | (channel_gain << 4) | (channel_srb2_enabled << 3) | channel_source;

       channel_setting_buffer[channel_index] = channel_setting;
       ads1299_register_buffer.chnset[channel_index] = channel_setting;//写入寄存器

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
    web_server.send(200);
}

void stop_streaming()
{
    streaming_enabled = false;
    Serial.println("stop_streaming!");
    web_server.send(200);
}

void switch_raw_output()
{
    Serial.println("switch_raw_output!");
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
    web_server.send(200, "text/json", json_string);
}

void set_tcp_config()
{
    Serial.println("IN set_tcp_config\n");
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
    tcp_client.stop();

    get_tcp_config();
}

void invalid_request()
{
    Serial.println("IN invalid_request\n");
    web_server.send(404, "text/plain", "Invalid Request!");
}