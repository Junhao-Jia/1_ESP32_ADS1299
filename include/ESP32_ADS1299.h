#ifndef ESP32_ADS1299_H
#define ESP32_ADS1299_H

#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

#include <ESPmDNS.h>

#include "Definition.h"
#include <ArduinoJson.h>


enum ads1299_command : uint8_t      //ADS1299控制字
{
    ads1299_command_start = 0x08,   //启动
    ads1299_command_stop = 0x0A,    //停止

    ads1299_command_rdatac = 0x10,    //启用连续读取
    ads1299_command_sdatac = 0x11,    //停止连续读取

    ads1299_command_rreg = 0x20,    //读寄存器        
    ads1299_command_wreg = 0x40     //写寄存器
};

typedef struct ads1299_register_packet  //ADS1299的寄存器包，这个数据结构对应各个寄存器地址
{
    uint8_t id;

    uint8_t config1;
    uint8_t config2;
    uint8_t config3;

    uint8_t loff;

    uint8_t chnset[8];  //八个通道
    
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

typedef struct ads1299_data_packet  //ADS1299采集到的数据
{
    uint32_t stat : 24;

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

extern bool streaming_enabled ;
volatile extern int     boardStat ; //
extern byte    regData[24] ; // array is used to mirror register data
extern ads1299_data_packet ads1299_data_buffer ;
extern ads1299_register_packet ads1299_register_buffer ;

extern openbci_data_packet openbci_data_buffer[OPENBCI_DATA_BUFFER_SIZE] ;//OPENBCI_DATA_BUFFER_SIZE equals 50

extern uint16_t openbci_data_buffer_head ;
extern uint16_t openbci_data_buffer_tail ;
extern uint8_t channel_setting_buffer[8] ;
extern uint8_t sample_counter ;
extern uint8_t* tcp_transfer_buffer ;


extern WebServer web_server;
extern WiFiClient tcp_client;

extern size_t wifi_latency ;


//SPI communication method
byte xfer(byte _data);
void csHigh(int SS);
//SPI chip select method
void csLow(int SS);

byte RREG(byte _address, int targetSS);
void WREG(byte _address, byte _value );

void SDATAC(int targetSS);
void RDATAC(int targetSS);
void START(int targetSS);

byte ADS_getDeviceID(int targetSS);
void IRAM_ATTR ads1299_read_buffer(void* input_buffer, size_t buffer_size);
void RDATA();
void IRAM_ATTR ads1299_write_byte(uint8_t byte_to_write);
void ads1299_load_registers();
void ads1299_flush_registers();
void ads1299_pwr_up_seq();
void IRAM_ATTR ads1299_drdy_interrupt();

size_t get_sampling_rate();
size_t get_sample_delay();
size_t gain_from_channel(uint8_t channel_index);
IPAddress ip_from_string(String ip_string);
uint8_t digit_from_char(char digit_char);
void get_system_info();
void get_board_info();
void process_command();
void start_streaming();
void stop_streaming();
void switch_raw_output();
void get_tcp_config();
void set_tcp_config();
void stop_tcp_connection();
void invalid_request();

#endif 

