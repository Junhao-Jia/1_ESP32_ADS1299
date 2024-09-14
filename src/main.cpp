#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

#include <ESPmDNS.h>

#include <ArduinoJson.h>

#include "Definition.h"
#include "ESP32_ADS1299.h"

IPAddress local_ip(192, 168, 4, 1);
IPAddress network_gateway(192, 168, 4, 1);
IPAddress subnet_mask(255, 255, 255, 0);

void setup()
{ 
    Serial.begin(115200);
    pinMode(ADS1299_PIN_RESET, OUTPUT);
    pinMode(ADS1299_PIN_DRDY, INPUT);
    pinMode(ADS1299_PIN_SS, OUTPUT);
    digitalWrite(ADS1299_PIN_SS, HIGH);

    SPI.begin(ADS1299_PIN_SCK, ADS1299_PIN_MISO, ADS1299_PIN_MOSI, ADS1299_PIN_SS);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    delay(500);
    Serial.println("Starting Power up sequence...");
    ads1299_pwr_up_seq();
    delay(1000);
    ads1299_pwr_up_seq();
    delay(1000);
    Serial.println("Sequence completed\n");
    SDATAC(ADS1299_PIN_SS);
    delay(500);
    Serial.println("ADS id:");
    Serial.print(ADS_getDeviceID(BOARD_ADS), HEX); 
    Serial.println("");
    Serial.println("Start configure!");
    //1
    WREG(CONFIG1, 0x96 );
    //2  
    WREG(CONFIG2, 0xC0 | ADS1299_TEST_INT | ADS1299_TESTSIGNAL_PULSE_SLOW);
    Serial.println("CONFIG2 id:");
    Serial.print(RREG(CONFIG2, BOARD_ADS), HEX);
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
    Serial.print(RREG(CH8SET, BOARD_ADS), HEX);
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
    
    Serial.println("Start WIFI");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SOFT_AP_SSID, SOFT_AP_PASSWORD);
    delay(250); 
    WiFi.softAPConfig(local_ip, network_gateway, subnet_mask); 
    delay(250);
    MDNS.begin("openbci");
    Serial.println("WiFi:tcp_transfer_buffer");
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
    attachInterrupt(digitalPinToInterrupt(ADS1299_PIN_DRDY), ads1299_drdy_interrupt, FALLING);

}

uint64_t last_micros = 0;

void loop()
{
  
    if (streaming_enabled == true)
    {
        uint64_t current_micros = micros();
        Serial.println("loop.begin"); 
       
        size_t tcp_write_size = wifi_latency / get_sample_delay();

        int16_t packets_to_write = openbci_data_buffer_tail - openbci_data_buffer_head;

        if (packets_to_write < 0) 
          packets_to_write += OPENBCI_DATA_BUFFER_SIZE;
        
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
