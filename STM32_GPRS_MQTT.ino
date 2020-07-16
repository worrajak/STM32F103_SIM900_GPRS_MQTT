#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <TSL2561.h>
#include <mqtt.h>

char atCommand[50];
byte mqttMessage[127];
int mqttMessageLength = 0;
String gsmStr = "";
String gprsStr = "";
char mqttSubject[50];
byte data1;
boolean smsReady = false;
boolean smsSent = false;
boolean gprsReady = false;
boolean mqttSent = false;
int sentCount = 0;

TSL2561 tsl(TSL2561_ADDR_FLOAT); 

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;
int vbat_int;

String response = "";


String buildJson() {
  String data = "{";
  data+="\n";
  data+= "\"d\": {";
  data+="\n";
  data+="\"myName\": \"Agrinode_01\",";
  data+="\n";
  data+="\"lux\": ";
  data+=lux_int;
  data+= ",";
  data+="\n";
  
  data+="\"temperature\": ";
  data+=temperature;
  data+= ",";
  data+="\n";

  data+="\"humidity\": ";
  data+=humidity;
  data+= ",";
  data+="\n";
    
  data+="\"battery\": ";
  data+=(float)vbat_int*0.01;
  data+="\n";
  data+="}";
  data+="\n";
  data+="}";
  return data;
}

void readData(){  
 
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    lux_int = tsl.calculateLux(full, ir);

    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    tempC_int = temperature*10;
    hum_int = humidity*2;

   //temperature = 25.5;
   //humidity = 85.0;
   adc_enable(ADC1);
   vbat_int = 120 * 4096 / adc_read(ADC1, 17);
   adc_disable(ADC1);
   float vbat_f = vbat_int*0.01;  

   String json = buildJson();
   char jsonStr[300];
   json.toCharArray(jsonStr,300);

   sendMQTTMessage("agrinode", "34.87.25.24", "1883", "agrinode01",jsonStr);
   
   Serial.println();
}

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

void setup() {

setup_vdd_sensor();
pinMode(PB5, OUTPUT);
Serial.begin(9600);
delay(3000);
Serial.println("Starting.....");
Serial2.begin(19200);
delay(3000);
digitalWrite(PB5, LOW);

  if (tsl.begin()) {
    Serial.println("Found TLS2561 sensor");
  } else {
    Serial.println("No TLS2561 sensor?");
    digitalWrite(PB5, HIGH);
    while (1);
  }  

  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)  
  
  bool status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        digitalWrite(PB5, HIGH);
        while (1);
    }

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);

  sendCommand("AT+CPIN?\r\n");
  delay(2000);
  sendCommand("AT+CREG?\r\n");
  delay(2000);
  sendCommand("AT+CGATT?\r\n");
  delay(2000);    
  sendCommand("AT+CSQ\r\n");
  delay(2000);  
  sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
  delay(2000);
  sendCommand("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n");
  delay(2000);       
  sendCommand("AT+SAPBR=1,1\r\n");
  delay(2000);
  sendCommand("AT+SAPBR=2,1\r\n");
  delay(2000);    
  Serial.println(F("Leave setup"));
}


void loop() {
    readData(); 
    delay(30000);
}

void sendCommand(String atComm){
response = "";
Serial2.print(atComm);
  while(Serial2.available()){
    char ch = Serial2.read();
    response += ch;
  }
  Serial.println(response);
  oled.clear();
  oled.println(response);  
}

void sendMQTTMessage(char* clientId, char* brokerUrl, char* brokerPort, char* topic, char* message){
  sendCommand("AT\r\n");
  delay(1000); // Wait a second
  sendCommand("AT+CSTT=\"m-wap\",\"mms\",\"mms\"\r\n"); // Puts phone into GPRS mode
  delay(2000); // Wait a second
  sendCommand("AT+CIICR\r\n");
  delay(2000);
  sendCommand("AT+CIFSR\r\n");
  delay(2000);
  strcpy(atCommand, "AT+CIPSTART=\"TCP\",\"");
  strcat(atCommand, brokerUrl);
  strcat(atCommand, "\",\"");
  strcat(atCommand, brokerPort);
  strcat(atCommand, "\"\r\n");
  sendCommand(atCommand);
  delay(2000);
  sendCommand("AT+CIPSEND\r\n");
  delay(2000);
  mqttMessageLength = 16 + strlen(clientId);
  //Serial.println(mqttMessageLength);
  mqtt_connect_message(mqttMessage, clientId);
  for (int j = 0; j < mqttMessageLength; j++) {
    Serial2.write(mqttMessage[j]); // Message contents
    //Serial.write(mqttMessage[j]); // Message contents
    //Serial.println("");
  }
    Serial2.write(byte(26)); // (signals end of message)
    //Serial.println("Sent");
    delay(10000);
 sendCommand("AT+CIPSEND\r\n");
 delay(2000);
 mqttMessageLength = 4 + strlen(topic) + strlen(message);
 mqtt_publish_message(mqttMessage, topic, message);
 for (int k = 0; k < mqttMessageLength; k++) {
  Serial2.write(mqttMessage[k]);
 }
  Serial2.write(byte(26)); // (signals end of message)
  Serial.println("-------------Sent-------------"); // Message contents
  delay(5000);
 sendCommand("AT+CIPCLOSE");
 Serial.println("AT+CIPCLOSE");
 delay(2000);
}
