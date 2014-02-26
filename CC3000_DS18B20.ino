
#include <OneWire.h>
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

OneWire  ds(A5);
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  9
#define ADAFRUIT_CC3000_CS    10

// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, 
                                         ADAFRUIT_CC3000_IRQ, 
                                         ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2);

// We get the SSID & Password from memory thanks to SmartConfigCreate!

#define WEBSITE      "leerubin.net63.net"
#define WEBPAGE      "/temp.php?currentTemp="
uint32_t ip;
float lastTemp = 0;
int completedFirstSubmission = 0;
#define IDLE_TIMEOUT_MS  3000

void setup(void) {
  pinMode(5, OUTPUT);
  Serial.begin(38400);
}
/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/
void sendTemp(float z)
{
  if (!cc3000.begin(false, true))
  {
    if (!cc3000.begin(false)) {
      //Unable to initialize Module....
      blink_times (4);
      while(1);
    }
    if (!cc3000.startSmartConfig(false)) {
      blink_times (3);
      while(1);
    }
  }

  /* Round of applause! */
  Serial.println(F("Reconnected!"));
  
  /* Wait for DHCP to complete */
  while (!cc3000.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }
  /*while (! displayConnectionDetails()) {
    delay(1000);
  }*/
  blink_times (2);
  
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("no resolve!"));
    }
    delay(500);
  }
  //cc3000.printIPdotsRev(ip);
  
    Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    
    //int j = 0;
    //j = (int)z;
    //String w = String(j);
    //Serial.println("*****************");
    //Serial.println(w);
    //dtostrf(z,'2','2',w);
    //sprintf(w, "?currentTemp=%f", z);
    www.fastrprint(F("GET "));
    www.fastrprint(WEBPAGE);
    //www.fastrprint(j);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
  } else {
    blink_times(7);   
    return;
  }

  delay(3000);
  
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  /*unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      //Serial.print(c);
      lastRead = millis();
    }
  }*/
  www.close();
  
  
  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time your try to connect ... */
  //Serial.println(F("\nClosing the connection"));
    cc3000.disconnect();
}

void loop(void)
{
  int i = 0;
  float t = temp();
  
  if (t != 0) {
    float a = 0;
    a = abs(t - lastTemp);
    if ((a > 2) || (completedFirstSubmission == 0)) {
      //2 degree change.....send to server.
      completedFirstSubmission = 1;
      lastTemp = t;
      sendTemp(t);
      for (i = 0; i < 20; i++) {
        digitalWrite(5, HIGH);
        delay(30);
        digitalWrite(5, LOW);
        delay(30);
      }
    }
    Serial.println(t);
    Serial.print("lastTemp="); Serial.println(lastTemp);
    Serial.print("completedFirstSubmission="); Serial.println(completedFirstSubmission);
  }
}

void blink_times(int i) 
{
  for(int n = 0; n < i; n++) {
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
    delay(500);
  }
  delay(1000);
}

float temp(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  //byte addr[8];
  float celsius, fahrenheit;
  byte addr[8] = {0x28, 0xB3, 0x00, 0x90, 0x04, 0x00, 0x00, 0x3F};
  /* if ( !ds.search(addr)) {
    Serial.println("No temp sensor found!");
    ds.reset_search();
    delay(250);
    return 0;
  }
  for (i = 0;i < 9; i++) {
    Serial.println(addr[i], HEX);
  }
  */
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(1000);
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  //ds.reset_search();
  return fahrenheit;
}
/*
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.println(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.println(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.println(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.println(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.println(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    return true;
  }
}*/
