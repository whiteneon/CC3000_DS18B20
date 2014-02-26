/*************************************************** 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
/*  This sketch attempts to re-connect to an AP using
    data from a previous SmartConfig session (i.e. from
    the SmartConfigCreate sketch!).

    It will attempt to do the following:

    * Initialization the CC3000
    * Re-connect using previous SmartConfig data (see SmartConfigCreate)
    * DHCP printout
    * Disconnect

    If the connection fails, there are two likely
    explanations:

    1.) You haven't run the SmartConfigCreate sketch and
        successfully connected to a network, since this
        is the process that 'saves' the AP details to the
        device
    2.) You've used one of the other non SmartConfig
        sketches, which erase all stored profiles from
        the CC3000 memory in order to manually establish
        a connection using hard coded values. (This sketch
        uses an optional flag in the Adafruit_CC3000.begin
        function to avoid this erasure process!).

    SmartConfig is still beta and kind of works but is not fully
    vetted! It might not work on all networks!
*/
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

#define WEBSITE      "http://leerubin.net63.net/"
#define WEBPAGE      "/temp.php?currentTemp=92.3"
uint32_t ip;
#define IDLE_TIMEOUT_MS  3000

/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(5, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  // Optional SSID scan
  // listSSIDResults();
  
  /* Optional setting a static IP Address  */

//unsigned long IPAdd[1] = {0x3202A8C0};
//unsigned long SubNetMask[1] = {0x00FFFFFF};
//unsigned long dfGW[1] = {0x0102A8C0};
unsigned long DNSServer[1] = {0x08080808};


unsigned long IPAdd[4] = {0x00};
unsigned long SubNetMask[4] = {0x00};
unsigned long dfGW[4] = {0x00};
//unsigned long DNSServer[4] = {0x00};


    if (!cc3000.setStaticIPAddress(IPAdd, SubNetMask, dfGW, DNSServer))
    {
      Serial.println(F("Failed to set static IP"));
      while(true);
    }
    

/* End setting static IP Address*/


  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  // Optional: Do a ping test on the website
  /*
  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");  
  replies = cc3000.ping(ip, 5);
  Serial.print(replies); Serial.println(F(" replies"));
  */  

  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint(WEBPAGE);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
  } else {
    Serial.println(F("Connection failed"));    
    return;
  }

  Serial.println(F("-------------------------------------"));
  
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis();
  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      Serial.print(c);
      lastRead = millis();
    }
  }
  www.close();
  Serial.println(F("-------------------------------------"));
  
  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time your try to connect ... */
  Serial.println(F("\n\nDisconnecting"));
  cc3000.disconnect();
  
}

void loop(void)
{
  float t = temp();
  if (t != 0) {
    Serial.println(t);
    int i = 0;
    for (i = 0; i < 20; i++) {
      digitalWrite(5, HIGH);
      delay(30);
      digitalWrite(5, LOW);
      delay(30);
    }
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
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    return 0;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
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
  if (0) {
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
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
}
*/
