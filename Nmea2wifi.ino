/*
x Signal K json /API
x multi UDP
x Show last 20 NMEA sentences page

https://github.com/jcable/nmea-link

*/

#include "FS.h"

#include <ESP8266WiFi.h>            // https://github.com/esp8266/Arduino
#include <WiFiUDP.h>

// Needed for Wifi Manager library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
//mDNS Dont seem to work in soft AP Mode
#include <ESP8266mDNS.h>            // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS

// NMEA sentence parser
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPS https://github.com/mikalhart/TinyGPSPlus

// json builder
#include <ArduinoJson.h>

// Time lib
#include <TimeLib.h>                // http://playground.arduino.cc/Code/Time
char gpsTimestamp_buffer[24] = "1970-01-01T00:00:01.00";       //json format
char currentTimestamp_buffer[24] = "1970-01-01T00:00:01.00"; 

// SoftSerial for NMEA input (or for the debug....)
#include <SoftwareSerial.h>

#include <Metro.h>

#include <ArduinoOTA.h>

// Defines
#define OTA 1                       // Use OverTheAir Updates
#define MAX_SRV_CLIENTS 5           // How many clients should be to connect simultainiuslys
#define NORMAL_WIFI false           // Wifimanager normal start
#define RESET_WIFI true             // Wifimanager reset stored config
#define NMEA_MAX_LENGTH 200         // Max charaters in a NMEA sentence
#define MAX_UDP_BUFFER 512
#define WEBPORT 80
#define HOSTNAME "nmea2wifi"
#define LED LED_BUILTIN
#define MAX_LOG_FILE_SIZE 5000
#define NMEAPORT Serial             //#define NMEAPORT nmeaSerial
#define DEBUGPORT Serial

const char compile_date[] = __DATE__ " " __TIME__;

const char ssid[32] = HOSTNAME;         // SSID and Hostname for device
const char password[32] = "admin";      // OTA password
const int boatMMSI = 1234567;
const char boatName[32] = "Aida";
boolean wifiConnected = false;          // Global wifi connection state

unsigned int UDPPort = 5050;
unsigned int TCPPort = 5051;

IPAddress UDPremoteip;
unsigned int UDPremoteport;

const int rxPin = 12;                   // Serial receive Pin D6 = GPIO12
const int txPin = 13;                   // Serial transmit Pin D7 = GPIO13

int nmeaBaud = 9600;                    // Detected baud rate

char nmeaBuffer[128];                   // NMEA byte/char buffer to hold a full collected nmea sentence max 82 chars..
int nmeaBufferIndex = 0;
int nmeaStatus = 0;                     // We are reading a nmea sentence $ until *xx 0=notreading,1=reading_complete

//Enables reading the ESP8266 supply voltage
ADC_MODE(ADC_VCC);                  

//Start a configurable timer
Metro timer = Metro(0); 

//GPS nmea parser instance
TinyGPSPlus gps;
TinyGPSCustom ais(gps, "AIVDM", 5); // !AIVDM sentence, 6th element

// Software serial actual NMEA sentences read
SoftwareSerial nmeaSerial(txPin, rxPin, false, 256);      // tx,rx, invert, buffersize

// TCP instance to let us send and receive packets over TCP
WiFiServer nmeaTCPServer(TCPPort);
WiFiClient nmeaTCPServerClients[MAX_SRV_CLIENTS];

// UDP instance to let us send and receive packets over UDP
WiFiUDP nmeaUDPServer;

// Webserver instance
ESP8266WebServer webServer(WEBPORT);
  
// put your setup code here, to run once:
void setup() {
  
  pinMode(LED, OUTPUT);   
  digitalWrite(LED, LOW);                                       //Turn on led in setup mode
  
  Serial.begin(115200);

  Serial.println("");
  Serial.println(compile_date);
  Serial.printf("Sketch size: %u\n\r", ESP.getSketchSize());
  Serial.printf("Free size: %u\n\r", ESP.getFreeSketchSpace());

  // Setup SP filesystem
  SPIFFS.begin();                                             //TODO : Check for error                                       
  showSPIFFS();

  //readConfig();

  // Auto detect baudrate
  nmeaBaud = determineBaudRate(rxPin);                        // Try to detect BAUD on rxPIN
  DEBUGPORT.print("Detected Baudrate:"); DEBUGPORT.println(nmeaBaud);
  nmeaSerial.begin(nmeaBaud?nmeaBaud:9600);                   // Start on detected baud or set to 9600
  nmeaSerial.write("nmeaSerial started");                     // Debug output on softserial port
  
#if OTA
  initializeOTA();
#endif
  
  // Initialise wifi connection 
  wifiConnected = initWifi(NORMAL_WIFI);     //Blocking until some WIFI is setup
  DEBUGPORT.println("connected...yeey :)");

  // Initialise mDNS for WEB and NMEA TCP/UDP 
  initMDNS(ssid);

  if( initWebHandlers() ){
    DEBUGPORT.print("Webserver started on port:"); DEBUGPORT.println(WEBPORT);  
  }
  else{
    DEBUGPORT.print("Failed to start Webserver");
  }


  //Setup TCP server
  nmeaTCPServer.begin();                    //TODO:<-- Check success
  nmeaTCPServer.setNoDelay(true);
  Serial.print("TCP server started on port:"); Serial.println(TCPPort);
  
  //Setup UDP server
  nmeaUDPServer.begin(UDPPort);              //TODO:<-- Check success
  Serial.print("UDP server started on port:"); Serial.println(UDPPort);

  Serial.print(F("Free Heap: "));
  Serial.println(ESP.getFreeHeap());
    
  digitalWrite(LED, HIGH);                                 //Turn off led when setup is done
} // End setup

// Main loop
//TODO: Check that wifi is still connected, webserver is running and both UDP/TCP is listening
//Maybe use a timer (Ticker) to check every minute?

void loop() {
 
#if OTA
    ArduinoOTA.handle();                              // take care of OTA updates 
#endif

  // Take care of incoming web requests
  webServer.handleClient();      

  // Read from Serial and parse out NMEA messages into the Buffer
  // and send chars to nmea encoder
  nmeaBufferIndex = readNMEASerial( nmeaBuffer, nmeaBufferIndex, &nmeaStatus);
    
  handleTCP(nmeaBuffer, nmeaStatus);
  handleUDP(nmeaBuffer, nmeaStatus);

  //Use the string
  if ( nmeaStatus ){                                // We have a full string so use it now
    ledBlink(100);                                  // Start a 100ms blink
    DEBUGPORT.print("\n\rSentense: "); DEBUGPORT.println(nmeaBuffer);
    log2file(nmeaBuffer);                           // Safe the string to the /nmea.log file 

    StaticJsonBuffer<500> jsonBuffer;
    JsonObject &json = prepareResponse(jsonBuffer);
    json.prettyPrintTo(DEBUGPORT);

    //showParsedNmea();                                 //Print some parsed NMEA stats from encoder
    //char buffer[256];
    //json.printTo(buffer, sizeof(buffer));

    if(timeStatus()!=timeSet)gpsSetTime();

    timeToString(currentTimestamp_buffer, now());
    DEBUGPORT.println(currentTimestamp_buffer);
    DEBUGPORT.println(gps.time.age());
    
    nmeaStatus = 0;                                     // String consumed -> reset
  }
  
  ledBlink(0);
    
} // End loop()

JsonObject& prepareResponse(JsonBuffer &jsonBuffer) {

  JsonObject &root = jsonBuffer.createObject();

  JsonObject &vessels = root.createNestedObject("vessels");
  JsonObject &vessel = vessels.createNestedObject("self"); 
  vessel["mmsi"] = boatMMSI;
  vessel["name"] = boatName;  
  
  if (gps.date.isValid()){
      gpsDateToString(gpsTimestamp_buffer);
      gpsTimestamp_buffer[10]='T';           //Overwrite null char ;-)
  }
  if(gps.time.isValid()){
      gpsTimeToString(&gpsTimestamp_buffer[11]);
  }
  vessel["timestamp"] = gpsTimestamp_buffer;

  JsonObject &navigation = vessel.createNestedObject("navigation");
  if ( gps.location.isUpdated() && gps.location.isValid() ){
    JsonObject &position = navigation.createNestedObject("position");
    position.set("longitude",gps.location.lng(),6);
    position.set("latitude",gps.location.lat(),6);
  }
  
  return root;
}

int gpsTimeToString(char *buffer){
  return sprintf( buffer,"%02d:%02d:%02d.%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond() );
}

int gpsDateToString(char *buffer){
  return sprintf( buffer,"%4d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day() );
}

int timeToString(char *buffer, time_t t){
  if(sizeof(buffer) > 20)return 0;
  return sprintf( buffer,"%4d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t) );
}

void gpsSetTime(){
  if( gps.time.isValid() && gps.date.isValid() ){ //
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year() );
    //adjustTime(gps.time.age()/1000);
  }
  
}

//--------------------------------------------------------

/*
  $xxMTW,20.33,C*21
  $SDMTW,26.8,C*45
  $SDDPT,3.6,0.0*52
  $SDDBT,1330.5,f,0405.5,M,0221.6,F*2E
  $WIMWV,214.8,R,0.1,K,A*28

  !AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C
  
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  $GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C
  $GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62
  $GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77
  $GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C
  $GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D
  $GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F
*/
//--------------------------------------------------------
void ledBlink(int start){
  if(start){
    timer.interval(start);
    timer.reset();
    digitalWrite(LED,LOW);
  }
  if(timer.check()){
    digitalWrite(LED,HIGH);
    timer.interval(0);
  }  
}

int readNMEASerial( char *buffer, int index, int *status ){
  char c;                                              // temp for incoming serial char
  if( NMEAPORT.available() ){                          // we have data in the UART buffer

    c = NMEAPORT.read();                               // read one char from the UART buffer
    DEBUGPORT.write(c);                                // Local echo of what we got
    gps.encode(c);                                     // Send to encoder

    if (c == '$' || c == '!' || index && c != '\r'){    // Start char or started to read?
      buffer[index++] = c;                              // Fill the buffer with chars
      *status = 0;
            
      if (buffer[index-3]=='*'){                        // Have we read the * and the two chksum chars?
        buffer[index] = '\0';                           // Terminate the string
        index = 0;                                      // Then stop adding chars to the buffer
        *status = 1;                                     // Done with a string

        DEBUGPORT.print(" chksum:");DEBUGPORT.printf(" %x", (unsigned char)calcNmeaChecksum(buffer) );
      }
      
      if ( index > NMEA_MAX_LENGTH )index=0;             // Have we passed max, the startover         
    }
    else {
      index=0;  
    }

  }                                                      // END serial available
  return index;
}

int calcNmeaChecksum(char *nmeaMessageBuffer){
  int parity = 0, i=1; //skip the $
  while(nmeaMessageBuffer[i]!='*'){
      parity ^= nmeaMessageBuffer[i++];
  }
  return parity;
}

boolean handleTCP(char *nmeaMessageBuffer, int nmeaStatus){
//
// TCP Handling 
//    
  if ( nmeaTCPServer.hasClient() ){                                             // Do we have ANY clients connected?
    //DEBUGPORT.println("TCP: client connected");
    ledBlink(250);
    for( uint8_t i = 0; i < MAX_SRV_CLIENTS; i++){                              // Loop over the client array
      if (!nmeaTCPServerClients[i] || !nmeaTCPServerClients[i].connected() ){   // Is the array spot taken or client no longer connected?
        if(nmeaTCPServerClients[i])
          nmeaTCPServerClients[i].stop();                                       // If there is a clients and not connected, stop it!
        nmeaTCPServerClients[i] = nmeaTCPServer.available();                    // Found a free spot, store the new client here (only if client sent any data!!) replace with connected()?
        continue;                                                               // Break the for loop
      }
    }
    //Reached end of for loop. no free/disconnected spots. Reject additional clients
    //WiFiClient tempClient = nmeaTCPServer.available();                          // temp client just to say stop
    //tempClient.stop();
    nmeaTCPServer.available().stop();
  }  // No TCP clients connected
  
  //push NMEA data to all connected TCP clients
  if(nmeaStatus==1){                                                               //Do we have a full NMEA sentence?
    uint8_t i;
    for(i = 0; i < MAX_SRV_CLIENTS; i++){                                          //Loop over all clients
      if (nmeaTCPServerClients[i] && nmeaTCPServerClients[i].connected()){         //Only worry about connected clients
        nmeaTCPServerClients[i].println(nmeaMessageBuffer);                        //Send the string to the client        
        delay(1);                                                                  //Helthy delay to finish the job
        DEBUGPORT.print("TCP: NMEA sentence pushed to client#"); Serial.println(i);    //Log it on console         
      }
    }
  }

  //check TCP clients for incoming data - What should we use if for???
  uint8_t i = 0;
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (nmeaTCPServerClients[i] && nmeaTCPServerClients[i].connected()){
      if(nmeaTCPServerClients[i].available()){
        while(nmeaTCPServerClients[i].available())                                  // Get data from the TCP client and push it to Serial
          Serial.write(nmeaTCPServerClients[i].read());                             // Dump it to console for now  
        Serial.print("\r\n");
      }
    }
  }
  // End TCP Handling  
  return true;
}

boolean handleUDP(char *nmeaMessageBuffer, int nmeaStatus){
//
// UDP handling
//
// TODO: Multiple UDP clients handeling (store IP and Port in array)
byte UDPpacketBuffer[MAX_UDP_BUFFER];           // Buffer to hold incoming
int noBytes = nmeaUDPServer.parsePacket();

  if ( noBytes && noBytes<MAX_UDP_BUFFER ) {
    Serial.print("UDP: ");
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" bytes received from ");
    Serial.print(nmeaUDPServer.remoteIP());
    Serial.print(":");
    Serial.println(nmeaUDPServer.remotePort());

    nmeaUDPServer.read(UDPpacketBuffer, noBytes);                                 //Read the packet into the buffer
    Serial.write(UDPpacketBuffer, noBytes);                                       //Dump it to console for now
    Serial.print("\r");
    
    UDPremoteip = nmeaUDPServer.remoteIP();                                          //Single UDP client for now
    UDPremoteport = nmeaUDPServer.remotePort();                                      //TODO: Keep a arry of x UDP clients, and loop
     
    //response back to UDP sender
    nmeaUDPServer.beginPacket(nmeaUDPServer.remoteIP(), nmeaUDPServer.remotePort());
    nmeaUDPServer.write("Answer from ESP8266 ChipID#");
    nmeaUDPServer.print(system_get_chip_id());
    nmeaUDPServer.write("#IP of ESP8266#");
    nmeaUDPServer.println(WiFi.localIP());
    nmeaUDPServer.endPacket(); //
  }


  if(nmeaStatus && UDPremoteip){                                                 //Do we have a full NMEA sentence and a receiver?
    nmeaUDPServer.beginPacket(UDPremoteip, UDPremoteport);                       //nmeaUDPServer.beginPacketMulticast(IPAddress('239.0.0.36'), 5052, WiFi.localIP()); or broadcast .255
    nmeaUDPServer.print(nmeaMessageBuffer);
    nmeaUDPServer.endPacket();
    Serial.println("NMEA String pushed to UDP client");                          //Log it on console  
  }
   // End UDP Handling
  return true;
}

/*
WiFiManager
1. try connecting to last used wifi network, if fails try next
2. try captive portal to configure new network, if fails try next (5 minutes timeout)
3. Setup AP mode as last resort
*/
boolean initWifi(boolean rst){

  boolean state = true;
    
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  if(rst){
    //reset settings - for testing
    wifiManager.resetSettings();
  }
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep 180 in seconds
  wifiManager.setTimeout(60);  //5 minutes 300
  
  //fetches ssid and pass from storage and tries to connect
  //if it does not connect it starts an access point with the specified name
  //if autoConnect fails it start in real AP mode
  if(!wifiManager.autoConnect(ssid)) {
    Serial.println("failed to connect and hit timeout");
    state = false;
    delay(3000);

    //Everything failed so start a soft AP
    if(WiFi.softAP(ssid)){                      //ssid, No password
      state = true;
      
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("AP IP address: "); Serial.println(myIP);
    }
  }  

  WiFi.printDiag(Serial);
  return state;
}  //End initWifi

// function to return valid received baud rate
// Note that the serial monitor has no 600 baud option and 300 baud
// doesn't seem to work with version 22 hardware serial library
long determineBaudRate(int recpin) {
  long baud, rate = 10000, x;
  for (int i = 0; i < 10; i++) {
    x = pulseIn(recpin,LOW,200);        // measure the next zero bit width. 200ms (defalut 1sec timeout.)
    if (x > 0)
      rate = x < rate ? x : rate;   //min();
  }
  if (rate < 12)
    baud = 115200;
    else if (rate < 20)
    baud = 57600;
    else if (rate < 29)
    baud = 38400;
    else if (rate < 40)
    baud = 28800;
    else if (rate < 60)
    baud = 19200;
    else if (rate < 80)
    baud = 14400;
    else if (rate < 150)
    baud = 9600;
    else if (rate < 300)
    baud = 4800;
    else if (rate < 600)
    baud = 2400;
    else if (rate < 1200)
    baud = 1200;
    else
    baud = 0;  
  return baud;
}

void showSPIFFS(){
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {    
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    DEBUGPORT.printf("FS File: %s, size: %s\n\r", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  DEBUGPORT.printf("\n");
}

bool initMDNS( const char *name ){
  delay(5);
  //Setup and Register mDNS
  if ( MDNS.begin(name) ) {
    Serial.print("*mDNS: Responder STARTED. Hostname -> "); Serial.println(name);

    // Register the services
    MDNS.addService("http", "tcp", WEBPORT);            // Web server - discomment if you need this
    MDNS.addService("nmea", "tcp", TCPPort);            // NMEA server
    MDNS.addService("nmea", "udp", UDPPort);            // NMEA server
    MDNS.addService("signalk-http", "tcp", WEBPORT);    // Signal K server  _signalk-http._tcp
    
  }
  else {
    Serial.println("*mDNS: Responder FAILED");
  }  
}

bool showParsedNmea(){

  if( ais.isUpdated() ){
    decodeAIS( ais.value() );
  }

  if ( gps.location.isUpdated() && gps.location.isValid() ){
    Serial.print( F("Location FixAge=") );
    Serial.print( gps.location.age() );
    Serial.print( F(" Lat=") );
    Serial.print( gps.location.lat(), 6 );
    Serial.print( F(" Long=") );
    Serial.println( gps.location.lng(), 6 );
   } 

  if (gps.date.isUpdated() && gps.date.isValid()){
    Serial.print(F("Date: "));
    Serial.print(gps.date.day());
    Serial.print(F("-"));
    Serial.print(gps.date.month());
    Serial.print(F("-"));
    Serial.println(gps.date.year());
  }

  if (gps.time.isUpdated() && gps.time.isValid()){
    Serial.print(F("Time: "));
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }

  if (gps.speed.isUpdated()){
   Serial.print(F("Knots="));
   Serial.println(gps.speed.knots());
  }

  if (gps.course.isUpdated()){
    Serial.print(F("Cource Deg="));
    Serial.println(gps.course.deg());
  }

  if (gps.temp.isUpdated()){
    Serial.print(F("Temp Celcius="));
    Serial.print( gps.temp.celcius() ); 
  }

  if (gps.dept.isUpdated()){
    Serial.print(F("Dept Meters="));
    Serial.print( gps.dept.meters() ); 
  }

  //Serial.print("CHARS="); Serial.println( gps.charsProcessed() );                        
  //Serial.print("SENTENCES="); Serial.println( gps.sentencesWithFix() );
  //Serial.print("CSUM ERR="); Serial.println( gps.failedChecksum() );
}

//Log complete NMEA messages to SPIFFS
int log2file( const char *log ){
  int size;
  
  File f = SPIFFS.open("/nmea.log", "a");
  
  if (!f) {
    DEBUGPORT.println("Failed to create/open file for logging");
  } 
  else {   
    f.println(log);
  }
  
  size = f.size();
  f.close(); 

  if(size > MAX_LOG_FILE_SIZE){
    SPIFFS.remove("/nmea.log");
    DEBUGPORT.println("Rotating log file");  
  }

  return size;
}
#if OTA
void initializeOTA() {
    ArduinoOTA.setHostname(ssid);                               // Use ssid as hostname
    ArduinoOTA.setPassword(password);                           // OTA PASSWORD

    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    ArduinoOTA.onStart([]() {
        DEBUGPORT.println("* OTA: Start");
    });
    ArduinoOTA.onEnd([]() {
        DEBUGPORT.println("\n*OTA: End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        DEBUGPORT.printf("*OTA: Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        DEBUGPORT.printf("*OTA: Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) DEBUGPORT.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) DEBUGPORT.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) DEBUGPORT.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) DEBUGPORT.println("Receive Failed");
        else if (error == OTA_END_ERROR) DEBUGPORT.println("End Failed");
    });
    ArduinoOTA.begin();
    DEBUGPORT.println("*OTA: Enabled");
}
#endif

