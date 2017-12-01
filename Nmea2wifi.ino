/*
x aisData Array
x add aisData array to json
x Parse API and connect to gauge
x DONE Signal K json /API
x multi UDP
x DONE Show last 20 NMEA sentences page

https://github.com/jcable/nmea-link

*/

#include "FS.h"
#include "Nmea2wifi.h"

#include <ESP8266WiFi.h>            // https://github.com/esp8266/Arduino
#include <WiFiUDP.h>

#include <WebSocketsServer.h>		// WebSockets for live updates

// Needed for Wifi Manager library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
//mDNS Dont seem to work in soft AP Mode
#include <ESP8266mDNS.h>            // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS


// NMEA sentence parser
#include "TinyGPS++.h"              // https://github.com/mikalhart/TinyGPS https://github.com/mikalhart/TinyGPSPlus

// json builder
#include <ArduinoJson.h>

// Time lib
#include <TimeLib.h>                // http://playground.arduino.cc/Code/Time

// SoftSerial for NMEA input (or for the debug....)
#include <SoftwareSerial.h>

// Timer for LED blinks
#include <Metro.h>

// OTA stuff
#include <ArduinoOTA.h>

// Defines
#define         OTA               1                 // Use OverTheAir Updates
#define         MAX_SRV_CLIENTS   5                 // How many clients should be to connect simultainiuslys
#define         NORMAL_WIFI       false             // Wifimanager normal start
#define         RESET_WIFI        true              // Wifimanager reset stored config
#define         NMEA_MAX_LENGTH   200               // Max charaters in a NMEA sentence
#define         MAX_UDP_BUFFER    512
#define         HOSTNAME          "nmeawifi"
#define         LED               LED_BUILTIN
#define         MAX_LOG_FILE_SIZE 5000
#define         NMEAPORT          Serial            // #define NMEAPORT nmeaSerial
#define         DEBUGPORT         Serial
#define         MAXSTR            32


//Enables reading the ESP8266 supply voltage
ADC_MODE(ADC_VCC);

const char		compile_date[]  = __DATE__ " " __TIME__;

char			ssid[MAXSTR]    = HOSTNAME;         // SSID and Hostname for device
char			password[MAXSTR]= "admin";          // OTA password
char			boatName[MAXSTR]= "Aida";
unsigned int	boatMMSI        = 1234567;
boolean			wifiConnected   = false;            // Global wifi connection state

unsigned int	UDPPort         = 5050;
unsigned int	TCPPort         = 5051;
unsigned int	WEBPort         = 80;
unsigned int	WEBSock         = 81;

IPAddress		UDPremoteip;
unsigned int	UDPremoteport;

int				rxPin           = 20;                 // Serial receive Pin D1 = GPIO5 = ArduinoPIN 20
int				txPin           = 19;                 // Serial transmit Pin D2 = GPIO4 = ArduinoPIN 19

int				nmeaBaud        = 0;                  // Detected baud rate
int				nmeaBaudDefault = 9600;               // Default baud rate

char			nmeaBuffer[128];                      // NMEA byte/char buffer to hold a full collected nmea sentence max 82 chars..
int				nmeaBufferIndex = 0;
int				nmeaStatus      = 0;                  // We are reading a nmea sentence $ until *xx 0=notreading,1=reading_complete

char			gpsTimestamp_buffer[24]      = "1970-01-01T00:00:01.00";       // For json output.
char			currentTimestamp_buffer[24]  = "1970-01-01T00:00:01.00"; 
					  
// Start a configurable timer
Metro timer = Metro(0); 

// GPS nmea parser instance
TinyGPSPlus gps;
TinyGPSCustom ais(gps, "AIVDM", 5);                   // Special AIS !AIVDM sentence, 6th element

// Software serial actual NMEA sentences read   
SoftwareSerial *nmeaSerial = NULL;                    // Pointer to hold the SoftwareSerial object         

// TCP instance to let us send and receive packets over TCP
WiFiServer nmeaTCPServer(TCPPort);                    //
WiFiServer *nmeaTCPServer2 = NULL;
WiFiClient nmeaTCPServerClients[MAX_SRV_CLIENTS];

// UDP instance to let us send and receive packets over UDP
WiFiUDP nmeaUDPServer;

// Webserver instance
ESP8266WebServer webServer(WEBPort);
ESP8266WebServer *webServer2 = NULL;

// Web Sockets Server
WebSocketsServer webSocket = WebSocketsServer(WEBSock);

/*
* Set up system and configurations
* SPIFF (Flash file system)
* Wifi
* Software Serial
* OTA
* TCP and UDP
* Webserver
*/
void setup() {

	pinMode(LED, OUTPUT);   
	digitalWrite(LED, LOW);                                       //Turn on led in setup mode
  
	Serial.begin(115200);

	Serial.println("");
	Serial.println(compile_date);
	Serial.printf("Sketch size: %u\r\n", ESP.getSketchSize());
	Serial.printf("Free size: %u\r\n", ESP.getFreeSketchSpace());
	Serial.printf("TinyGPS++ %s\r\n", gps.libraryVersion() );

	// Mount SPIFFS and print content
	mountSPIFFS();

	//Init logfile
	log2file("# bootup");

	//Read config from SPIFF file
	readConfig("/config.json");

	// Setup SoftwareSerialport
	nmeaSerial = new SoftwareSerial(txPin, rxPin, false, 256);   // tx,rx, invert, buffersize
	nmeaBaud = determineBaudRate(rxPin);                         // Try to detect BAUD on rxPIN
	if(nmeaSerial)DEBUGPORT.println(F("SoftSerial init"));
	nmeaSerial->begin(nmeaBaud ? nmeaBaud : nmeaBaudDefault);    // Start on detected baud or set to 9600
	Serial.print(F("Detected Baudrate:")); Serial.println(nmeaBaud);
	Serial.print(F("Using Baudrate:")); Serial.println(nmeaBaud ? nmeaBaud : nmeaBaudDefault);
	nmeaSerial->write("nmeaSerial started");                     // Debug output on softserial port

#if OTA
	initializeOTA();
#endif

	// Initialise wifi connection 
	wifiConnected = initWifi(NORMAL_WIFI);     //Blocking until some WIFI is setup
	Serial.println("*WM: connected...");

	// Initialise mDNS for WEB and NMEA TCP/UDP 
	initMDNS(ssid);

	if( initWebHandlers() ){
		Serial.print(F("*WEB Server started on port:")); DEBUGPORT.println(WEBPort);  
	}
	else{
		Serial.print(F("*WEB Failed to start Webserver"));
	}

	// Setup WebSockets and add listner function
	webSocket.begin();
	webSocket.onEvent(webSocketEvent);
	Serial.print(F("*WebSocket server started on port:")); Serial.println(WEBSock);

	//Setup TCP server
	nmeaTCPServer.begin();                    //TODO:<-- Check success
	nmeaTCPServer.setNoDelay(true);
	Serial.print(F("*TCP server started on port:")); Serial.println(TCPPort);
  
	//Setup UDP server
	nmeaUDPServer.begin(UDPPort);              //TODO:<-- Check success
	Serial.print(F("*UDP server started on port:")); Serial.println(UDPPort);

	Serial.print(F("Free Heap: ")); Serial.println(ESP.getFreeHeap());

	digitalWrite(LED, HIGH);                                 //Turn off led when setup is done
} // End setup

/*
* Main loop
* =========
* Handle processes : OTA upgrade, Webserver, TCP & UDP connections
* Log captured NMEA sentences to plain log file on SPIFF
* Log decoded NMEA to JSON file on SPIFF
*
* TODO: Add decoded NMEA to memeory log
* TODO: Check that wifi is still connected, webserver is running and both UDP/TCP is listening 
* Maybe use a timer (Ticker) to check every minute?
*/
void loop() {

#if OTA
	ArduinoOTA.handle();                              // take care of OTA updates 
#endif

	// Take care of incoming web requests
	webServer.handleClient();      

	// Read from Serial and parse out NMEA messages into the Buffer
	// and send chars to nmea encoder
	nmeaBufferIndex = readNMEASerial( nmeaBuffer, nmeaBufferIndex, nmeaStatus);

	// Send out valid NMEA string to connected TCP and UDP clients, take care of incomming data  
	handleTCP(nmeaBuffer, nmeaStatus);
	handleUDP(nmeaBuffer, nmeaStatus);

	// Use the string
	if ( nmeaStatus ){											// We have a full string so use it now
		ledBlink(100);												// Start a 100ms blink
		//DEBUGPORT.print("\r\nNMEA Sentense: "); DEBUGPORT.println(nmeaBuffer);

		log2file(nmeaBuffer);									// Save the string to the /nmea.log file 

		StaticJsonBuffer<300> jsonBuffer;					// Allocate 300 on the heap for buffer
		JsonObject &json = nmeaToJSON(jsonBuffer);
		json.printTo(DEBUGPORT);								// Log to serial debug
		//DEBUGPORT.print("\r\njsonBuffer size: "); DEBUGPORT.println(jsonBuffer.size());		// Show used buffer size

		size_t size = json.measureLength() + 1; 			// json serialized len+1
		char jsonString[size];
		json.printTo(jsonString, size); 						// writes string to buffer
		//DEBUGPORT.print("\r\njsonString size: "); DEBUGPORT.println(size);						// Show string length
		log2file(jsonString);									// 
		
		//DEBUGPORT.print("\r\nCurrent Time: "); DEBUGPORT.println(currentTimeToString());

		//webSocket.sendTXT(0, "C");
		webSocket.sendTXT(0, jsonString);

		nmeaStatus = 0;											// String consumed -> reset
	}

	if(timeStatus() == timeNotSet)gpsSetTime();			// Set system time if not set or if sync is needed

	webSocket.loop();									// Check webSockets Events

	ledBlink(0);													// Handle LED blinker
	 
} // End loop()

/*
* Create a JSON string from current data to write to datalog log
* {timestamp:"asd",postition:}
*/
JsonObject& nmeaToJSON(JsonBuffer &jsonBuffer) {

	JsonObject &root = jsonBuffer.createObject();

	//TODO : zero Timestamp buffer before use
	if (gps.date.isValid()){
		gpsDateToString(gpsTimestamp_buffer);
		gpsTimestamp_buffer[10]='T';						//Overwrite null char ;-)
	}
	if (gps.time.isValid()){
		gpsTimeToString(&gpsTimestamp_buffer[11]);
	}
	root["gpstime"] = gpsTimestamp_buffer;				// time and date from NMEA data for this event
	currentTimeToString();									// Update time
	root["currenttime"] = currentTimestamp_buffer;	// current clock time even if not set
	root["millis"]=millis();								// millis since bootup

	if ( gps.location.isUpdated() && gps.location.isValid() ){
		JsonArray &position = root.createNestedArray("position");
		position.add(gps.location.lng());
		position.add(gps.location.lat());
	}

	if (gps.speed.isUpdated() && gps.speed.isValid()){
		root["speed"] = gps.speed.knots();
	}

	if (gps.course.isUpdated() && gps.course.isValid()){
		root["cog"] = gps.course.deg();
	}

	if (gps.temp.isUpdated() && gps.temp.isValid()){
		root["temp"] = gps.temp.celcius();
	}

	if (gps.dept.isUpdated() && gps.dept.isValid()){
		root["dept"] = gps.dept.meters();
	}

	return root;
}


/*
* Signal K representation of current NMEA and AIS data
*/
JsonObject& prepareAPI(JsonBuffer &jsonBuffer) {

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
  
  if ( gps.location.isValid() ){
	 JsonObject &position = navigation.createNestedObject("position");
	 position.set("longitude",gps.location.lng(),6);
	 position.set("latitude",gps.location.lat(),6);
	 position.set("age",gps.location.age());
  }

  if (gps.speed.isValid()){
	 JsonObject &position = navigation.createNestedObject("speedThroughWater");
	 position.set("knots",gps.speed.knots(),6);
	 position.set("age",gps.speed.age());
  }

  if (gps.course.isValid()){
	 JsonObject &position = navigation.createNestedObject("courseOverGroundTrue");
	 position.set("degrees",gps.course.deg(),6);
	 position.set("age",gps.speed.age());
  }

  if (gps.temp.isValid()){
	 JsonObject &position = navigation.createNestedObject("waterTemp");
	 position.set("celcius",gps.temp.celcius(),6);
	 position.set("age",gps.temp.age());
  }
  
  if (gps.dept.isValid()){
	 JsonObject &position = navigation.createNestedObject("deptBelowKeel");
	 position.set("meter",gps.dept.meters(),6);
	 position.set("age",gps.dept.age());
  }

  return root;
}

void gpsSetTime(){
  if( gps.time.isValid() && gps.date.isValid() ){   //TODO : Needs to separate setting Time & Date
	 setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year() );
	 adjustTime(gps.time.age()/1000);
  }
}

int gpsTimeToString(char *buffer){
  return sprintf( buffer,"%02d:%02d:%02d.%02d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond() );
}

int gpsDateToString(char *buffer){
  return sprintf( buffer,"%4d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day() );
}

int timeToString(char *buffer, time_t t){
  return sprintf( buffer,"%4d-%02d-%02dT%02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t) );
}

char* currentTimeToString(){
  timeToString( currentTimestamp_buffer, now() );
  return currentTimestamp_buffer;
}


//--------------------------------------------------------

/*
$xxMTW,20.33,C*21
$SDMTW,26.8,C*45
$SDDPT,3.6,0.0*52
$SDDBT,1330.5,f,0405.5,M,0221.6,F*31
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

int readNMEASerial( char *buffer, int index, int &status ){
  char c;                                              // temp for incoming serial char
  if( NMEAPORT.available() ){                          // we have data in the UART buffer

	 c = NMEAPORT.read();                               // read one char from the UART buffer
	 //DEBUGPORT.write(c);                                // Local echo of what we got
	 gps.encode(c);                                     // Send to encoder
	  
	 if (c == '$' || c == '!' || index ){                // Start char or started to read? && c != '\n'
		buffer[index++] = c;                              // Fill the buffer with chars
		status = 0;
				
		if (buffer[index-4]=='*'){                        // Have we read the * and the two chksum chars and newline should be -3?
		  buffer[index] = '\0';                           // Terminate the string
		  index = 0;                                      // Then stop adding chars to the buffer
		  status = 1;                                     // Done with a string

		  //DEBUGPORT.print("chksum:");DEBUGPORT.printf(" %x\n", (unsigned char)calcNmeaChecksum(buffer) );
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
  int parity = 0, i=1; //skip the $ or !
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
	 //DEBUGPORT.println("*TCP client connected");
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
		  DEBUGPORT.print("*TCP NMEA sentence pushed to client#"); Serial.println(i);    //Log it on console         
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
	 nmeaUDPServer.beginPacket(UDPremoteip, UDPremoteport);                       
	 //nmeaUDPServer.beginPacketMulticast(IPAddress('239.0.0.36'), 5052, WiFi.localIP()); or broadcast .255
	 nmeaUDPServer.print(nmeaMessageBuffer);
	 nmeaUDPServer.endPacket();
	 Serial.println("NMEA String pushed to UDP client");                          //Log it on console  
  }
	// End UDP Handling
  return true;
}

/*
* Web Socket Event handler
*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t lenght) { // When a WebSocket message is received
	switch (type) {
		case WStype_DISCONNECTED:             // if the websocket is disconnected
			DEBUGPORT.printf("[%u] Disconnected!\n", num);
			break;
		case WStype_CONNECTED: {              // if a new websocket connection is established
			IPAddress ip = webSocket.remoteIP(num);
			DEBUGPORT.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
			}
			break;
		case WStype_TEXT: {                    // if new text data is received
			DEBUGPORT.printf("[%u] get Text: %s\n", num, payload);
			// send Response message to client
			// webSocket.sendTXT(num, "Tack!");
			//if (payload[0] == '#') {            // we get RGB data
			//	break;
			//}
			}
			break;
	}
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

  //WiFi.printDiag(DEBUGPORT);
  return state;
}  //End initWifi

/*
 * function to return valid received baud rate
 * Note that the serial monitor has no 600 baud option and 300 baud
 * doesn't seem to work with version 22 hardware serial library
*/
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

// Setup SP filesystem
void mountSPIFFS(){
	if(SPIFFS.begin() ){
		showSPIFFS();
	}
	else {
		DEBUGPORT.println(F("Mount SPIFFS failed, trying to format"));
		SPIFFS.format();
		//TODO : Fetch one index.htm that can fetch the rest of the data files from github
	}
}

void showSPIFFS(){
	FSInfo fs_info;
	SPIFFS.info(fs_info);

	DEBUGPORT.printf("Total bytes: %s, Used bytes: %s\r\n", formatBytes(fs_info.totalBytes).c_str(), formatBytes(fs_info.usedBytes).c_str());

	Dir dir = SPIFFS.openDir("/");
	while (dir.next()) {    
		String fileName = dir.fileName();
		size_t fileSize = dir.fileSize();
		DEBUGPORT.printf("FS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
	}
	DEBUGPORT.printf("\n");
}

/*
 * Read a config.json file in json format from SPIFF
 * parse the key/values and update variables
*/
void readConfig( const char *filename ){
  StaticJsonBuffer<400> jsonBuffer;
  
  File f = SPIFFS.open(filename, "r");

  if (!f) {
	 DEBUGPORT.println(F("cfg* file open failed"));
	 return; 
  } 
  else{
	 JsonObject& root = jsonBuffer.parseObject(f);
	 
	 if (!root.success()) {
		DEBUGPORT.println(F("cfg* parse config failed"));
		return;
	 }
  
	 //root.prettyPrintTo(DEBUGPORT);
	 
	 if( root.containsKey("tcpPort") )
		TCPPort = root["tcpPort"];

	 if( root.containsKey("udpPort") )
		UDPPort = root["udpPort"];
		  
	 if( root.containsKey("webPort") )
		WEBPort = root["webPort"];

	 if( root.containsKey("webSock") )
		WEBSock = root["webSock"];

	 if( root.containsKey("txPin") )
		txPin = root["txPin"];

	 if( root.containsKey("rxPin") )
		rxPin = root["rxPin"];
	 
	 if( root.containsKey("hostname") ){
		strncpy(ssid, root["hostname"], MAXSTR);
	 }
	 
	 if( root.containsKey("boatName") ){
		strncpy(boatName, root["boatName"] , MAXSTR);
	 }

	 if( root.containsKey("boatMMSI") ){
		boatMMSI  = root["boatMMSI"];
	 }

	 if( root.containsKey("otaPassword") ){
		strncpy(password, root["otaPassword"], MAXSTR);
	 }
	 
	 if( root.containsKey("baudRate") ){
		nmeaBaud = root["baudRate"];
	 }	 
  }
}

/*
 * Init discovery mDNS discoveryprotocol
*/
bool initMDNS( const char *hostname ){
	delay(5);
	// Setup and Register mDNS
	if ( MDNS.begin(hostname) ) {
		Serial.print("*mDNS: Responder STARTED. Hostname -> "); Serial.println(hostname);

		// Register services
		MDNS.addService("http", "tcp", WEBPort);            // Web server - discomment if you need this
		MDNS.addService("nmea", "tcp", TCPPort);            // NMEA server
		MDNS.addService("nmea", "udp", UDPPort);            // NMEA server
		MDNS.addService("signalk-http", "tcp", WEBPort);    // Signal K server  _signalk-http._tcp
	}
	else {
		Serial.print("*mDNS: Responder FAILED");
	}  
}

/*
 * Debug output from NMEA and AIS messages
*/
bool showParsedNmea(){
  aisdata aisData; 
  
  if( ais.isUpdated() && ais.isValid() ){
	 if( decodeAIS( ais.value(), aisData ) ){
		Serial.print(F("*AIS msg type="));Serial.println(aisData.type);
		Serial.print(F("*AIS MMSI="));Serial.println(aisData.mmsi);
		Serial.print(F("*AIS Navigation status="));Serial.println(aisData.navigationStatus);
		Serial.print(F("*AIS Turnrate="));Serial.println(aisData.RateOfTurn);
		Serial.print(F("*AIS SOG="));Serial.println(aisData.sog);
		Serial.print(F("*AIS Longitude="));Serial.println(aisData.longitude,6);
		Serial.print(F("*AIS Latitude="));Serial.println(aisData.latitude,6);
		Serial.print(F("*AIS COG="));Serial.println(aisData.cog);
		Serial.print(F("*AIS True heading="));Serial.println(aisData.trueHeading); 
		Serial.print(F("*AIS UTC Second="));Serial.println(aisData.UTCSeconds); 
	 }
	 else{
		Serial.print( F("*AIS Error: Could not decode message") );
	 }
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

  Serial.print("CHARS="); Serial.println( gps.charsProcessed() );                        
  Serial.print("SENTENCES="); Serial.println( gps.sentencesWithFix() );
  Serial.print("CSUM ERR="); Serial.println( gps.failedChecksum() );

  return true;
}

/*
 * Log complete nmea messages to file log
*/
int log2file( const char *log ){
  int size;
  
  File f = SPIFFS.open("/nmea.log", "a");
  
  if (!f) {
	 DEBUGPORT.println("*LOG Failed to create/open file for logging");
  } 
  else {   
	 f.println(log);
  }
  
  size = f.size();
  f.close(); 

  if(size > MAX_LOG_FILE_SIZE){
	 SPIFFS.remove("/nmea.log");
	 DEBUGPORT.println(F("*LOG Rotating log file"));  
  }

  return size;
}

/*
 * OTA Update of firmware functions
*/
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
