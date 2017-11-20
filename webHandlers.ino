/*
 * Intit Web handlders for different paths/URLs
*/
File fsUploadFile;

// Setup Web handlers
int initWebHandlers(){

  //Handle root + index.htm
  webServer.on("/", [](){                             
    if( !handleFileRead("/") )
      webServer.send(404, "text/plain", "FileNotFound");
  });
  webServer.on("/nmea", handleNMEA);
  webServer.on("/log", handleLog);
  webServer.on("/api", HTTP_GET, handleAPI);
  webServer.on("/reset", handleReset);

  //Setup FS Browser handlers
  webServer.on("/list", HTTP_GET, handleFileList);                  //list directory
  webServer.on("/edit", HTTP_GET, [](){                             //load editor
    if(!handleFileRead("/edit.htm")) webServer.send(404, "text/plain", "FileNotFound");
  });
  webServer.on("/edit", HTTP_PUT, handleFileCreate);                 //create file
  webServer.on("/edit", HTTP_DELETE, handleFileDelete);              //delete file
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  webServer.on("/edit", HTTP_POST, [](){ webServer.send(200, "text/plain", ""); }, handleFileUpload);
  
  //get heap status, analog input value and all GPIO statuses in one json call
  webServer.on("/all", HTTP_GET, [](){
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += "}";
    webServer.send(200, "text/json", json);
    json = String();
  });
  
  //called when the url is not defined here
  //use it to load content from SPIFFS
  webServer.onNotFound([](){
    if(!handleFileRead(webServer.uri()))
      webServer.send(404, "text/plain", "FileNotFound");
  });

  webServer.begin();                         //TODO:<-- Check success
  return true;
}

/* DEBUG WEBSERVER
for ( uint8_t i = 0; i < webServer.args(); i++ ) {
  page += " " + webServer.argName ( i ) + ": " + webServer.arg ( i ) + "\n";
}
*/

/*
 * Handler functions
*/

// Webserver NMEA handlder
void handleNMEA() {
  String page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "NMEA2WIFI");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  //page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("<dl>");
  page += F("<dt>GPS status</dt>");
  page += gps.sentencesWithFix();
  
  page += F("<dd>");
  page += F("Lng/Lat:");
  page += F("</dd>");      

  page += F("</dl>");
  page += FPSTR(HTTP_END);
  webServer.send(200, "text/html", page); 
  page = String();  //release 
}

// Webserver NMEA message log handlder
void handleLog() {
  String page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "NMEA2WIFI");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  //page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("<dl>");
  page += F("<dt>NMEA messages</dt>");
  //page += gps.sentencesWithFix();
  
  File f = SPIFFS.open("/nmea.log", "r");
  if (!f) {
    page += F("*LOG/WEB file open failed");
  } 
  else {
    while(f.available()) {
      page += F("<dd>");
      page += f.readStringUntil('\n');
      page += F("</dd>");      
    }    
  }
  f.close();
  
  page += F("</dl>");
  page += FPSTR(HTTP_END);
  webServer.send(200, "text/html", page); 
  page = String();  //release 
}

// Webservice API handlder
void handleAPI() {
  char buffer[500];
  StaticJsonBuffer<500> jsonBuffer;
  //JsonObject &json = prepareAPI(jsonBuffer);
  JsonObject &json = prepareLog(jsonBuffer);
  json.printTo(buffer, 500);
  webServer.send(200, "text/json", buffer );
}

// Webserver reset handlder
void handleReset() {
  String page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Info");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  //page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("Module will reset in a few seconds.");
  page += FPSTR(HTTP_END);
  webServer.send(200, "text/html", page);

  delay(5000);
  //reset the ESP and reconfigure wifi
  initWifi(RESET_WIFI);
  ESP.reset();
  delay(2000);
}

// SPIFF Browser functions
bool handleFileRead(String path){
  //DEBUGPORT.println("*WEB handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = webServer.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

// Upload handler
// for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done 
void handleFileUpload(){
  if(webServer.uri() != "/edit") return;
  HTTPUpload& upload = webServer.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    DEBUGPORT.print("*WEB handleFileUpload Name: "); DEBUGPORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    DEBUGPORT.print("*WEB handleFileUpload Size: "); DEBUGPORT.println(upload.totalSize);
  }
}

// Delete a file from SPIFF
void handleFileDelete(){
  if(webServer.args() == 0) 
    return webServer.send(500, "text/plain", "BAD ARGS");
  String path = webServer.arg(0);
  DEBUGPORT.println("*WEB handleFileDelete: " + path);
  if(path == "/")
    return webServer.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return webServer.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  webServer.send(200, "text/plain", "");
  path = String();
}

// Create a new file if it does not exist
void handleFileCreate(){
  if(webServer.args() == 0)
    return webServer.send(500, "text/plain", "BAD ARGS");
  String path = webServer.arg(0);
  DEBUGPORT.println("*WEB handleFileCreate: " + path);
  if(path == "/")
    return webServer.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return webServer.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return webServer.send(500, "text/plain", "CREATE FAILED");
  webServer.send(200, "text/plain", "");
  path = String();
}

// Directory of files in SPIFF
void handleFileList() {
  if(!webServer.hasArg("dir")) {webServer.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = webServer.arg("dir");
  DEBUGPORT.println("*WEB handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  webServer.send(200, "text/json", output);
}

/*
 * Helper functions
*/

// Format size string
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

// Return type of content string from filename
String getContentType(String filename){
  if(webServer.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

