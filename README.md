# nmea2wifi

WeMos D1 pro (ESP8266) thingie that reads NMEA signals from a (Garmin) plotter and push them to TCP or UDP listeners. Automatic wifi network creation or (re)join existing wifi. Can also parse AIS in NMEA sentences. Log NMEA and show current and historical values on a web service or as graphs on web pages.

Combination of libraries

* TinyGPS++               // https://github.com/mikalhart/TinyGPS https://github.com/mikalhart/TinyGPSPlus
* ArduinoJson             // https://arduinojson.org
* WiFiManager             // https://github.com/tzapu/WiFiManager
* ESP8266WiFi             // https://github.com/esp8266/Arduino
* ESP8266WebServer        // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer
* ESP8266mDNS             // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS
* TimeLib                 // http://playground.arduino.cc/Code/Time

