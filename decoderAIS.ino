/*
* Decode AIS data
*/


/*
// Takes a full NMEA String and try to find AIS payload
int AISdecoder(String msg){
  
  // find !AIVDM or !AIVDO and Trim string
  String ais_string=msg.substring(msg.indexOf("!AIVD"),msg.indexOf("*")+3);

  // find 6bit ais encoded string, 5th element is payload
  String six_bit=getValue(ais_string, ',' ,5);

  aisdata a;
  return decodeAIS(six_bit, a);
}
*/

int decodeAIS(String six_bit, aisdata &aisData ) {
  
  // six bit ascii table. (gpsd.berlios.de/AIVDM)
  String six_bit_table[120];
  six_bit_table[48]="000000";
  six_bit_table[49]="000001";
  six_bit_table[50]="000010";
  six_bit_table[51]="000011";
  six_bit_table[52]="000100";
  six_bit_table[53]="000101";
  six_bit_table[54]="000110";
  six_bit_table[55]="000111";
  six_bit_table[56]="001000";
  six_bit_table[57]="001001";
  six_bit_table[58]="001010";
  six_bit_table[59]="001011";
  six_bit_table[60]="001100";
  six_bit_table[61]="001101";
  six_bit_table[62]="001110";
  six_bit_table[63]="001111";
  six_bit_table[64]="010000";
  six_bit_table[65]="010001";
  six_bit_table[66]="010010";
  six_bit_table[67]="010011";
  six_bit_table[68]="010100";
  six_bit_table[69]="010101";
  six_bit_table[70]="010110";
  six_bit_table[71]="010111";
  six_bit_table[72]="011000";
  six_bit_table[73]="011001";
  six_bit_table[74]="011010";
  six_bit_table[75]="011011";
  six_bit_table[76]="011100";
  six_bit_table[77]="011101";
  six_bit_table[78]="011110";
  six_bit_table[79]="011111";
  six_bit_table[80]="100000";
  six_bit_table[81]="100001";
  six_bit_table[82]="100010";
  six_bit_table[83]="100011";
  six_bit_table[84]="100100";
  six_bit_table[85]="100101";
  six_bit_table[86]="100110";
  six_bit_table[87]="100111";
 // 88-95 not used in encoding
  six_bit_table[96]="101000";
  six_bit_table[97]="101001";
  six_bit_table[98]="101010";
  six_bit_table[99]="101011";
  six_bit_table[100]="101100";
  six_bit_table[101]="101101";
  six_bit_table[102]="101110";
  six_bit_table[103]="101111";
  six_bit_table[104]="110000";
  six_bit_table[105]="110001";
  six_bit_table[106]="110010";
  six_bit_table[107]="110011";
  six_bit_table[108]="110100";
  six_bit_table[109]="110101";
  six_bit_table[110]="110110";
  six_bit_table[111]="110111";
  six_bit_table[112]="111000";
  six_bit_table[113]="111001";
  six_bit_table[114]="111010";
  six_bit_table[115]="111011";
  six_bit_table[116]="111100";
  six_bit_table[117]="111101";
  six_bit_table[118]="111110";
  six_bit_table[119]="111111";

  // convert 6 bit string to binary String
  String ais_binary="";
  for (int x=0; x<six_bit.length(); x++) {
    ais_binary += six_bit_table[ six_bit[x] ];
  }

  String temp_s="";

  temp_s=ais_binary.substring(0,6);
  aisData.type=bin_to_int(temp_s);
  if( aisData.type>3 )return 0;  //Can only decode type 1,2,3
    
  temp_s=ais_binary.substring(6,6+2);
  int ais_repeat_indicator=bin_to_int(temp_s);
    
  temp_s=ais_binary.substring(8,8+30);
  aisData.mmsi=bin_to_int(temp_s);
    
  temp_s=ais_binary.substring(38,38+4);
  aisData.navigationStatus=bin_to_int(temp_s);
  
  temp_s=ais_binary.substring(42,42+8);
  aisData.RateOfTurn=bin_to_int(temp_s);
  
  temp_s=ais_binary.substring(50,50+10);
  aisData.sog=bin_to_int(temp_s)/10;

  String ais_position_accuracy=ais_binary.substring(60,60+1);
  
  aisData.longitude = bin_to_I4( ais_binary.substring(61,61+28) )/60.0/10000;
  aisData.latitude = bin_to_I4( ais_binary.substring(89,89+27) )/60.0/10000;

  temp_s=ais_binary.substring(116,116+12);
  //3600==N/A
  aisData.cog=bin_to_int(temp_s)/10;
   
  temp_s=ais_binary.substring(128,128+9);
  //511 ==N/A
  aisData.trueHeading=bin_to_int(temp_s);

  temp_s=ais_binary.substring(137,137+6);
  aisData.UTCSeconds=bin_to_int(temp_s);

  /*
  Serial.print("*AIS binary string=");Serial.println(ais_binary);
  Serial.print("*AIS string length=");Serial.println(ais_binary.length());
  Serial.println("*AIS --------------");
  Serial.print("*AIS msg type=");Serial.println(aisData.type);
  Serial.print("*AIS Repeat indicator=");Serial.println(ais_repeat_indicator);
  Serial.print("*AIS MMSI=");Serial.println(aisData.mmsi);
  Serial.print("*AIS Navigation status=");Serial.println(aisData.navigationStatus);
  Serial.print("*AIS Turnrate=");Serial.println(aisData.RateOfTurn);
  Serial.print("*AIS SOG=");Serial.println(aisData.sog);
  Serial.print("*AIS Position accuracy=");Serial.println(ais_position_accuracy);
  Serial.print("*AIS Longitude=");Serial.println(aisData.longitude,6);
  Serial.print("*AIS Latitude=");Serial.println(aisData.latitude,6);
  Serial.print("*AIS COG=");Serial.println(aisData.cog);
  Serial.print("*AIS True heading=");Serial.println(aisData.trueHeading); 
  Serial.print("*AIS UTC Second=");Serial.println(aisData.UTCSeconds); 
  */
  return 1;
}

// Get the Index'th Value from a delimter string using Separator
String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Convert a string with a ascii 0 and 1 to a base-10 number
int bin_to_int(String temp_s){
  int x,i=0;
  for (x=0; x<temp_s.length(); x++) {
    i=i + ((temp_s[x]-48) * (1<<(temp_s.length()-x-1)));
  }
  return i;
}

// Convert a string with a ascii 0 and 1 to a signed number (lon/lat)
double bin_to_I4(String temp_s){
  char temp_c[30] = "";

  int sign = temp_s[0]=='1'?-1:1;  
  temp_s.substring(1).toCharArray(temp_c, temp_s.length() );
  if(sign==-1) //flip for 1-complement
    for(int i=0;i<temp_s.length()-1;i++)temp_c[i] = temp_c[i]=='1'?'0':'1';
  double i4 = ( bin_to_int( temp_c )+1) * sign;
  return i4;
}


/*
!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C

Message sent (UTC) : 03:54:15
MMSI               : 477553000
Latitude           : 47.582833°
Longitude          : -122.345832°
Speed              : 0.0 knots
Heading            : 181°
Course over ground : 51°
Rate of turn       : 0°/min
Navigational status: 5

!AIVDM,1,1,,A,1000vV@P00Idw98ATMk00?wj0<0n,0*0F

# of ships in range: 54
MMSI               : 000016025
Latitude           : 30.705407°
Longitude          : -88.039618°
Speed              : 0.0 knots
Heading            : 511°
Course over ground : 0°
Rate of turn       : -9999°/min
Navigational status: 0
*/


/*

  1/10000 min +-180 East=postive, West=Neg
  The method is as follows:

  Take the decoded binary number from the AIS payload, 
  strip off the first bit as a sign, 
  then take the remaining bits as an unsigned integer 
  that is the two's complement of the actual number. 
  Convert the original data to its two's complement.

  The data payload is 1101000001101101111110111110
  
  The first bit of the payload was a "1" hence a negative number(i.e WEST). The remaining payload becomes:
  
  101000001101101111110111110
  
  We apply the method:
  
  101000001101101111110111110------original data
  010111110010010000001000001------one's complement
  010111110010010000001000010------add one to get two's complement

  Now we convert the new binary number to decimal as an unsigned integer:
  
  010111110010010000001000010 --> 49881154

  49881154 / 10000 = 4988.1154 min

  4988.1154 / 60 =83.135256666666667-degrees WEST
  

*/

/*
        1  2  3  4 5
        |  |  |  | | 
$--MWV,x.x,a,x.x,a*hh
1) Wind Angle, 0 to 360 degrees
2) Reference, R = Relative, T = True 
3) Wind Speed
4) Wind Speed Units, K/M/N
5) Status, A = Data Valid
6) Checksum


        1  2  3  4  5  6  7  8 9
        |  |  |  |  |  |  |  | |
$--VWR,x.x,a,x.x,N,x.x,M,x.x,K*hh<CR><LF>

1) Wind direction magnitude in degrees
2) Wind direction Left/Right of bow
3) Speed
4) N = Knots
5) Speed
6) M = Meters Per Second
7) Speed
8) K = Kilometers Per Hour
9) Checksum




SD Sounder Dept
DM Velocity Sensor, Speed Log, Water, Magnetic
VW Velocity Sensor, Speed Log, Water, Mechanical
WI Weather Instruments

DBK Depth Below Keel
DBS Depth Below Surface
DBT Depth Below Transducer <--
        1  2  3  4  5  6 7
        |  |  |  |  |  | |
$--DBT,x.x,f,x.x,M,x.x,F*hh
1) Depth, feet 
2) f = feet
3) Depth, meters 
4) M = meters
5) Depth, Fathoms  
6) F = Fathoms 
7) Checksum

Garmin NMEA 0183-information
Sända
Sats   Beskrivning
GPAPB APB: Kurs- eller spårstyrning (autopilot) mening ”B”
GPBOD BOD: Riktning (ursprung till destination)
GPBWC BWC: Riktning och avstånd till waypoint
GPGGA GGA: GPS-fixdata
GPGLL GLL: Geografisk position (latitud och longitud)
GPGSA GSA: GNSS DOP och aktiva satelliter
GPGSV GSV: GNSS-satelliter i sikte
GPRMB RMB: Rekommenderad minimiinformation för navigering
GPRMC RMC: Rekommenderat minimum för specifika GNSS-data
GPRTE RTE: Rutter
GPVTG VTG: Kurs över mark och markhastighet
GPWPL WPL: Waypoint-plats
GPXTE XTE: Avvikelse från utlagd kurs
PGRME E: Beräknat fel
PGRMM M: Kartdatum
PGRMZ Z: Höjd
SDDBT DBT: Djup under givare
SDDPT DPT: Djup
SDMTW MTW: Vattentemperatur
SDVHW VHW: Fart genom vattnet och kurs

Ta emot
Sats  Beskrivning
xxDPT Djup
xxDBT Djup under givare
xxMTW Vattentemperatur
xxVHW Fart genom vattnet och kurs
xxWPL Waypointens plats
xxDSC Digital selektiv anropsinformation
xxDSE Utökat digitalt selektivt anrop
xxHDG Kurs, avvikelse och variation
xxHDM Kurs, magnetisk
xxMWD Vindriktning och -hastighet
xxMDA Meteorologisk sammansatt
xxMWV Vindhastighet och -vinkel
xxVDM AIS VHF-datalänkmeddelande


*/

/*
  MTW Water Temperature
          1  2 3
          |  | | 
  $--MTW,x.x,C*hh
  1) Degrees
  2) Unit of Measurement, Celcius 
  3) Checksum
*/

/*
{
  "vessels": {
    "urn:mrn:signalk:uuid:c0d79334-4e25-4245-8892-54e8ccc8021d": { //urn:mrn:imo:mmsi:366982330
      "version": "0.1",
      "name": "motu",
      "mmsi": "2345678",
      "source": "self",
      "timezone": "NZDT",
      "navigation": {
        "state": {
          "value": "sailing",
          "source": "self",
          "timestamp": "2014-03-24T00:15:41Z"
        },
        "headingTrue": {
          "value": 2.3114,
          "$source": "nmea0183-1.II",
          "sentence": "HDT",
          "timestamp": "2014-03-24T00:15:41Z"
        },
        "speedThroughWater": {
          "value": 2.556,
          "$source": "n2k-1.160",
          "pgn": 128259,
          "timestamp": "2014-03-24T00:15:41Z"
        },
        "position": {
          "longitude": 23.53885,
          "latitude": 60.0844,
          "$source": "nmea0183-2.GP",
          "timestamp": "2014-03-24T00:15:42Z",
          "sentence": "GLL"
        }
      }
    }
  }
}

{
  "vessels": {
    "urn:mrn:imo:mmsi:366982330": {
      "mmsi": "230099999",
      "navigation": {
        "position": {
          "longitude": 173.1693,
          "latitude": -41.156426,
          "altitude": 0,
          "timestamp": "2015-01-25T12:01:01Z",
          "$source": "a.suitable.path"
        },
        "courseOverGroundTrue": {
          "value": 245.69,
          "timestamp": "2015-01-25T12:01:01Z",
          "$source": "a.suitable.path"
        }
      }
    }
  },
  "version": "1.0.0"
}

*/

