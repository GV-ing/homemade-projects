
#include <SolarCalculator.h>
#include <TimeLib.h>
#include <Servo.h>


#define PIN_AZIMUTH 8
#define PIN_ELEVATION 9
Servo myservo_Azimuth;
Servo myservo_Elevation;

// Location
double latitude = 40.84197;
double longitude = 14.23234;
int utc_offset = 1;

// Refresh interval, in seconds
int interval = 10;



void setup()
{
  Serial.begin(9600);
  myservo_Azimuth.attach(PIN_AZIMUTH);
  myservo_Elevation.attach(PIN_ELEVATION);

  // Set system time to compile time
  setTime(toUtc(compileTime()));
  //test();
  delay(1000);

  // Set time manually (hr, min, sec, day, mo, yr)
  //setTime(0, 0, 0, 1, 1, 2022);
}





void loop()
{
  static unsigned long next_millis = 0;

  // At every interval
  if (millis() > next_millis)
  {
    time_t utc = now();
    double az, el;

    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, latitude, longitude, az, el);
    int elevation=el;
    int azimuth=az;
    // Print results
    Serial.print(F("Az: "));
    Serial.print(azimuth-90);
    Serial.print(F("°  El: "));
    Serial.print(elevation);
    Serial.println(F("°"));

    next_millis = millis() + interval * 1000L;
    if(el>0){
      myservo_Elevation.write(elevation);
      myservo_Azimuth.write(azimuth-90);
    }
  }
  
}




time_t toUtc(time_t local)
{
  return local - utc_offset * 3600L;
}




// Code from JChristensen/Timezone Clock example
time_t compileTime()
{
  const uint8_t COMPILE_TIME_DELAY = 8;
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[4], *m;
  tmElements_t tm;

  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);

  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  time_t t = makeTime(tm);
  return t + COMPILE_TIME_DELAY;
}



void test() {
 for (int pos = 0; pos <= 50; pos ++) { 
    // in steps of 1 degree
   myservo_Elevation.write(pos);
      myservo_Azimuth.write(pos);             
    delay(50);                       
  for (int pos = 50; pos >= 0; pos -= 1) { 
   myservo_Elevation.write(pos);
      myservo_Azimuth.write(pos);              
    delay(50);                    
  }
}
}
