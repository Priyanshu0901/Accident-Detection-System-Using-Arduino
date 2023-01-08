#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

bool   freefallDetected;
SoftwareSerial mySerial(5, 6);
SoftwareSerial GPS_SoftSerial(3, 4);
TinyGPSPlus gps; 
MPU6050 mpu;
char arr[30];
const int touch = 7;
const int buzzer = 2;
unsigned long start;
double lat_val, lng_val, alt_m_val;
uint8_t hr_val, min_val, sec_val;
bool loc_valid, alt_valid, time_valid;
String str;
volatile float minutes, seconds;
volatile int degree, secs, mins;


int freefallBlinkCount = 0;

void setup() {
  Serial.begin(9600); 
  GPS_SoftSerial.begin(9600); 
  pinMode(buzzer,OUTPUT);
  pinMode(touch,INPUT);
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  
  mpu.setIntFreeFallEnabled(true);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);

  mpu.setFreeFallDetectionThreshold(17);
  mpu.setFreeFallDetectionDuration(2);  
  
  checkSettings();

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  
  attachInterrupt(0, doInt, RISING);
  
}

void loop() {
        smartDelay(1000); 
        update();
        display();  //stores the data in the str variable 
        //Serial.print(str);
        Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();
  

  Serial.print(act.isFreeFall);
  Serial.print("\n");
  
  if (act.isFreeFall==1)
  { 
    ping();
    freefallBlinkCount++;
    if(freefallBlinkCount==1)
      {alert(str);}
   
    if (freefallBlinkCount == 20||digitalRead(touch)==HIGH)
    {freefallBlinkCount = 0;
      act.isFreeFall = 0;
   digitalWrite(2,LOW);
   
    }
  }        
        delay(100);
}

void alert(String a){
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+918697017290\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  a.toCharArray(arr,30);
  mySerial.write(arr);//text content
  mySerial.print((char)30);
  updateSerial();
}
void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
void ping(){
  int i;
  for(i=0;i<3;i++){
  digitalWrite(2,HIGH);
        delay(500);
  digitalWrite(2,LOW);
        delay(50);}
   for(i=0;i<3;i++){     
        digitalWrite(2,HIGH);
        delay(100);
  digitalWrite(2,LOW);
        delay(450);}
        for(i=0;i<3;i++){
  digitalWrite(2,HIGH);
        delay(500);
  digitalWrite(2,LOW);
        delay(50);}
         
        }
void doInt()
{
  freefallBlinkCount = 0;
  freefallDetected = true;  
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fal Threshold:          ");
  Serial.println(mpu.getFreeFallDetectionThreshold());

  Serial.print(" * Free FallDuration:           ");
  Serial.println(mpu.getFreeFallDetectionDuration());
  
  Serial.print(" * Clock Source:              ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:             ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets:     ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Accelerometer power delay: ");
  switch(mpu.getAccelPowerOnDelay())
  {
    case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
    case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
    case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
    case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
  }  
  
  Serial.println();
}


void update()
{
  lat_val = gps.location.lat(); /* Get latitude data */
        loc_valid = gps.location.isValid(); /* Check if valid location data is available */
        lng_val = gps.location.lng(); /* Get longtitude data */
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
        hr_val = gps.time.hour(); /* Get hour */
        min_val = gps.time.minute();  /* Get minutes */
        sec_val = gps.time.second();  /* Get seconds */
        time_valid = gps.time.isValid();  /* Check if valid time data is available */
}


void display()
{
 if (!loc_valid)
        {          
          str.concat("Latitude : ");
          str.concat("*****\n");
          str.concat("Longitude : ");
          str.concat("*****\n");
        }
        else
        {
          DegMinSec(lat_val);
          str.concat("Latitude in Decimal Degrees : ");
          str.concat(lat_val);
          str.concat("Latitude in Degrees Minutes Seconds : ");
          str.concat(degree);
          str.concat("\t");
          str.concat(mins);
          str.concat("\t");
          str.concat(secs);
          str.concat("\n"); 
          DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
          str.concat("Longitude in Decimal Degrees : ");
          str.concat(lng_val);
          str.concat("Longitude in Degrees Minutes Seconds : ");
          str.concat(degree);
          str.concat("\t");
          str.concat(mins);
          str.concat("\t");
          str.concat(secs);
          str.concat("\n"); 
        }
        if (!alt_valid)
        {
          str.concat("Altitude : ");
          str.concat("*****\n");
        }
        else
        {
          str.concat("Altitude : ");
          str.concat(alt_m_val);  
          str.concat("\n"); 
        }
        if (!time_valid)
        {
          str.concat("Time : ");
          str.concat("*****\n");
        }
        else
        {
          char time_string[32];
          sprintf(time_string, "Time : %02d/%02d/%02d \n", hr_val, min_val, sec_val);
          str.concat(time_string); 
          str.concat("\n");    
        }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  
      gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}
void DegMinSec( double tot_val)  
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}
