#include "SoftwareServo.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"
#include "Adafruit_MMA8451.h"
#include "Adafruit_MPL3115A2.h"



int angle = 90;

SoftwareServo myservo;
SoftwareServo mymotor;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

void setup()
{
  Serial.begin(9600);                     // set up Serial library at 9600 bps
  delay (5000);
  Serial.println("===================================================================");
  Serial.println("===================== BBOTS SYSTEM INIT ===========================");
  Serial.println("===================================================================");
  Serial.println("");
  IOConfig();
  Serial.println("IO Config finish!!!");
  Serial.println("");
  delay(500);

  Test();

}


void loop() {
  Serial.println("");
  Serial.println("===================================================================");
  Serial.println("========================= MAIN LOOP ===============================");
  Serial.println("===================================================================");
  Serial.println("");

  LedControl(0 , 1 , 0);

  // SERVO CONTROL
  angle++;

  if (angle > 179) {
    angle = 1;
  }

  //Sensor COntrol
  sensors_event_t event; 
  mag.getEvent(&event);

  custom_delay(2500);
  Serial.println("===========");
  Serial.println("= COMPASS =");
  Serial.println("===========");
  Serial.println("");
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  Serial.println("");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  Serial.println("");  
 
  custom_delay(2500);
  Serial.println("=================");
  Serial.println("= ACCELEROMETER =");
  Serial.println("=================");
  Serial.println("");

   mma.read();
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.print(mma.z); 
  Serial.println();

  /* Get a new sensor event */ 
  // MMA8541Q
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");
  
  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();
  
  switch (o) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }
  Serial.println();
  Serial.println("");
  custom_delay(2500);

  // MPL3115A2
  Serial.println("=============");
  Serial.println("= ALTIMETER =");
  Serial.println("=============");
  Serial.println("");
   if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
  Serial.println("");

  custom_delay(2500);
  LedControl(0 , 0 , 0);
  custom_delay(250);

}


void Test(void)
{
  Serial.println("");
  Serial.println("===================================================================");
  Serial.println("========================= TEST INIT ===============================");
  Serial.println("===================================================================");
  Serial.println("");
  Serial.println("===============");
  Serial.println("=  RGB Test:  =");
  Serial.println("===============");
  Serial.println("");
  delay(2500);
  Serial.print("...");
  LedControl(1 , 0 , 0);
  delay (2500);
  Serial.print("...");
  LedControl(1 , 1 , 0);
  delay (2500);
  Serial.print("...");
  LedControl(1 , 1 , 1);
  delay (2500);
  Serial.print("...");
  LedControl(0 , 1 , 1);
  delay (2500);
  Serial.print("...");
  LedControl(0 , 0 , 1);
  delay (2500);
  Serial.print("...");
  LedControl(0 , 0 , 0);
  delay (2500);
  Serial.print("...");
  Serial.println("");  
  Serial.println("RGB Test Finish!!!");
  Serial.println("");
  Serial.println("================");
  Serial.println("= Buzzer Test: =");
  Serial.println("================");
  Serial.println("");
  tone(8, 2500, 250);
  Serial.print("...");
  tone(8, 3000, 250);
  Serial.print("...");
  Serial.println("");  
  Serial.println("Buzzer Test Finish!!!");
  Serial.println("");
  delay (2500);
  
  Serial.println("===============");
  Serial.println("= Servo Test: =");
  Serial.println("===============");
  Serial.println("");

  for (int f = 30; f < 150; f++)
  {
    myservo.write(f);
    mymotor.write(f);
    SoftwareServo::refresh();
    custom_delay(100);
    Serial.print(".");
  }
  Serial.println("");  
  Serial.println("Servo Test Finish!!!");
  Serial.println("");
  delay (2500);


  // SENSOR TEST INIT
  Serial.println("=============================");
  Serial.println("= HMC5883 Magnetometer Test =");
  Serial.println("=============================");
  Serial.println("");
  
    /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
   delay (2500);
   Serial.println("Adafruit MMA8451 test!");
   
   if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
  Serial.println("");  
  Serial.println("Adafruit MMA8451 test Finish!!!");
  Serial.println("");  
  delay (2500);

  Serial.println("============================");
  Serial.println("= Adafruit MPL3155A2 test! =");
  Serial.println("============================");
  Serial.println("");
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
    
  Serial.println("");  
  Serial.println("Adafruit MPL3155A2 test Finish!!!");
  Serial.println("");  

  delay(250);

  Serial.println("Test Finish:");
}

void custom_delay(int miliseconds)        // Custom delay that allow servo refresh
{
  int val = 0;
  for (int i = 0; i <= miliseconds; i++)
  {

     
    SoftwareServo::refresh();
	if (digitalRead(2)== 1)
	{
	  LedControl(1, 1, 1);	
    Serial.println("Button Pulsed!!!");
	}else
	{
    LedControl(0 , 0 , 0);
  }
    delay(1);
  }
}


void LedControl(unsigned char red, unsigned char green, unsigned char blue)
{
  if (red > 0)
  {
    digitalWrite(A0, LOW);
  }
  else
  {
    digitalWrite(A0, HIGH);
  }

  if (green > 0)
  {
    digitalWrite(9, LOW);
  }
  else
  {
    digitalWrite(9, HIGH);
  }

  if (blue > 0)
  {
    digitalWrite(A3, LOW);
  }
  else
  {
    digitalWrite(A3, HIGH);
  }
}


void IOConfig()
{
  // RGB Leds
  pinMode(9, OUTPUT);    // Green
  digitalWrite(9, HIGH);
  pinMode(A0, OUTPUT);   // Red
  digitalWrite(A0, HIGH);
  pinMode(A3, OUTPUT);   // Blue
  digitalWrite(A3, HIGH);

  // Buzzer Control
  pinMode(8, OUTPUT);    // 2N7002 gate Control

  // Push Button
  pinMode(2, INPUT);

  // Actuators
  pinMode(5, OUTPUT);    // Servo Control PWM signal
  mymotor.attach(5);
  pinMode(6, OUTPUT);    // Motor Control PWM signal
  myservo.attach(6);
  //Conectivity
  pinMode(A1, OUTPUT);   // Wifi On/Off
  pinMode(A2, OUTPUT);   // Bluetooth SW_Button
  pinMode(7, OUTPUT);    // Bluettoth Wake
  pinMode(10, OUTPUT);   // Wifi Program On/Off

}


// HMC5883L
void displaySensorDetails(void) 
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



