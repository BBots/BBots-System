#include "SoftwareServo.h"

int angle = 90;
SoftwareServo myservo;
SoftwareServo mymotor;
void setup()
{
  Serial.begin(9600);                     // set up Serial library at 9600 bps
  delay (5000);
  Serial.println("BBots system Init:");   // prints hello with ending line break
  IOConfig();
  Serial.println("IO Config finish");
  Serial.println("...");
  delay(500);
  Serial.println("Test Init:");
  Test();

}


void loop() {

  LedControl(0 , 1 , 0);


  angle++;

  if (angle > 179) {
    angle = 1;
  }

  custom_delay(2500);
  Serial.println("Running Main loop [End]...");
  LedControl(0 , 0 , 0);
  custom_delay(2500);
}


void Test(void)
{
  Serial.println("RGB Test:");
  delay(2500);
  LedControl(1 , 0 , 0);
  delay (2500);
  LedControl(1 , 1 , 0);
  delay (2500);
  LedControl(1 , 1 , 1);
  delay (2500);
  LedControl(0 , 1 , 1);
  delay (2500);
  LedControl(0 , 0 , 1);
  delay (2500);
  LedControl(0 , 0 , 0);
  delay (2500);
  Serial.println("RGB Test Finish");

  Serial.println("Buzzer Test:");
  tone(8, 2500, 250);
  tone(8, 3000, 250);
  Serial.println("Buzzer Test Finish");
  delay (2500);

  Serial.println("Servo Test:");

  for (int f = 30; f < 150; f++)
  {
    myservo.write(f);
    mymotor.write(f);
    SoftwareServo::refresh();
    custom_delay(100);
  }
  Serial.println("Servo Test Finish");
  delay (2500);

  Serial.println("Test Finish:");
}

void custom_delay(int miliseconds)        // Custom delay that allow servo refresh
{
  for (int i = 0; i <= miliseconds; i++)
  {
    SoftwareServo::refresh();
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



