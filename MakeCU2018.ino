#define BLYNK_PRINT Serial
#include <BlynkSimpleCurieBLE.h>
#include <CurieBLE.h>
#include <Wire.h>

#include <math.h>
#include <CurieIMU.h>
/* PINS */
int temp_pin = A0;
int hall_pin = A2;
int vol_readpin = A4;
int vol_pin = 4; //starting from the highest range
int gain = 11;


String botName = "PocketSensy";

// Setup time section for polling connection
#include <Time.h>
#include <TimeLib.h>
int lastPoll = now();

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "your_auth_code";

BLEPeripheral  blePeripheral;

int timeSinceHit = 1000;
int timeSinceDestroyed = 0;
int hitPin = 9;

//######### SETUP ######################################
void setup() {
  Serial.begin(9600);
  delay(1000);
  set_Analog(temp_pin);
  set_Analog(hall_pin);
  setup_vol();
  setup_accl();

  // The name your bluetooth service will show up as, customize this if you have multiple devices
  //blePeripheral.setLocalName(botName);
  //blePeripheral.setDeviceName(botName);
  //blePeripheral.setAppearance(384);

  Blynk.begin(auth, blePeripheral);

  blePeripheral.begin();
  Serial.println("Waiting for connections...");

  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void set_Analog(int pin)
{
  pinMode (pin, INPUT);
}
/*
 * ====================================VOLTAGE=================================================
 */
void setup_vol()
{
  set_range(vol_pin);
}

void set_range(int pin)
{
  pinMode(4, INPUT);
  pinMode(3, INPUT); 
  pinMode(2, INPUT); 
  if (pin > 0) //1
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  /*if (pin == 4) {gain = 101;}
  if (pin == 3) {gain = 11;}
  if (pin == 2) {gain = 2;}*/
}

float process_vol()
{
   float vol_read;
   float input_voltage;
   int analog_value = analogRead(vol_readpin);
   input_voltage = (analog_value * 5.0) / 1024.0; 
   vol_read = input_voltage * gain;

   /*if (input_voltage < 0.9)
   {
    vol_pin = vol_pin - 1;
    set_range(vol_pin);
   }
   if (input_voltage >= 4.9)
   {
    vol_pin = vol_pin + 1;
    set_range(vol_pin);
   }
  */
    Serial.println(analog_value);
    Serial.println(input_voltage);
    return vol_read;
}

/*
 * ====================================TEMPERATURE==========================
 */
float process_temp()
{
  float Analog_T;
  float T;
  Analog_T = analogRead (temp_pin);
  T = log(((10240000/Analog_T) - 10000));
  T = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * T * T ))* T );
  float C = T - 273.15;
  return C;
  
}

/*
 * ====================================HALL EFFECT================================================
 */
int process_hall()
{
  float Analog_Mag;
  int Field_I;
  Analog_Mag = analogRead(hall_pin) * (5 / 1023);
  Field_I = map(Analog_Mag, 1, 4, -100, 100);
}

/*
 * =======================================ACCELERATION=======================================================
 */
void setup_accl()
{
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(2); // range: 2g; 
}

int x_accl = 0;
int y_accl = 0;
void process_accl() {
  float ax, ay, az;   //scaled accelerometer values

  // read accelerometer measurements from device, scaled to the configured range
  CurieIMU.readAccelerometerScaled(ax, ay, az);

  // display tab-separated accelerometer x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Blynk.virtualWrite(15, ax*100 + 50); 
  
  Serial.print("\t");
  Serial.print(ay);
  Blynk.virtualWrite(16, ay*100 + 50); 
  Serial.print("\t");
  Serial.print(az);
  Serial.println();
}

int lastNow = millis();
int timer = 0;
int tickZero = 0;
int start = millis();
//########## LOOP ######################################
void loop() {
  Blynk.run();
  blePeripheral.poll();
  // Read in sensors for damage
  handleSensors();
  delay(10);
}


//######### Subroutines ################################


float voltage = 0.0;
float temp = 0.0;
int hall_effect = 0.0;
// Read the destroyed status
BLYNK_READ(V8)
{
  Blynk.virtualWrite(8, voltage);
}

BLYNK_READ(V9)
{
  Blynk.virtualWrite(9, temp);
}

BLYNK_READ(V10)
{
  Blynk.virtualWrite(10, hall_effect);
}

BLYNK_READ(V15)
{
  Blynk.virtualWrite(15, x_accl*100 + 50); 
}

BLYNK_READ(V16)
{
  Blynk.virtualWrite(16, y_accl*100 + 50); 
}

void handleSensors() {
  Serial.print("Voltage: ");
  //Serial.print(process_vol(), 2);
  voltage = (process_vol() - 0.65)*(1.5/2.0);
  voltage = voltage - (voltage)*(voltage)*0.00934;
  Serial.println("V ");
  Serial.print ("Field Intensity: ");
  //Serial.print(process_hall());
  hall_effect = process_hall();
  Serial.println("mT ");
  Serial.print ("Temperature: ");
  //Serial.print (process_temp(), 2); 
  temp = process_temp()/2.0;
  Serial.println ("℃ ");
  process_accl();
  Blynk.virtualWrite(10, hall_effect);
  Blynk.virtualWrite(9, temp);
  Blynk.virtualWrite(20, temp);
  Blynk.virtualWrite(8, voltage);
}
