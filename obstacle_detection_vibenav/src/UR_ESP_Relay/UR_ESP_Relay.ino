// Tactile Navigation Stick for the Visually Impaired
// ESP32-Based Full Code

#include <WiFi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// -------------------- Pins --------------------
// Ultrasonic Sensors
#define TRIG_FRONT  12
#define ECHO_FRONT  14
#define TRIG_SIDE   27
#define ECHO_SIDE   26

// Vibration Motors
#define VIB_FRONT   32
#define VIB_SIDE    33
#define VIB_TEMP    25
#define VIB_FIRE    4

// Temperature Sensor (DS18B20)
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Flame Sensor
#define FLAME_SENSOR 34

// SIM800L
#define SIM_TX 17
#define SIM_RX 16
SoftwareSerial sim800(SIM_TX, SIM_RX);

// GPS
#define GPS_TX 5
#define GPS_RX 18
SoftwareSerial gpsSerial(GPS_TX, GPS_RX);
TinyGPSPlus gps;

// MPU6050
Adafruit_MPU6050 mpu;
bool fallDetected = false;

// -------------------- Constants --------------------
const int tempThreshold = 45;   // Celsius
const int flameThreshold = 3000; // ADC value
const long distanceThreshold = 70; // cm

// -------------------- Functions --------------------
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void vibrate(int pin, int duration = 300) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}

void checkObstacle() {
  long front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long side = readUltrasonic(TRIG_SIDE, ECHO_SIDE);

  if (front < distanceThreshold && front > 0) vibrate(VIB_FRONT);
  if (side < distanceThreshold && side > 0) vibrate(VIB_SIDE);
}

void checkTemperature() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC >= tempThreshold) vibrate(VIB_TEMP);
}

void checkFlame() {
  int flameVal = analogRead(FLAME_SENSOR);
  if (flameVal < flameThreshold) vibrate(VIB_FIRE);
}

void checkFall() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  if (abs(a.acceleration.z) < 3 && abs(a.acceleration.x) > 7) {
    fallDetected = true;
  }
}

void sendSMS(String msg) {
  sim800.println("AT+CMGF=1");
  delay(100);
  sim800.println("AT+CMGS=\"+911234567890\""); // Replace with your phone number
  delay(100);
  sim800.println(msg);
  delay(100);
  sim800.write(26); // CTRL+Z to send
  delay(5000);
}

void sendEmergencyAlert() {
  String alert = "Emergency! Fall or hazard detected.";
  if (gps.location.isValid()) {
    alert += "\nLocation: ";
    alert += String(gps.location.lat(), 6);
    alert += ", ";
    alert += String(gps.location.lng(), 6);
  } else {
    alert += "\nLocation: Unavailable";
  }
  sendSMS(alert);
  fallDetected = false;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_SIDE, OUTPUT);
  pinMode(ECHO_SIDE, INPUT);
  pinMode(VIB_FRONT, OUTPUT);
  pinMode(VIB_SIDE, OUTPUT);
  pinMode(VIB_TEMP, OUTPUT);
  pinMode(VIB_FIRE, OUTPUT);
  pinMode(FLAME_SENSOR, INPUT);

  // Modules
  sensors.begin();
  sim800.begin(9600);
  gpsSerial.begin(9600);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }

  delay(2000);
  Serial.println("System Ready");
}

// -------------------- Loop --------------------
void loop() {
  checkObstacle();
  checkTemperature();
  checkFlame();
  checkFall();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (fallDetected) {
    sendEmergencyAlert();
  }

  delay(300);
}