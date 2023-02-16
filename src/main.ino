#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>

#include <Adafruit_SSD1306.h>

#include <PID_v1.h>

// Serial Debug Output
#define DEBUG

// Pinout
#define TEMP_SENSOR_PIN 2
#define SSR_Heating_PIN 8

// Config
#define DISPLAY_I2C_ADDRESS 0x3D

// Temperatur Sensor Init
// Setup a OneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_PIN);
// Pass OneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);
// OLed Display
// Auflösung des SSD1306-OLED-Displays
#define DISPLAY_WIDTH 128  // Breite in Pixeln
#define DISPLAY_HEIGHT 64  // Höhe in Pixeln
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, -1);

// PID Controller
double Kp =200, Ki = 20, Kd = 0;
double Setpoint, Input, Output;
int WindowSize = 5000;
unsigned long windowStartTime;
PID pidControl(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);




void setup() {
  Serial.begin(9600);
  // Initialize Temperature Sensor
  sensors.begin(); // Start up the library
  // Initialize OLed Display
  Wire.begin();
  Wire.setClock(400000L);
  Serial.println("Hello");
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS)) {
    Serial.println("SSD1306 nicht gefunden");
    for (;;) ;
  }
  Serial.println("Done");
  //display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS);
  display.clearDisplay();
  //display.fillScreen(WHITE);

  // PID Controller
  pinMode(SSR_Heating_PIN, OUTPUT);
  windowStartTime = millis();
  Setpoint = 40;
  pidControl.SetOutputLimits(0, WindowSize);
  //turn the PID on
  pidControl.SetMode(AUTOMATIC);
}

void loop() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
  } 
  printDisplayText(95, tempC);
  Input = (double)tempC;
  pidControl.Compute();
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime) {
    digitalWrite(SSR_Heating_PIN, HIGH);
  }
  else {
    digitalWrite(SSR_Heating_PIN, LOW);
  }
  #ifdef DEBUG
    Serial.print("Sollwert: ");
    Serial.print(Setpoint);
    Serial.print(" C");
    Serial.print("Istwert: ");
    Serial.print(tempC);
    Serial.println(" C");
  #endif
}

void printDisplayText(float targetValue, float actualValue){
    display.clearDisplay();  // Display(puffer) löschen
    display.setTextSize(1);  // kleine Schriftgröße (Höhe 8px)
    display.setTextColor(WHITE);  // helle Schrift, dunkler Grund)
    display.setCursor(0, 0);  // links oben anfangen
    display.print("Sollwert: ");
    display.print(targetValue);
    display.println(" °C");
    display.print("Istwert: ");
    display.print(actualValue);
    display.print(" °C");
    display.display();
}
