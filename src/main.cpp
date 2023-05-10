#include <Arduino.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <max6675.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

#define EEPROM_SIZE 4

#define TEMP_PLUS_BUTTON_PIN 16
#define TEMP_MINUS_BUTTON_PIN 17

#define Thermo_DO_PIN 5
#define Thermo_CS_PIN 23
#define Thermo_CLK_PIN 18

#define SSR_PIN 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX6675 thermocouple(Thermo_CLK_PIN, Thermo_CS_PIN, Thermo_DO_PIN);

#define ONE_WIRE_BUS 14

#define BREW_DETECTION_PIN 15
#define BREW_LOGIC_LEVEL LOW

hw_timer_t * timer = NULL;
hw_timer_t * timer2 = NULL;
hw_timer_t * timer3 = NULL;

double currentTemperature = 0;
double targetTemperature = 75;
float shotTime = 0;

// PID
int WindowSize = 1000; // 5000
unsigned long windowStartTime;
double Output;

double kp = 62;
double tn = 52;
double tv = 11.5;
double imax = 55;

double KP = kp;//62;
double KI = kp / tn; //TN = 52
double KD = tv * kp; //TV =11.5
PID pid(&currentTemperature, &Output, &targetTemperature, KP, KI, KD, DIRECT);
unsigned int isrCounter = 0;

bool brewDetected = false;
bool firstGoodTemperatureRead = false;

//OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
//DallasTemperature sensors(&oneWire);

void updateDisplay(float soll, float ist, float timer){
  display.setRotation(2);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    // Display static text
    display.println("Soll:"+ String(soll,0));
    display.println("Ist:" + String(ist,1));
    //display.println("Timer:"+ String(timer,1));
    if(timer < 100){
  	  display.println("Timer:"+ String(timer,1));
    }
    else if(timer >= 100){
      display.println("Timer:"+String(timer,0));
    }  
    display.display();
}

void ARDUINO_ISR_ATTR onTimer(){
  if(Output <= isrCounter)
  {
    digitalWrite(SSR_PIN, LOW);
  }
  else
  {
    digitalWrite(SSR_PIN, HIGH);
  }
  isrCounter += 10;
  if(isrCounter >= WindowSize){
    isrCounter = 0;
  }
}

void ARDUINO_ISR_ATTR onStopWatchIncrement(){
  if(brewDetected == true){
    shotTime += 0.1;
  }
}

void ARDUINO_ISR_ATTR onSensorRead(){
  currentTemperature = thermocouple.readCelsius();
  /*if((currentTemperature != 0) && (firstGoodTemperatureRead = false)){
    firstGoodTemperatureRead = true;
  }*/
}

void checkButtons()
{
  if(digitalRead(TEMP_PLUS_BUTTON_PIN) == HIGH)
  {
    targetTemperature++;
    EEPROM.write(0, targetTemperature);
    EEPROM.commit();
    delay(100);
  }
  if(digitalRead(TEMP_MINUS_BUTTON_PIN) == HIGH)
  {
    targetTemperature--;
    EEPROM.write(0, targetTemperature);
    EEPROM.commit();
    delay(100);
  }
}

void brewStarted() {
  brewDetected = true;      
  shotTime = 0;          
}

void brewEnded() {
  brewDetected = false;
}

void brewDetection(){
  if ((digitalRead(BREW_DETECTION_PIN) == BREW_LOGIC_LEVEL) && brewDetected == false){
    brewStarted();
  }
  else if((digitalRead(BREW_DETECTION_PIN) != BREW_LOGIC_LEVEL) && brewDetected == true){
    brewEnded();
  }
}

/*void safetyCheck(){
  if(firstGoodTemperatureRead){
    if (currentTemperature == 0){
    Serial.println("HELP!");
      while(1){
        digitalWrite(SSR_PIN, LOW);
      }
    }
  }
  else if(isnan(currentTemperature)){
    Serial.println("HELP!");
    while(1){
      digitalWrite(SSR_PIN, LOW);
    }
  }
}*/

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  targetTemperature = EEPROM.read(0);
  if(targetTemperature == 255)
  {
    targetTemperature = 95;
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  delay(2000);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onStopWatchIncrement, true);
  timerAlarmWrite(timer2, 100000, true);
  timerAlarmEnable(timer2);

  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer3, &onSensorRead, true);
  timerAlarmWrite(timer3, 1000000, true);
  timerAlarmEnable(timer3);

  pinMode(TEMP_PLUS_BUTTON_PIN, INPUT);
  pinMode(TEMP_MINUS_BUTTON_PIN, INPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(BREW_DETECTION_PIN, INPUT);

  windowStartTime = millis();
  pid.SetSampleTime(WindowSize);
  pid.SetOutputLimits(0, WindowSize);
  pid.SetMode(AUTOMATIC);
  pid.SetIntegratorLimits(0, 75);
  pid.SetSmoothingFactor(0.6);
}

void loop() {
  // put your main code here, to run repeatedly:
  pid.Compute();
  checkButtons();
  updateDisplay(targetTemperature, currentTemperature, shotTime);
  brewDetection();
  //safetyCheck();
}

