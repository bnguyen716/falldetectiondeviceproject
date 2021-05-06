// Gyroscopic Fall Detection Device with ECG Monitoring Project
// Briana Nguyen, Anthony Pipitone, Asyraf Norfadilah
// Spr21 Final Project BME 4056C

/* libraries used in this code */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "MAX30105.h"
#include "heartRate.h"

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

/* initializing variables for MPU */
float prev_gyroX, prev_gyroY, prev_gyroZ;
float current_gyroX, current_gyroY, current_gyroZ;
float change;

/* initializing variables for leds and button */
const int switchButton = 13;     // the number of the pushbutton pin
const int redLed =  10;         // the number of the red LED pin
const int greenLed = 9;        // the number of the green LED pin

/* initializing variables for LCD */
const int rs = 12, en = 11, d4 = 6, d5 = 5, d6 = 4, d7 = 3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/* initializing for time elapsed */
unsigned long bounceTime;
bool blinkLed, debounce, buttonState;
unsigned long previousTime = 0;
byte seconds ;
byte minutes ;
byte hours ;

/* initializing for heart rate */
const byte RATE_SIZE = 4; // amount of numbers used to average the bpm
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float bpm;
int beatAvg;

void setup() {

  Serial.begin(2000000);
  while (!Serial)
    delay(10); //

  /* initializing mpu6050 */
  if (!mpu.begin()) {      // A4 & A5 --> SDA & SCL pins
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  /* default settings for mpu */
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /* initializing max30102 */
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {     // built in I2C pins
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  particleSensor.setup(); // default settings

  /* configuring leds, button, led blinking */
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(switchButton, INPUT);
  blinkLed = debounce = buttonState = true;
  bounceTime = millis();

  /* initializing lcd screen size */
  lcd.begin(16, 2);
}

void loop() {

  /* device is activated; indicate by green LED */
  digitalWrite(greenLed, HIGH);

  /* loop for detecting fall */
  while (change < 2) {                  // change < 2 indicates no fall, threshold
    /* assigning previous loop's angular rotation value as current to start next loop calculation */
    prev_gyroX = current_gyroX;
    prev_gyroY = current_gyroY;
    prev_gyroZ = current_gyroZ;

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* read gyroscope data for mpu6050 and print out the values; measures angular rotation */
    prev_gyroX = g.gyro.x;
    prev_gyroY = g.gyro.y;
    prev_gyroZ = g.gyro.z;
    /* print out x, y, z rotation on serial monitor */
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
    Serial.println("");
    delay(500); // end first set of values; next loop starts another set

    change = current_gyroX - prev_gyroX;    // calculate difference in angular rotation values from the previous set
    Serial.print(change > 2);     // prints out 0 if no fall; 1 if fall detected
    Serial.println("");
    /* if change >= 2, fall has been detected so break the while loop to stop reading mpu
      and continue to following lines */
    if (change >= 2) {
      break;
    }
  }

  /* if fall is detected, ie an x axis change of at least 2, do the following */
  if (change >= 2) {
    /* blink red led */
    if (blinkLed) {
      digitalWrite(redLed, HIGH);
      delay(100);
      digitalWrite(redLed, LOW);
      delay(100);
      /* if no fall, keep red led off and clear lcd */
    } else {
      digitalWrite(redLed, LOW);
      lcd.clear();
    }
    /* start time and print time passed once fallen on the lcd */
    if (millis() >= (previousTime))
    {
      previousTime = previousTime + 1000;
      seconds = seconds + 1;
      if (seconds == 60) {
        seconds = 0;
        minutes = minutes + 1;
      } if (minutes == 60) {
        minutes = 0;
        hours = hours + 1;
      } if (hours == 13) {
        hours = 1;
      }
      lcd.setCursor(0, 0);    // prints on first line
      lcd.print("Time: ");
      lcd.print (hours, DEC);
      lcd.print (":");
      lcd.print (minutes, DEC);
      lcd.print (":");
      lcd.println(seconds, DEC);
    }
    /* read max30102 for heart rate data and print bpm on lcd */
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true)
    {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      bpm = 60 / (delta / 1000.0);    //calculate current bpm
      if (bpm < 255 && bpm > 20)
      { /* Take average of readings */
        rates[rateSpot++] = (byte)bpm;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    /* prints on serial monitor the IR value from sensor, current BPM, and average BPM */
    Serial.print("IR = ");
    Serial.print(irValue);
    Serial.print(", BPM = ");
    Serial.print(bpm);
    Serial.print(", Avg BPM = ");
    Serial.print(beatAvg);
    /* indicate whether or not sensor detects if a finger is placed on it */
    if (irValue < 50000)
      Serial.print("No finger detected pls place on top :D ");
    Serial.println("");
    /* print bpm on second line */
    lcd.setCursor(0, 1);
    lcd.print("BPM: ");
    lcd.print(beatAvg);
    /* debounce code for switch button to turn off red led if false fall detected */
    if (debounce) {
      if (millis() - bounceTime > 50)
        debounce = false;            //end debounce
    } else {
      if (digitalRead(switchButton) != buttonState) {
        buttonState = digitalRead(switchButton);
        bounceTime = millis();
        debounce = true;            //start debounce
        if (buttonState) {          // was button pressed?
          blinkLed = !blinkLed;     // blink red LED
        } 
      }
    }
  }
}
