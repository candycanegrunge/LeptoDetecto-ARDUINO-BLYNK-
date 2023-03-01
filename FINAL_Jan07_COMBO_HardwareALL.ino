
#include <LiquidCrystal_I2C.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include "GravityTDS.h"

// LCD
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

// TDS
#define TdsSensorPin A1
GravityTDS gravityTds;
 
float temp = 25,tdsValue = 0;

//Humidity Sensor 
#define DHTPIN 2   //DHT pin connected to digital pin 2 Arduino

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE); 
const int digitalOut = 3; // led pin 3 for the output 

// Soil Moisture sensor
#define soilWet 180  // Define max value we consider soil 'wet'
#define soilDry 750   // Define min value we consider soil 'dry'
#define soil20 636  // 20% of soil moisture content
// Sensor pins
#define sensorPower 7
#define sensorPin A0

//PH sensor
#define PH_PIN A1
float voltage,phValue,temperature = 25;
DFRobot_PH ph;

//Temp sensor
const int SENSOR_PIN = 13;              // Temp sesnor PIN 13 
OneWire oneWire(SENSOR_PIN);            // setup a oneWire instance
DallasTemperature tempSensor(&oneWire); // pass oneWire to DallasTemperature library
float tempCelsius;                      // temperature in Celsius

void setup()
{
    Serial.begin(9600);  
    ph.begin();
    tempSensor.begin();    // initialize the sensor
    
    // LCD
    lcd.init();
    lcd.clear();         
    lcd.backlight();      // Make sure backlight is on

    //Soil Moisture 
    pinMode(sensorPower, OUTPUT);
    digitalWrite(sensorPower, LOW);       // Initially keep the sensor OFF

    // Humidity 
    pinMode(DHTPIN, INPUT); // Initialize pin D2 as the input 
    pinMode(digitalOut, OUTPUT); // Initialize pin D3 as the output 
    dht.begin();

    // TDS
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
    gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
    gravityTds.begin();  //initialization
}

void loop()
{
 pHsensor();     //Call PH sensor

 Tempsensor();    //Call Temp Sensor

 SoilMoistureSensor();    //Call Soil Moisture Sensor

 HumiditySensor();    // Call Humidity Sensor

 TDS_Sensor();    // Call TDS Sensor
    
 delay(1000);
}


void Tempsensor()
{
  tempSensor.requestTemperatures();             // send the command to get temperatures
  tempCelsius = tempSensor.getTempCByIndex(0);  // read temperature in Celsius

  Serial.print("Temperature: ");
  Serial.print(tempCelsius);    // print the temperature in Celsius
  Serial.println(" C");

  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("TEMP C: ");
  lcd.print(tempCelsius);

  
  if (tempCelsius >= 28 && tempCelsius <= 30) {
    lcd.setCursor(0,1);   //Set cursor to character 2 on line 1
    lcd.print("Status: RISK");
  }
  else {
    lcd.setCursor(0,1);   //Set cursor to character 2 on line 0
    lcd.print("Status: NO RISK");
  }
  delay(2000);
  lcd.clear();
}

void pHsensor()
{
 static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        phValue = ph.readPH(voltage,temperature);  // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.println(phValue,2);
    }
    ph.calibration(voltage,temperature);           // calibration process by Serail CMD

    lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
    lcd.print("pH Value: ");
    lcd.print(phValue);

    if (phValue >= 7.0 && phValue <= 7.5) {
    lcd.setCursor(1,1);   //Set cursor to character 2 on line 1
    lcd.print("Status: RISK");
   }
    else {
    lcd.setCursor(1,1);   //Set cursor to character 2 on line 1
    lcd.print("Status: NO RISK");
  }

  delay(2000);
  lcd.clear();
}

// SOIL MOISTURE ----- START

void SoilMoistureSensor()
{
  int moisture = readSensor();
  Serial.print("Analog Output: ");
  Serial.println(moisture);

  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("SOIL: ");
  lcd.print(moisture);


  // Determine status of our soil
  if (moisture <= soil20) {
    Serial.println("Status: Soil is suitable for Leptospira bacteria");
    lcd.setCursor(1,1);   //Set cursor to character 2 on line 1
    lcd.print("Status: RISK");
  }
  else {
    Serial.println("Status: Soil is dry. NO risk");
    lcd.setCursor(1,1);   //Set cursor to character 2 on line 0
    lcd.print("Status: NO RISK");
  }
  
  delay(1000);  // Take a reading every second for testing
          // Normally you should take reading perhaps once or twice a day
  Serial.println();

  lcd.clear();
}

//  This function returns the analog soil moisture measurement
int readSensor() {
  digitalWrite(sensorPower, HIGH);  // Turn the sensor ON
  delay(10);              // Allow power to settle
  int val = analogRead(sensorPin);  // Read the analog value form sensor
  digitalWrite(sensorPower, LOW);   // Turn the sensor OFF
  return val;             // Return analog moisture value
}

// SOIL MOISTURE ----- END ------------------


// HUMIDITY - START

void HumiditySensor()
{
 // Wait a few seconds between measurements.
  delay(5000); // delay for 10 sec 
  // 10000ms = 10s 

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity(DHTPIN); //  
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("HUMIDITY: ");
  lcd.print(h);

    if ( h >65 ) // threshold but the value can be changed later (this is just for testing)
    {
     digitalWrite(digitalOut, HIGH); //turn on the led 
     lcd.setCursor(1,1);   //Set cursor to character 2 on line 0
     lcd.print("Status: RISK");
     // if the humidity > threshold, the LED will turn ON 
     // need to use digital, since this module has already converted the analog input to dig o/p
    }

    else {
      digitalWrite(digitalOut, LOW); // turn off the led
      lcd.setCursor(1,1);   //Set cursor to character 2 on line 0
      lcd.print("Status: NO RISK");
      // if humidity < threshold, the LED will turn OFF 
    }

  // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
    }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.println(h);

  delay(2000);
  lcd.clear();
}

// HUMIDITY - END


// TDS Sensor -- START

 void TDS_Sensor()
{
 //temperature = readTemperature();  //add your temperature sensor and read it
    gravityTds.setTemperature(temp);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print("TDS Value: ");
    Serial.print(tdsValue,0);
    Serial.println("ppm");
    
    lcd.setCursor(4,0);   //Set cursor to character 2 on line 0
    lcd.print("SALINITY: ");

    lcd.setCursor(3,1);   //Set cursor to character 2 on line 0
    lcd.print(tdsValue);
    lcd.print(" ppm");
    delay(1000);
    lcd.clear();
  }

// TDS Sensor -- END
