#include <Wire.h>
#include <ST7032.h>
#include "MHZ19.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
ST7032 lcd;
MHZ19 myMHZ19;
Adafruit_BME680 bme; // I2C

#define SEALEVELPRESSURE_HPA (1013.25)
unsigned long getDataTimer = 0;
int showflg=0;
byte nuru[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
byte one[8] = {
  0b00001,
  0b00001,
  0b00001,
  0b00001,
  0b00001,
  0b00001,
  0b00001,
  0b00001
};
byte two[8] = {
  0b00011,
  0b00011,
  0b00011,
  0b00011,
  0b00011,
  0b00011,
  0b00011,
  0b00011
};
byte three[8] = {
  0b00111,
  0b00111,
  0b00111,
  0b00111,
  0b00111,
  0b00111,
  0b00111,
  0b00111
};
byte four[8] = {
  0b01111,
  0b01111,
  0b01111,
  0b01111,
  0b01111,
  0b01111,
  0b01111,
  0b01111
};


void setup() {
  Serial.begin(9600); 
  Serial2.begin(9600);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  
  
  myMHZ19.begin(Serial2); 

    myMHZ19.autoCalibration();
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setContrast(40);
  lcd.createChar(0, nuru);
  lcd.createChar(1, one);
  lcd.createChar(2, two);
  lcd.createChar(3, three);
  lcd.createChar(4, four);
}

void loop() {
if (millis() - getDataTimer >= 2000){
  //lcd.clear();
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Temperature = ");
    Serial.print(bme.temperature);
    Serial.println(" *C");
  
    Serial.print("Pressure = ");
    Serial.print(bme.pressure / 100.0);
    Serial.println(" hPa");
  
    Serial.print("Humidity = ");
    Serial.print(bme.humidity);
    Serial.println(" %");
  
    Serial.print("Gas = ");
    Serial.print(bme.gas_resistance / 1000.0);
    Serial.println(" KOhms");
  
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    int CO2; 
    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
    if below background CO2 levels or above range (useful to validate sensor). You can use the 
    usual documented command with getCO2(false) */

    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
    
    Serial.print("CO2 (ppm): ");                      
    Serial.println(CO2);                                

    int8_t Temp;
    Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");                  
    Serial.println(CO2);
    
    lcd.setCursor(0, 0);
    lcd.print("                ");
    
    lcd.setCursor(0, 0);
    lcd.print("Co2:");
    lcd.print(CO2);

    int numOfChar, diff;
    if(CO2<800){
      numOfChar=0;
    } else if(CO2<900){
      numOfChar=1;
      diff = map(900-CO2,100,0,0,4);
    } else if(CO2<1000){
      numOfChar=2;
      diff = map(1000-CO2,100,0,0,4);
    } else if(CO2<1100){
      numOfChar=3;
      diff = map(1100-CO2,100,0,0,4);
    } else if(CO2<1200){
      numOfChar=4;
      diff = map(1200-CO2,100,0,0,4);
    } else if(CO2<1300){
      numOfChar=5;
      diff = map(1300-CO2,100,0,0,4);
    } else if(CO2<1400){
      numOfChar=6;
      diff = map(1400-CO2,100,0,0,4);
    } else if(CO2<1500){
      numOfChar=7;
      diff = map(1500-CO2,100,0,0,4);
    } else {
      numOfChar=8;
    }
    int i;
    for(i=0; i<numOfChar; i++){
      lcd.setCursor(15-i, 0);
      lcd.write(0);
    }
    if(numOfChar!=8 && numOfChar!=0){
      lcd.setCursor(15-i, 0);
      lcd.write(diff);
    }
   
    lcd.setCursor(0, 1);
    lcd.print("                ");
    
    lcd.setCursor(0, 1);
    if(false){
      lcd.print(bme.pressure / 100.0);
      lcd.print(" ");
    } else {
      lcd.print(bme.humidity);
      lcd.print("% ");
    }
    lcd.setCursor(8, 1);
    lcd.print(bme.temperature);
    lcd.print("C");
    getDataTimer = millis();
    showflg++;
  }
}
