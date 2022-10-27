#include <Wire.h>
#include <ST7032.h>
#include "MHZ19.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <WiFi.h>
#include "Ambient.h"

#define led1 14
#define led2 15

ST7032 lcd;
MHZ19 myMHZ19;
Adafruit_BME680 bme; // I2C

const char* ssid     = "rg-wpa";
const char* password = "kuroshirosenmu";

WiFiClient client;
Ambient ambient;

unsigned int channelId = 52689;
const char* writeKey = "34319f04c8df7d3f";

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

unsigned int sec_counter = 0;
float bme_temp[120];
float bme_hum[120];
float bme_pres[120];
float co2_co2[120];
int status = 0;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  ambient.begin(channelId, writeKey, &client);

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
    if (millis() - getDataTimer >= 1000){
    //lcd.clear();
        if(WiFi.status() == WL_CONNECTED){
          digitalWrite(led1,HIGH);
        } else {
          Serial.println("Reconnecting to WiFi...");
          digitalWrite(led1,LOW);
          WiFi.disconnect();
          WiFi.reconnect();
        }
        
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

        float Temp;
        Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
        Serial.print("Temperature (C): ");
        Serial.println(CO2);

        Serial.println(sec_counter);

        lcd.setCursor(0, 0);
        lcd.print("                ");

        lcd.setCursor(0, 0);
        lcd.print("Co2:");
        lcd.print(CO2);

        int numOfChar, diff;
        if(CO2<500){
        numOfChar=0;
        } else if(CO2<600){
        numOfChar=1;
        diff = map(600-CO2,100,0,0,4);
        } else if(CO2<700){
        numOfChar=2;
        diff = map(700-CO2,100,0,0,4);
        } else if(CO2<800){
        numOfChar=3;
        diff = map(800-CO2,100,0,0,4);
        } else if(CO2<900){
        numOfChar=4;
        diff = map(900-CO2,100,0,0,4);
        } else if(CO2<1000){
        numOfChar=5;
        diff = map(1000-CO2,100,0,0,4);
        } else if(CO2<1100){
        numOfChar=6;
        diff = map(1100-CO2,100,0,0,4);
        } else if(CO2<1200){
        numOfChar=7;
        diff = map(1200-CO2,100,0,0,4);
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
        lcd.print("%         ");
        }
        lcd.setCursor(8, 1);
        lcd.print(bme.temperature);
        lcd.print("C  ");

        bme_temp[sec_counter] = bme.temperature;
        bme_hum[sec_counter] = bme.humidity;
        bme_pres[sec_counter] = (bme.pressure / 100.0);
        co2_co2[sec_counter] = CO2;



        getDataTimer = millis();
        sec_counter++;
    }

    if(sec_counter>=120){
        sec_counter = 0;
        float send_temp=0, send_hum=0, send_pres=0, send_co2=0;
        for (int i = 0; i < 120; i++){
            send_temp += bme_temp[i];
        }
        send_temp = send_temp / 120;
        ambient.set(1, send_temp);


        for (int i = 0; i < 120; i++) {
            send_hum += bme_hum[i];
        }
        send_hum = send_hum / 120;
        ambient.set(2, send_hum);


        for (int i = 0; i < 120; i++) {
            send_pres += bme_pres[i];
        }
        send_pres = send_pres / 120;
        ambient.set(3, send_pres);


        for (int i = 0; i < 120; i++) {
            send_co2 += co2_co2[i];
        }
        send_co2 = send_co2 / 120;
        ambient.set(4, send_co2);

        Serial.println("ambient send");
        Serial.println(send_temp);
        Serial.println(send_hum);
        Serial.println(send_pres);
        Serial.println(send_co2);

        status = ambient.send();
    }

    if(status){
        digitalWrite(led2, HIGH);
    } else {
        digitalWrite(led2, LOW);
    }
}
