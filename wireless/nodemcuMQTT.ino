/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject tothe following conditions: * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

// For a connection via I2C using Wire include
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`


// Include custom images
#include "images.h"

// Include DHT
#include "DHTesp.h"

// Initialize the OLED display using Wire library
SSD1306Wire  display(0x3c, D3, D5);
// SH1106 display(0x3c, D3, D5);

#define CYCLE_DURATION 5000
typedef void (*Display)(void);

//Wifi
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "devNET"
#define WLAN_PASS       "password"
IPAddress ip(192, 168, 0, 231); //set static ip
IPAddress gateway(192, 168, 0, 1); //set getteway
IPAddress subnet(255, 255, 255, 0);//set subnet

#define AIO_SERVER      "192.168.0.231"
#define AIO_SERVERPORT  1883      192
#define AIO_USERNAME    "nodemcu"

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
//Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish pirSensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pirSensor");

// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");


int displayMode = 0;
int counter = 1;

int ledPin = D1;
int pirDataPin = D0;
int pirState = LOW;
int pirVal = 0;

DHTesp dht;
float humidity;
float temperatureF = 0.0;
float temperatureC = 0.0;
float heatIndex = 0.0;

void setup() {
  //Serial Setup
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.config(ip, gateway, subnet);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP()); 

  // Setup MQTT subscription for onoff feed.
  //mqtt.subscribe(&onoffbutton);
  
  //OLED Setup
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  //PIR Setup
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pirDataPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  //DHT Setup
  dht.setup(4, DHTesp::DHT11);
}

uint32_t x=0;

void drawPIRDisplay() {
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, "PIR Status:");

  if (pirVal == HIGH) {
    display.drawString(0, 25, "Motion DETECTED");
  } else {
    display.drawString(0, 25, "No Motion");
  }
}

void drawDHTDisplay() {
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, "DHT Status:");
  //Serial.println("DHT Debug");
  //Serial.println(humidity, 1);
  display.drawString(0, 20, "Humiditity: " + String(humidity)+" %");
  display.drawString(0, 30, "Temperature: " + String(temperatureF)+" °F");
  display.drawString(0, 40, "Heat Index: " + String(heatIndex)+" °F");
}

void drawHomeDisplay() {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 22, "HOME");
}

void pirModule() {
  pirVal = digitalRead(pirDataPin);  // read input value
  if (pirVal == HIGH) {            // check if the input is HIGH
    digitalWrite(ledPin, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
      //Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    digitalWrite(ledPin, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      //Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}

void dhtModule() {
  
 const unsigned long timeTwoSeconds = 2 * 1000UL;
 static unsigned long lastSampleTime = 0 - timeTwoSeconds;  // initialize such that a reading is due the first time through loop()

 unsigned long now = millis();
 if (now - lastSampleTime >= timeTwoSeconds)
 {
    lastSampleTime += timeTwoSeconds;
    // add code to take temperature reading here
    humidity = dht.getHumidity();
    temperatureC = dht.getTemperature();
    temperatureF = dht.toFahrenheit(temperatureC);
    heatIndex = dht.computeHeatIndex(temperatureF, humidity, true);
 }
}
 // add code to do other stuff here
  //delay(dht.getMinimumSamplingPeriod());

//  Serial.print(dht.getStatusString());
//  Serial.print("\t");
//  Serial.print(humidity, 1);
//  Serial.print("\t\t");
//  Serial.print(temperature, 1);
//  Serial.print("\t\t");
//  Serial.print(dht.toFahrenheit(temperature), 1);
//  Serial.print("\t\t");
//  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
//  Serial.print("\t\t");
//  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
//}

Display displays[] = {drawPIRDisplay, drawDHTDisplay, drawHomeDisplay};
int cycleLength = (sizeof(displays) / sizeof(Display));
long timeSinceLastModeSwitch = 0;

void loop() {
  // clear the display
  display.clear();
  // draw the current demo method
  displays[displayMode]();
  
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(10, 128, String(millis()));
  // write the buffer to the display
  display.display();
  
  //PIR
  pirModule();

  //DHT
  dhtModule();
  
  if (millis() - timeSinceLastModeSwitch > CYCLE_DURATION) {
    displayMode = (displayMode + 1)  % cycleLength;
    timeSinceLastModeSwitch = millis();
  }
  counter++;
  delay(10);

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  //MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

//  Adafruit_MQTT_Subscribe *subscription;
//  while ((subscription = mqtt.readSubscription(5000))) {
//    if (subscription == &onoffbutton) {
//      Serial.print(F("Got: "));
//      Serial.println((char *)onoffbutton.lastread);
//    }
//  }

  // Now we can publish stuff!
  //Serial.print(F("\nSending pirSensor val "));
  //Serial.print(x);
  //Serial.print("...");
  //if (! pirSensor.publish(x++)) {
  //  Serial.println(F("Failed"));
  //} else {
  //  Serial.println(F("OK!"));
  //}

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  //if(! mqtt.ping()) {
  //  mqtt.disconnect();
  //}
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
//void MQTT_connect() {
//  int8_t ret;
//
//  // Stop if already connected.
//  if (mqtt.connected()) {
//    return;
//  }
//
//  Serial.print("Connecting to MQTT... ");
//
//  uint8_t retries = 3;
//  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//       Serial.println(mqtt.connectErrorString(ret));
//       Serial.println("Retrying MQTT connection in 5 seconds...");
//       mqtt.disconnect();
//       delay(5000);  // wait 5 seconds
//       retries--;
//       if (retries == 0) {
//         // basically die and wait for WDT to reset me
//         while (1);
//       }
//  }
//  Serial.println("MQTT Connected!");
//}
