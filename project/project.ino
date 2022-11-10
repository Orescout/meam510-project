// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  WIFI  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "body.h"
#include "html510.h"



HTML510Server h(80);



const char* ssid = "TEAM HAWAII WIFI";
const char* password = "borabora";

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CONSTANTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define LEDC_CHANNEL 1 // use first of 6  
#define LEDC_RESOLUTION_BITS 12
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1) 
#define LEDC_FREQ_HZ 6000
#define LEDPIN 1

#define LED_BUILTIN 2
#define Hz 10

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void println(int x) {
  Serial.println(x);
}

void handleRoot() {
  h.sendhtml(body);
}

void handleH() {
  digitalWrite(LEDPIN, HIGH); // LED ON
  h.sendhtml(body);
}

void handleL() {
  digitalWrite(LEDPIN, LOW); // LED OFF
  h.sendhtml(body);
}
//
//void handleFrequencySlider(){
//  int v = h.getVal();
//  h.sendplain(String(v));
//  Serial.print("Frequency: "); Serial.println(String(h.getVal()));
//}
//
//void handleDutyCycleSlider(){
//  String v = String(h.getVal());
//  h.sendplain(v + "%");
//  Serial.print("Duty Cycle: "); Serial.print(v); Serial.println("%");
//}

void handleKeyPressed(){
  String key = h.getText();
  key.replace("%22", "\"");
  h.sendplain(key);
  Serial.println("Key Pressed: " + key);
}


void setup() { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
//  (LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
//  ledcAledcSetupttachPin(LEDPIN, LEDC_CHANNEL);
//  Serial.begin(9600);
//  pinMode(LED_BUILTIN, OUTPUT);
//  pinMode(5, INPUT);
  pinMode(LEDPIN, OUTPUT);

  Serial.begin(9600);
  
  Serial.print("Access Point SSID: "); Serial.print(ssid);
  WiFi.mode(WIFI_AP); // Set Mode to Access Point
  WiFi.softAP(ssid, password); // Define access point SSID and its password

  IPAddress myIP(192, 168, 1, 161); // Define my unique Static IP (from spreadsheet on slides)
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); // Define the (local IP, Gateway, Subnet mask)
  
  Serial.print("Use this URL to connect: http://"); Serial.print(myIP); Serial.println("/"); // Print the IP to connect to
  
  h.begin();
//  h.attachHandler("/H", handleH);
//  h.attachHandler("/L", handleL);
  h.attachHandler("/ ", handleRoot);
//  h.attachHandler("/frequency_slider?val=", handleFrequencySlider);
//  h.attachHandler("/duty_cycle_slider?val=", handleDutyCycleSlider);
  h.attachHandler("/key_pressed?val=", handleKeyPressed);
}

void loop() { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  h.serve();
  delay(10);

//    ledcWrite(LEDC_CHANNEL, 0);
//    delay(1000);
//    ledcWrite(LEDC_CHANNEL, 200);
//    delay(1000);
  
//  int duty_cycle = (int) analogRead(5) / 16.1;
//  ledcWrite(LEDC_CHANNEL, duty_cycle);
//  
//  println(analogRead(5));
//  println(duty_cycle);
//
//  delay((int) 1000.0 * 1.0 / Hz);

//  ledcWrite(LEDC_CHANNEL, 0);
//  delay((int) 1000.0 * 1.0 / Hz);
}

// https://medesign.seas.upenn.edu/index.php/Guides/ESP32-pins
