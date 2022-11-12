// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  WIFI  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "body.h"
#include "html510.h"
#include "Libraries/ArduinoJson.h"

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

void handleKeyPressed(){
  String keys = h.getText();
  keys.replace("%22", "\""); // Reconstruct quotations "" in JSON
  h.sendplain(keys);
  Serial.println("Keys Pressed: " + keys);

  StaticJsonBuffer<140> jsonBuffer; // Create the JSON buffer
  JsonObject& keysJSON = jsonBuffer.parseObject(keys); // Parse and create json object

  // Look variables
  int look_up = keysJSON["38"];
  int look_down = keysJSON["40"];
  int look_left = keysJSON["37"];
  int look_right = keysJSON["39"];\
  if (look_up == look_down == 1) { // If both look up+down --> cancel out
    look_up = 0;
    look_down = 0;
  }
  if (look_right == look_left == 1) { // If both look left+right --> cancel out
    look_right = 0;
    look_left = 0;
  }

  // Move variables
  int move_up = keysJSON["87"];
  int move_down = keysJSON["83"];
  int move_left = keysJSON["65"];
  int move_right = keysJSON["68"];
  if (move_up == move_down == 1) { // If both look up+down --> cancel out
    move_up = 0;
    move_down = 0;
  }
  if (move_right == move_left == 1) { // If both look left+right --> cancel out
    move_right = 0;
    move_left = 0;
  }

  // Speed variables
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
//  int speed_0  = keysJSON["48"];
}


void setup() { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  pinMode(LEDPIN, OUTPUT);

  Serial.begin(115200);
  
  Serial.print("Access Point SSID: "); Serial.print(ssid);
  WiFi.mode(WIFI_AP); // Set Mode to Access Point
  WiFi.softAP(ssid, password); // Define access point SSID and its password

  IPAddress myIP(192, 168, 1, 161); // Define my unique Static IP (from spreadsheet on slides)
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); // Define the (local IP, Gateway, Subnet mask)
  
  Serial.print("Use this URL to connect: http://"); Serial.print(myIP); Serial.println("/"); // Print the IP to connect to
  
  h.begin();

  h.attachHandler("/ ", handleRoot);
  h.attachHandler("/key_pressed?val=", handleKeyPressed);
}

void loop() { // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  h.serve();
  delay(10);
}

// https://medesign.seas.upenn.edu/index.php/Guides/ESP32-pins
