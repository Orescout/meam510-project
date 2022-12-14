/* MEAM510 - Final project
 * VIVE location sent via ESP-NOW
 * Demonstrates high precision vive localization and displays on RGB LED
 * SOPHIE THOREL
 */
#include "vive510.h"

#include <WiFi.h>
#include <math.h>

//ESPNow: libraries to include
#include <esp_now.h>
#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define VivePinLEFT 19 // pin receiving signal from first LEFT Vive circuit: ie output of vive circuit
#define VivePinRIGHT 20 //pin receiving signal from second RIGHT Vive circuit
#define teamNumber 30 //TODO: important for game, when you implement ESPnow-Game-Sender.ino


//initialize VIVE object
Vive510 viveLeft(VivePinLEFT);
//TODO: initialize second VIVE object
Vive510 viveRight(VivePinRIGHT);


//define int values to each of the coordinates the vive function returns
#define LEFT_X 1
#define LEFT_Y 2
#define RIGHT_X 3
#define RIGHT_Y 4


//Bearing angle: measured in relation to the board orientation
//float Bearing;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ESPNOW SENDER SETUP: copied from canvas~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define CHANNEL 1                  // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0x84,0xF7,0x03,0xA8,0xBE,0x30} // receiver MAC address (last digit should be even for STA)

esp_now_peer_info_t peer1 = 
{
  .peer_addr = MAC_RECV, 
  .channel = CHANNEL,
  .encrypt = false,
};
//staff comm
esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78}, 
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};
esp_now_peer_info_t peercomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78},  //TODO: update to peer MAC address
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};

// callback when data is sent 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  //else Serial.println("Fail "); 
}

//game sender to required staff: taken from game-sender.ino code
void pingstaff() {
  uint8_t teamNum = 30;
  esp_now_send(staffcomm.peer_addr, &teamNum, 1);     
}

//send message to staff ESP32 once per second with vive XY location
void sendXY(int robotX, int robotY) {
  char msg[13];
  sprintf(msg, "%2d:%4d,%4d", teamNumber, robotX, robotY);
  esp_now_send(staffcomm.peer_addr, (uint8_t *) msg, 13);
}

//////////~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ NAVIGATION FUNCTIONS ////////~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
/*
//function to provide bearing angle of the bot, given two (x,y) pairs. Theta goes clockwise from the North direction
float getBearingAngle() {
  const float RAD2DEG = 180.0f / PI;
  float rad = atan2((LeftY - RightY), (LeftX-RightX)); //arctan returns angle in radians TODO: check
  float theta = rad*RAD2DEG; //convert to degrees

  //bound angle between 0 and 360
  if (theta < 0) {
    theta +=360;
  }
  BearingAngle = theta; //update global variable
  return theta; //TODO: since we are updating a global variable, no need to return anything. change function to void
}

//function to calculate bearing angle between bot and object
void getAngle(int targetX, int targetY) {
  //update bearing by passing in a command to wheels to rotate until target bearing is met
  float targetAngle = 0;
  //math to calculate targetAngle
  ////pass commands to motors
  //update current angle 
}
*/
//function to return the center coordinates of the bot, ie. middle pt between the two vive sensor readings
int getCenterX(int lx, int rx) {
  int CenterX = (lx + rx)/2;
  return CenterX;
}
int getCenterY(int ly, int ry) {
  int CenterY = (ly + ry)/2;
  return CenterY;
}

//function to return median of 3 values
int med3Filt(int a, int b, int c) {
  int mid;
  if ((a<=b) && (a <= c)) {
    mid = (b<=c) ? b : c;
  } else if ((b <= a) && (b <= c)) {
    mid = (a <= c) ? a : c;
  } else {
    mid = (a <= b) ? a : b;
  }
  return mid;
}


//function to get the vive values
int getVive(int viveCoord) {
  //initialize all return values to 0
  int avg_lx = 0;
  int avg_ly = 0;
  int avg_rx = 0;
  int avg_ry = 0;

  int vals_to_avg = 5; //parameter to set number of readings to average over
  switch (viveCoord) {
    case LEFT_X:
      //Serial.print("VIVELEFT STATUS:");
      //Serial.println(viveLeft.status());
      Serial.print("");
      Serial.print("");
      viveLeft.status();
      if (viveLeft.status() == VIVE_RECEIVING) {
        //loop over to take an average of 20 readings
        /*
        for (int i = 0; i < vals_to_avg; i++) {
          avg_lx +=viveLeft.xCoord();
        }
        avg_lx = avg_lx/vals_to_avg;
        */
        //implement med filter
        int r1 = viveLeft.xCoord();
        int r2 = viveLeft.xCoord();
        int r3 = viveLeft.xCoord();
        avg_lx = med3Filt(r1, r2, r3);
        neopixelWrite(RGBLED,0,avg_lx/200,avg_lx/200);  // blue to greenish
        //Serial.printf("LeftX %d ", avg_lx);
        Serial.print("LeftX:");
        Serial.print(avg_lx);
        Serial.print(",");
        //Serial.print("");
        //Serial.print("");
        return avg_lx;
      } else {
          switch (viveLeft.sync(5)) {
            break;
            case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
              neopixelWrite(RGBLED,64,32,0);  // yellowish
              Serial.println("Left vive: weak signal");
            break;
            default:
            case VIVE_NO_SIGNAL: // nothing detected     
              neopixelWrite(RGBLED,128,0,0);  // red
              Serial.println("Left vive: no signal");
          }
      }
      return avg_lx;
      //TODO: DO FOR ALL 4 CASES
      break;
    case LEFT_Y:
      //Serial.print("VIVELEFT STATUS:");
      //Serial.println(viveLeft.status());
      Serial.print("");
      Serial.print("");
      viveLeft.status();

      if (viveLeft.status() == VIVE_RECEIVING) {
        int r1 = viveLeft.yCoord();
        int r2 = viveLeft.yCoord();
        int r3 = viveLeft.yCoord();
        avg_ly = med3Filt(r1, r2, r3);
        /*
        for (int i = 0; i < vals_to_avg; i++) {
          avg_ly +=viveLeft.yCoord();
        }
        avg_ly = avg_ly/vals_to_avg;
        */
        neopixelWrite(RGBLED,0,avg_ly/200,avg_ly/200);  // blue to greenish
        //Serial.printf("LeftY %d ", avg_ly);
        Serial.print("LeftY:");
        Serial.print(avg_ly);
        Serial.print(",");
        //Serial.print("");
        //Serial.print("");
        return avg_ly;
      } else {
          switch (viveLeft.sync(5)) {
            break;
            case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
              neopixelWrite(RGBLED,64,32,0);  // yellowish
              Serial.println("Left vive: weak signal");
            break;
            default:
            case VIVE_NO_SIGNAL: // nothing detected     
              neopixelWrite(RGBLED,128,0,0);  // red
              Serial.println("Left vive: no signal");
          }
      }
      return avg_lx;
      break;
    case RIGHT_X:
      //Serial.print("VIVERIGHT STATUS:");
      //Serial.println(viveRight.status());
      Serial.print("");
      Serial.print("");
      viveRight.status();
      //viveRight.status();
      if (viveRight.status() == VIVE_RECEIVING) {
        int r1 = viveRight.xCoord();
        int r2 = viveRight.xCoord();
        int r3 = viveRight.xCoord();
        avg_rx = med3Filt(r1, r2, r3);
        /*
        for (int i = 0; i < vals_to_avg; i++) {
          avg_rx +=viveRight.xCoord();
        }
        avg_rx = avg_rx/vals_to_avg;
        */
        neopixelWrite(RGBLED,0,avg_rx/200,avg_rx/200);  // blue to greenish
        //Serial.printf("RightX %d ", avg_rx);
        Serial.print("RightX:");
        Serial.print(avg_rx);
        Serial.print(",");
        //Serial.print("");
        //Serial.print("");
        return avg_rx;
      } else {
          switch (viveRight.sync(5)) {
            break;
            case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
              neopixelWrite(RGBLED,64,32,0);  // yellowish
              Serial.println("Right vive: weak signal");
            break;
            default:
            case VIVE_NO_SIGNAL: // nothing detected     
              neopixelWrite(RGBLED,128,0,0);  // red
              Serial.println("Right vive: no signal");
          }
      }
      return avg_rx;
        //TODO: DO FOR ALL 4 CASES
      break;
    case RIGHT_Y:
      //viveRight.status();
      //Serial.print("");
      //Serial.print("VIVERIGHT STATUS:");
      //Serial.println(viveRight.status());
      Serial.print("");
      Serial.print("");
      viveRight.status();

      if (viveRight.status() == VIVE_RECEIVING) {
        int r1 = viveRight.yCoord();
        int r2 = viveRight.yCoord();
        int r3 = viveRight.yCoord();
        avg_ry = med3Filt(r1, r2, r3);
        /*
        for (int i = 0; i < vals_to_avg; i++) {
          avg_ry +=viveRight.yCoord();
        }
        avg_ry = avg_ry/vals_to_avg;
        */
        neopixelWrite(RGBLED,0,avg_ry/200,avg_ry/200);  // blue to greenish
        //Serial.printf("RightY %d ", avg_ry);
        Serial.print("RightY:");
        Serial.print(avg_ry);
        Serial.print(",");
        return avg_ry;
      } else {
          switch (viveRight.sync(5)) {
            break;
            case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
              neopixelWrite(RGBLED,64,32,0);  // yellowish
              Serial.println("Right vive: weak signal");
            break;
            default:
            case VIVE_NO_SIGNAL: // nothing detected     
              neopixelWrite(RGBLED,128,0,0);  // red
              Serial.println("Right vive: no signal");
          }
      }
      return avg_ry;
      break;     
  } //end of switch
}

//function to get police car x and y
//TODO: implement using UDP
int getPoliceCarX() {
  return 0;
}
int getPoliceCarY() {
  return 0;
}

//function to calculate distances to police car
// get distance from left sensor to police
float DistL() {
  //get updated vive values
  int leftx = getVive(LEFT_X);
  int lefty = getVive(LEFT_Y);
  int rightx = getVive(RIGHT_X);
  int righty = getVive(RIGHT_Y);
  int PoliceX = getPoliceCarX();
  int PoliceY = getPoliceCarY();
  return ((sqrt(sq(leftx - PoliceX)) + sq(lefty-PoliceY)));//pythagorean distance
}

float DistR() {
  int leftx = getVive(LEFT_X);
  int lefty = getVive(LEFT_Y);
  int rightx = getVive(RIGHT_X);
  int righty = getVive(RIGHT_Y);
  int PoliceX = getPoliceCarX();
  int PoliceY = getPoliceCarY();
  return (sqrt(sq(rightx-PoliceX) + sq(righty-PoliceY)));//pythagorean distance
}

//function to rotate: dummy for now
//TODO w/ Ori: update drive function
void rotate_left() {
  return;
}
void rotate_right() {
  return;
}
void move_forward() {
  return;
}

//TODO: implement function to get value of TOF sensor
float getTOF() {
  return 0.0;
}

//function police car nav
void PoliceNav() {
  //parameters: TODO: will need to test out values that work best. Arbitrary for now
  int errorMargin = 30; //acceptable difference between both distances that bot can still assume is aligned with car
  int offsetDiff = 50; //offset difference in case we start aligned
  int minTOF = 10; //the smallest TOF value we consider to be where bot is practically touching object

  //if aligned with car, need to offset angle to be able to go in the right direction
  if((abs(DistR() -DistL()))<errorMargin) { 
    while (abs(DistR() -DistL()) <  offsetDiff) {
      rotate_left();//replace with individual wheel commands passed in to go f'n
    }
  }
  //if right is closer to police car, turn right:
  if (DistR() < DistL()) { //
    while (DistL() - DistR() >= errorMargin) {
      rotate_right();//TODO: replace with GO
    }
  }
  if (DistL() < DistR()) { //
    while (DistR() - DistL() >= errorMargin) {
      rotate_left();//TODO: replace with GO
    }
  }
  //while distances are fairly close together, move forward towards target
  //TODO: edit while condition to add TOF condition that will check if we have reached the car
  while (abs(DistL() - DistR()) <= errorMargin) {
    //if TOF sensor val <= small_val (indicating we have reached police car)
    if (getTOF() > minTOF) {
      move_forward(); //TODO: replace with GO
    } else { //we are at the car: need to move car 12 inches
      move_forward(); //TODO: update to check that police x and y coords have changed 12 inches

    }
   
    /*TODO: incorporate TOF logic to slow down/stop once we get close to object: 
      * once TOF reader reads a small distance eg. 10cm, slow down;
      8 Once it reads 2cm, slow down again
    */
  }
  if (getTOF() > minTOF) {
    //restart PoliceNav function
    PoliceNav();
  }
  //will end function once we've deviated enough
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~SETUP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
void setup() {
  Serial.begin(115200);

  //sender setup: copied from canvas
  WiFi.mode(WIFI_STA);  
  Serial.print("Sending MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed");
    ESP.restart();
  }
  esp_now_register_send_cb(OnDataSent); //optional if you want back interrupt
    
  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed");     // ERROR  should not happen
  }

  //staff comm
  esp_now_add_peer(&staffcomm);
  //vive begin
  viveLeft.begin();
  viveRight.begin();
  Serial.println("  Vive trackers started");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~LOOP~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                              
void loop() {  
  //static uint16_t x,y; //TODO: replace with global variables LeftX, LeftY
  neopixelWrite(RGBLED,255,255,255);  // full white
  //TODO: get 10 values of vive and send the average of 10; do this for all 4 values, for each x y
  int leftx = getVive(LEFT_X);
  int lefty = getVive(LEFT_Y);
  int rightx = getVive(RIGHT_X);
  int righty = getVive(RIGHT_Y);

  //Serial.print("leftx");
  //Serial.print(leftx);
  //Serial.print(",");
  //Serial.print("lefty");
  //Serial.print(lefty);
  //Serial.print(",");
  //Serial.print("rightx");
  //Serial.print(rightx);
  //Serial.print(",");
  //Serial.print("righty");
  //Serial.print(righty);
  //Serial.print(",");
  Serial.print("DeltaX");
  Serial.print(leftx-rightx);
  Serial.print(",");
  Serial.print("DeltaY");
  Serial.println(lefty-righty);

  //FULL NAV FUNCTION: TODO: would delete above calls of getVive

  int CenterX = getCenterX(leftx, rightx);
  int CenterY = getCenterY(rightx, righty); 

  pingstaff(); //staff comm
  sendXY(CenterX, CenterY); //send center x and y to staff computer (TAs)

  //PoliceNav(); //TODO: once UDP tested, implement it here 
  //sender
  static int count; //initialize count to put message together from sender
  uint8_t message[200]; // Max ESPnow packet is 250 byte data

  /*
  // TEST ESP: put some message together to send
  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");

  */
  //delay(500); // ESPNow max sending rate (with default speeds) is about 50Hz
  delay(100); //vive still looks with a delay of 100
  //serial.println("tacos");
}

