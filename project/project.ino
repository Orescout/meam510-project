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
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  else Serial.println("Fail "); 
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

//function to get the vive values
int getVive(int viveCoord) {
  //initialize all return values to 0
  int avg_lx = 0;
  int avg_ly = 0;
  int avg_rx = 0;
  int avg_ry = 0;

  switch (viveCoord) {
    case LEFT_X:
      Serial.print("VIVELEFT STATUS:");
      Serial.println(viveLeft.status());
      if (viveLeft.status() == VIVE_RECEIVING) {
        int lx1 = viveLeft.xCoord();
        int lx2 = viveLeft.xCoord();
        int lx3 = viveLeft.xCoord();
        int lx4 = viveLeft.xCoord();
        int lx5 = viveLeft.xCoord();
        int lx6 = viveLeft.xCoord();
        int lx7 = viveLeft.xCoord();
        int lx8 = viveLeft.xCoord();
        int lx9 = viveLeft.xCoord();
        int lx10 = viveLeft.xCoord();
        avg_lx = (lx1 + lx2 + lx3 + lx4 + lx5 +lx6 + lx7 + lx8 + lx9 + lx10)/10;
        neopixelWrite(RGBLED,0,avg_lx/200,avg_lx/200);  // blue to greenish
        Serial.printf("LeftX %d ", avg_lx);
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
      Serial.print("VIVELEFT STATUS:");
      Serial.println(viveLeft.status());
      if (viveLeft.status() == VIVE_RECEIVING) {
        int l1 = viveLeft.yCoord();
        int l2 = viveLeft.yCoord();
        int l3 = viveLeft.yCoord();
        int l4 = viveLeft.yCoord();
        int l5 = viveLeft.yCoord();
        int l6 = viveLeft.yCoord();
        int l7 = viveLeft.yCoord();
        int l8 = viveLeft.yCoord();
        int l9 = viveLeft.yCoord();
        int l10 = viveLeft.yCoord();
        avg_ly = (l1 + l2 + l3 + l4 + l5 +l6 + l7 + l8 + l9 + l10)/10;
        neopixelWrite(RGBLED,0,avg_ly/200,avg_ly/200);  // blue to greenish
        Serial.printf("LeftY %d ", avg_ly);
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
      Serial.print("VIVERIGHT STATUS:");
      Serial.println(viveRight.status());
      if (viveRight.status() == VIVE_RECEIVING) {
        int r1 = viveRight.xCoord();
        int r2 = viveRight.xCoord();
        int r3 = viveRight.xCoord();
        int r4 = viveRight.xCoord();
        int r5 = viveRight.xCoord();
        int r6 = viveRight.xCoord();
        int r7 = viveRight.xCoord();
        int r8 = viveRight.xCoord();
        int r9 = viveRight.xCoord();
        int r10 = viveRight.xCoord();
        avg_rx = (r1 + r2 + r3 + r4 + r5 +r6 + r7 + r8 + r9 + r10)/10;
        neopixelWrite(RGBLED,0,avg_rx/200,avg_rx/200);  // blue to greenish
        Serial.printf("RightX %d ", avg_rx);
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
      Serial.print("VIVERIGHT STATUS:");
      Serial.println(viveRight.status());
      if (viveRight.status() == VIVE_RECEIVING) {
        int r1 = viveRight.yCoord();
        int r2 = viveRight.yCoord();
        int r3 = viveRight.yCoord();
        int r4 = viveRight.yCoord();
        int r5 = viveRight.yCoord();
        int r6 = viveRight.yCoord();
        int r7 = viveRight.yCoord();
        int r8 = viveRight.yCoord();
        int r9 = viveRight.yCoord();
        int r10 = viveRight.yCoord();
        avg_ry = (r1 + r2 + r3 + r4 + r5 +r6 + r7 + r8 + r9 + r10)/10;
        neopixelWrite(RGBLED,0,avg_ry/200,avg_ry/200);  // blue to greenish
        Serial.printf("RightY %d ", avg_ry);
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
//TODO: update
void rotate_left() {
  return;
}
void rotate_right() {
  return;
}
void move_forward() {
  return;
}

void move(int speed, int FL, int FR, int BL, int BR) {

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

  //FULL NAV FUNCTION: TODO: would delete above calls of getVive

  int CenterX = getCenterX(leftx, rightx);
  int CenterY = getCenterY(rightx, righty); 
  pingstaff(); //staff comm
  sendXY(CenterX, CenterY); //send center x and y to staff computer (TAs)

  PoliceNav();
  //sender
  static int count; //initialize count to put message together from sender
  uint8_t message[200]; // Max ESPnow packet is 250 byte data

  /*
  // TEST ESP: put some message together to send
  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");

  delay(500); // ESPNow max sending rate (with default speeds) is about 50Hz
  */
}

