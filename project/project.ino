// Libraries to import
#include "body.h"
#include "html510.h"
#include "Libraries/ArduinoJson.h"

HTML510Server h(80);

// https://medesign.seas.upenn.edu/index.php/Guides/ESP32-pins

// Wifi
const char* ssid = "TEAM HAWAII WIFI";
const char* password = "borabora";

//PWM channels for ledC
#define PWM_CH_A 0
#define PWM_CH_B 1
#define PWM_CH_C 2
#define PWM_CH_D 3

//motor movement control pins
//FL
#define MOTOR_A_GPIO_ONE 12
#define MOTOR_A_GPIO_TWO 11
//FR
#define MOTOR_B_GPIO_ONE 14
#define MOTOR_B_GPIO_TWO 13
//BL
#define MOTOR_C_GPIO_ONE 10
#define MOTOR_C_GPIO_TWO 9
//BR
#define MOTOR_D_GPIO_ONE 3
#define MOTOR_D_GPIO_TWO 8

//encoder pins
#define ENCODER_GPIO_A 1
#define ENCODER_GPIO_B 2
#define ENCODER_GPIO_C 42
#define ENCODER_GPIO_D 41

//intitial direction of motors
//1 is CW, 0 is CCW
#define DIR_A 0
#define DIR_B 1
#define DIR_C 0
#define DIR_D 1

//reverse direction
#define BACK_DIR_A 1
#define BACK_DIR_B 0 
#define BACK_DIR_C 1
#define BACK_DIR_D 0

//PWM setup
#define PWM_RES 12
#define PWM_FREQ 25

//time keeping variable
float tick = 0.0;

//time interval for slot calculation (in milliseconds)
#define TIME_INTERVAL 200

//PID setup
#define Kp .05
#define Ki 1.1

class Motor
{
private:
    int pwm_ch;
    int move_gpio_one;
    int move_gpio_two;
    int encoder_gpio;
    int dir;
    int encoder_state;
    int old_encoder_state;
    int curr_slot;
    int old_slot;
    float integration_sum_PID;

public:
    Motor(int pwm_ch, int move_gpio_one, int move_gpio_two, int encoder_gpio, int dir)
    {
        this->pwm_ch = pwm_ch;
        this->move_gpio_one = move_gpio_one;
        this->move_gpio_two = move_gpio_two;
        this->encoder_gpio = encoder_gpio;
        this->dir = dir;

        init();
    }
    void init()
    {
        // Set initial direction
        changeDirection(1);

        // encoder pin setup
        pinMode(this->encoder_gpio, INPUT);

        // various initializations
        this->curr_slot = 0;
        this->old_slot = 0;
        this->encoder_state = 1;
        this->old_encoder_state = 0;
        this->integration_sum_PID = 0;
    }

    
    // calculates velocity using how many slots have moved in the time interval
    int getVel()
    {
        int vel = this->curr_slot - this->old_slot;
        this->old_slot = this->curr_slot;
        return vel;
    }

    // increments/decrements pos each time encoder goes
    // constantly keeping track of the number of slots gone by
    void updateEncoder()
    {
        // read encoder
        this->encoder_state = digitalRead(this->encoder_gpio);
        // if in new hole update
        if (this->encoder_state != this->old_encoder_state)
        {
            this->old_encoder_state = this->encoder_state;
            if (this->dir == 1)
            {
                // CW
                this->curr_slot = this->curr_slot + 1;
            }
            else
            {
                // CCW
                this->curr_slot = this->curr_slot - 1;
            }
        }
    }

    void go(int targetSpeed)
    {
      ledcWrite(this->pwm_ch, targetSpeed*400);
    }

    void changeDirection(int forward)
    {
        // set the direction that the motors moves
        if (forward == this->dir)
        {
            // Move Forward: setting pin one to be the PWM pin
            ledcAttachPin(this->move_gpio_one, this->pwm_ch);
            ledcSetup(this->pwm_ch, PWM_FREQ, PWM_RES);
    
            // setting the other gpio (pin two) to be grounded
            pinMode(this->move_gpio_two, OUTPUT);
            digitalWrite(this->move_gpio_two, LOW);
        }
        else
        {
            // Move Backward: setting pin two to be the PWM pin
            ledcAttachPin(this->move_gpio_two, this->pwm_ch);
            ledcSetup(this->pwm_ch, PWM_FREQ, PWM_RES);
    
            // setting the other gpio (pin one) to be grounded
            pinMode(this->move_gpio_one, OUTPUT);
            digitalWrite(this->move_gpio_one, LOW);
        }
    }
};

// Creating all the motor objects
Motor motorFrontLeftA(PWM_CH_A, MOTOR_A_GPIO_ONE, MOTOR_A_GPIO_TWO, ENCODER_GPIO_A, DIR_A);
Motor motorBackLeftB(PWM_CH_B, MOTOR_B_GPIO_ONE, MOTOR_B_GPIO_TWO, ENCODER_GPIO_B, DIR_B);
Motor motorFrontRightC(PWM_CH_C, MOTOR_C_GPIO_ONE, MOTOR_C_GPIO_TWO, ENCODER_GPIO_C, DIR_C);
Motor motorBackRightD(PWM_CH_D, MOTOR_D_GPIO_ONE, MOTOR_D_GPIO_TWO, ENCODER_GPIO_D, DIR_D);

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
  // Serial.println("Keys Pressed: " + keys);

  StaticJsonBuffer<600> jsonBuffer; // Create the JSON buffer
  JsonObject& keysJSON = jsonBuffer.parseObject(keys); // Parse and create json object

  // Look variables
  int look_left = keysJSON["37"];
  int look_right = keysJSON["39"];
  if (look_right == look_left == 1) { // If both look left+right --> cancel out
    look_right = 0;
    look_left = 0;
  }
  int look_direction = 0;
  if (look_left || look_right) {
    look_direction = -look_left + look_right;
  }
  if (look_direction != 0) {
    Serial.print("Look: ");
    Serial.print(look_direction);
    Serial.println(" direction.");
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
  int move_degrees = -1;
  if (move_up || move_down) {
    move_degrees = move_up * 0 + move_down * 180;
  }
  if (move_right) {
    move_degrees = 90 - move_up * 45 + move_down * 45;
  }
  if (move_left) {
    move_degrees = 270 - move_down * 45 + move_up * 45;
  }
  if (move_degrees != -1) {
    Serial.print("Move: ");
    Serial.print(move_degrees);
    Serial.println(" degrees.");
  }

  // Speed variables - biggest speed rules
  static int current_speed = 5;
  int speed_0  = keysJSON["48"];
  int speed_1  = keysJSON["49"];
  int speed_2  = keysJSON["50"];
  int speed_3  = keysJSON["51"];
  int speed_4  = keysJSON["52"];
  int speed_5  = keysJSON["53"];
  int speed_6  = keysJSON["54"];
  int speed_7  = keysJSON["55"];
  int speed_8  = keysJSON["56"];
  int speed_9  = keysJSON["57"];

  // Update current speed
  if (speed_0 || speed_1 || speed_2 || speed_3 || speed_4 || speed_5 || speed_6 || speed_7 || speed_8 || speed_9) { // If any 0-9 key is pressed
    current_speed = max(max(max(speed_0*0, speed_1*1), max(speed_2*2, speed_3*3)), max(max(max(speed_4*4, speed_5*5), max(speed_6*6, speed_7*7)), max(speed_8*8, speed_9*9)));
  }

  drive(move_degrees, look_direction, current_speed);
}

void setAllDirections(int direction_A, int direction_B, int direction_C, int direction_D) {
  motorFrontLeftA.changeDirection(direction_A);
  motorBackLeftB.changeDirection(direction_B);
  motorFrontRightC.changeDirection(direction_C);
  motorBackRightD.changeDirection(direction_D);
}

void moveAllMotors(int speed_A, int speed_B, int speed_C, int speed_D) {
  motorFrontLeftA.go(speed_A);
  motorBackLeftB.go(speed_B);
  motorFrontRightC.go(speed_C);
  motorBackRightD.go(speed_D);
}

//subroutine for turning each motor in the correct direction given the move_degree and look_direction
void drive(int move_degrees, int look_direction, int speed) {

  // STOP motors if move_deg = -1 and look_dir = 0
  if (move_degrees == -1 && look_direction == 0) {
    moveAllMotors(0, 0, 0, 0);
    Serial.println("STOP");
  }

  // TURN RIGHT handle case where both look and move is on: only look if move not on
  if (look_direction == 1  && move_degrees == -1) { //look right: rotate CW;
    setAllDirections(1, 1, 0, 0);
    moveAllMotors(speed, speed, speed, speed);
    Serial.println("LOOKING right");
  }

  // TURN LEFT
  if (look_direction == -1 && move_degrees == -1) { //look left: rotate CCW;
    setAllDirections(0, 0, 1, 1);
    moveAllMotors(speed, speed, speed, speed);
    Serial.println("LOOKING left");
  }

  // MOVE in XY
  switch (move_degrees) {
    case 0: // Forward
      setAllDirections(1, 1, 1, 1);
      moveAllMotors(speed, speed, speed, speed);
      Serial.println("0 degrees: MOVE N");
      break;

    case 45: // NE
      setAllDirections(1, 1, 1, 1);
      moveAllMotors(0, speed, speed, 0);
      Serial.println("45 degrees: MOVE NE");
      break;

    case 90: // RIGHT; East
      setAllDirections(1, 0, 0, 1);
      moveAllMotors(speed, speed, speed, speed);
      Serial.println("90 degrees: MOVE east");
      break;
      
    case 135: // SE
      setAllDirections(0, 0, 0, 0);
      moveAllMotors(speed, 0, 0, speed);
      Serial.println("135 degrees: MOVE SE");
      break;

    case 180: // SOUTH
      setAllDirections(0, 0, 0, 0);
      moveAllMotors(speed, speed, speed, speed);
      Serial.println("180 degrees: MOVE S");
      break;

    case 225: // SW
      setAllDirections(0, 0, 0, 0);
      moveAllMotors(0, speed, speed, 0);
      Serial.println("225 degrees: MOVE SW");
      break;

    case 270: // WEST (left)
      setAllDirections(0, 1, 1, 0);
      moveAllMotors(speed, speed, speed, speed);
      Serial.println("270 degrees: MOVE W");
      break;

    case 315: // NW
      setAllDirections(1, 1, 1, 1);
      moveAllMotors(speed, 0, 0, speed);
      Serial.println("315 degrees: MOVE NW");
      break;
  }
}

void setup() {
  Serial.begin(9600);
  
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

void loop() {
  h.serve(); // listen to the frontend commands
  
  delay(10);
}
