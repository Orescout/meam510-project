// Libraries to import
#include "body.h"
#include "html510.h"
#include "Libraries/ArduinoJson.h"

HTML510Server h(80);

// https://medesign.seas.upenn.edu/index.php/Guides/ESP32-pins

// Wifi
const char* ssid = "TEAM HAWAII WIFI";
const char* password = "borabora";

//PWM output pins
#define MOTOR_PWM_GPIO_A 4 // Switched from 4 11/14
#define MOTOR_PWM_GPIO_B 5 // Switched from 5 11/14
#define MOTOR_PWM_GPIO_C 6
#define MOTOR_PWM_GPIO_D 7

//PWM channels for ledC
#define PWM_CH_A 0
#define PWM_CH_B 1
#define PWM_CH_C 2
#define PWM_CH_D 3

//direction pins
//FL
#define DIRECTION_MOTOR_A_GPIO_ONE 12 //SWITCHED
#define DIRECTION_MOTOR_A_GPIO_TWO 11 //SWITCHED
//FR
#define DIRECTION_MOTOR_B_GPIO_ONE 14 //SWITCHED
#define DIRECTION_MOTOR_B_GPIO_TWO 13 //SWITCHED
//BL
#define DIRECTION_MOTOR_C_GPIO_ONE 10
#define DIRECTION_MOTOR_C_GPIO_TWO 9
//BR
#define DIRECTION_MOTOR_D_GPIO_ONE 3
#define DIRECTION_MOTOR_D_GPIO_TWO 8

//encoder pins
#define ENCODER_GPIO_A 1
#define ENCODER_GPIO_B 2
#define ENCODER_GPIO_C 42
#define ENCODER_GPIO_D 41

//intitial direction of motors
//1 is CW, 0 is CCW
#define DIR_A 0
#define DIR_B 1
#define DIR_C 1
#define DIR_D 1

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
    int motor_pwm_gpio;
    int pwm_ch;
    int direction_gpio_A;
    int direction_gpio_B;
    int encoder_gpio;
    int dir;
    int encoder_state;
    int old_encoder_state;
    int curr_slot;
    int old_slot;
    float integration_sum_PID;

public:
    Motor(int motor_pwm_gpio, int pwm_ch, int direction_gpio_A, int direction_gpio_B, int encoder_gpio, int dir)
    {
        this->motor_pwm_gpio = motor_pwm_gpio;
        this->pwm_ch = pwm_ch;
        this->direction_gpio_A = direction_gpio_A;
        this->direction_gpio_B = direction_gpio_B;
        this->encoder_gpio = encoder_gpio;
        this->dir = dir;

        init();
    }
    void init()
    {
        // ledC setup
        ledcAttachPin(this->motor_pwm_gpio, this->pwm_ch);
        ledcSetup(this->pwm_ch, PWM_FREQ, PWM_RES);

        // direction pin setup
        pinMode(this->direction_gpio_A, OUTPUT);
        pinMode(this->direction_gpio_B, OUTPUT);

        // set initial directions of GPIO direction pins to be HIGH and LOW
        digitalWrite(this->direction_gpio_A, LOW);
        digitalWrite(this->direction_gpio_B, HIGH);

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

    // PID goes here!!!!!
    void go(int targetSpeed)
    {
      //ST EDIT
      int temp_speed = 1500; //dummy pwm to pass to all motors unless passing in 0
      if (targetSpeed == 0) { //case where we want motor to stop
        ledcWrite(this->pwm_ch, targetSpeed); // TODO: CHANGE IF SHIT GOES TO HELL
      } else {
        ledcWrite(this->pwm_ch, temp_speed);
      }
        // // this is the number of slots per time interval it is moving
        // int currentSpeed = this->getVel();

        // Serial.print("Target: ");
        // Serial.println(targetSpeed);
        // Serial.print("Current: ");
        // Serial.println(currentSpeed);

        // // PID will be here
        // int error = targetSpeed - currentSpeed; //[slots/timeinterval]
        // float proportional = Kp * error;
        // Serial.print("proportional: ");
        // Serial.println(proportional);
        // Serial.print("Existing integration sum: ");
        // Serial.println(integration_sum_PID);
        // this->integration_sum_PID = Ki * (this->integration_sum_PID + error * TIME_INTERVAL / 1000);
        // Serial.print("integration_sum_PID: ");
        // Serial.println(integration_sum_PID);

        // // total_PI must be between 0 and 1
        // float total_PI = this->integration_sum_PID + proportional;
        // Serial.print("total_PI: ");
        // Serial.println(total_PI);
        // // TODO round it instead of cast
        // if (total_PI < (float)0.0)
        // {
        //     total_PI = 0.0;
        // }
        // else if (total_PI > (float)1.0)
        // {
        //     total_PI = 1.0;
        // }
        // // int new_duty_cycle = (int) min(max(total_PI, (float) 0.0), (float) 1.0) * (float) 4095.0;
        // int new_duty_cycle = (int)(total_PI * (float)4095.0);
        // Serial.print("DC: ");
        // Serial.println(new_duty_cycle);

        // // this is where the speed of the motor is changed based off of duty cycle 0-4095
        // ledcWrite(this->pwm_ch, new_duty_cycle);
    }

    // change the direction that the motor is going
    // forwards or backwards
    void changeDirection(int new_dir)
    {
        // set the direction that the motors moves
        if (new_dir == 1)
        {
            // CW; FORWARD
            digitalWrite(this->direction_gpio_A, LOW);
            digitalWrite(this->direction_gpio_B, HIGH);
        }
        else
        {
            // CCW; BACKWARD
            digitalWrite(this->direction_gpio_A, HIGH);
            digitalWrite(this->direction_gpio_B, LOW);
        }
    }
};

// Creating all the motor objects
Motor motorFrontLeft(MOTOR_PWM_GPIO_A, PWM_CH_A, DIRECTION_MOTOR_A_GPIO_ONE, DIRECTION_MOTOR_A_GPIO_TWO, ENCODER_GPIO_A, DIR_A);
Motor motorBackLeft(MOTOR_PWM_GPIO_B, PWM_CH_B, DIRECTION_MOTOR_B_GPIO_ONE, DIRECTION_MOTOR_B_GPIO_TWO, ENCODER_GPIO_B, DIR_B);
Motor motorFrontRight(MOTOR_PWM_GPIO_C, PWM_CH_C, DIRECTION_MOTOR_C_GPIO_ONE, DIRECTION_MOTOR_C_GPIO_TWO, ENCODER_GPIO_C, DIR_C);
Motor motorBackRight(MOTOR_PWM_GPIO_D, PWM_CH_D, DIRECTION_MOTOR_D_GPIO_ONE, DIRECTION_MOTOR_D_GPIO_TWO, ENCODER_GPIO_D, DIR_D);

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
  if (speed_0 || speed_1 || speed_2 || speed_3 || speed_4 || speed_5 || speed_6 || speed_7 || speed_8 || speed_9) { // If any 0-9 key is pressed
    // Update current speed
    current_speed = max(max(max(speed_0*0, speed_1*1), max(speed_2*2, speed_3*3)), max(max(max(speed_4*4, speed_5*5), max(speed_6*6, speed_7*7)), max(speed_8*8, speed_9*9)));
  }
  //Serial.print("Current speed: "); println(current_speed);
  //Serial.println(" ");

  drive(move_degrees, look_direction, current_speed);
}
//helper subroutine for drive()
void turnOnAllMotors(int speed) {
  motorFrontLeft.go(speed);
  motorBackLeft.go(speed);
  motorFrontRight.go(speed);
  motorBackRight.go(speed);
}
//subroutine for turning each motor in the correct direction given the move_degree and look_direction
void drive(int move_degrees, int look_direction, int speed) {

  //turn off motors if move_deg = -1 and look_dir = 0
  if (move_degrees == -1 && look_direction == 0) {
      motorFrontLeft.go(0);
      motorFrontRight.go(0);
      motorBackLeft.go(0);
      motorBackRight.go(0);
      Serial.println("STOPPED");
  }

  //handle case where both look and move is on: only look if move not on
  if (look_direction == 1  && move_degrees == -1) { //look right: rotate CW; 
    motorFrontLeft.changeDirection(1);
    motorBackLeft.changeDirection(0); //SWITCHED
    motorFrontRight.changeDirection(1); //SWITCHED
    motorBackRight.changeDirection(1); //SWITCHED

    //turn on motors 
    turnOnAllMotors(speed);
    Serial.println("LOOKING right");
  }
  if (look_direction == -1 && move_degrees == -1) { //look left: rotate CCW;
    motorFrontLeft.changeDirection(0); 
    motorBackLeft.changeDirection(1);//SWITCHED
    motorFrontRight.changeDirection(0);//SWITCHED
    motorBackRight.changeDirection(0);//SWITCHED

    //turn on motors 
    turnOnAllMotors(speed);
    Serial.println("LOOKING left");
  }

  switch (move_degrees) {
    case 0: // move forward
      motorFrontLeft.changeDirection(0);
      motorBackLeft.changeDirection(0);//SWITCHED
      motorFrontRight.changeDirection(0);//SWITCHED
      motorBackRight.changeDirection(0);//SWITCHED

      turnOnAllMotors(speed); //turn on motors

      Serial.println("0 degrees: MOVE N");
      break;
    case 45: //45 deg right, NE
      motorFrontRight.changeDirection(0); //SWITCHED
      motorBackLeft.changeDirection(0);//SWITCHED

      motorFrontRight.go(speed);
      motorBackLeft.go(speed);
      motorFrontLeft.go(0);
      motorBackRight.go(0);

      Serial.println("45 degrees: MOVE NE");
      break;
    case 90: // RIGHT; East
      motorFrontLeft.changeDirection(1);
      motorBackLeft.changeDirection(1); //SWITCHED
      motorFrontRight.changeDirection(1);//SWITCHED
      motorBackRight.changeDirection(0);//SWITCHED

      turnOnAllMotors(speed);
      Serial.println("90 degrees: MOVE east");
      break;
    case 135: // SE
      motorFrontLeft.changeDirection(0);
      motorBackRight.changeDirection(1); //SWITCHED

      //turn on motors
      motorFrontLeft.go(speed);
      motorBackRight.go(speed);
      motorFrontRight.go(0);
      motorBackLeft.go(0);

      Serial.println("135 degrees: MOVE SE");
      break;
    case 180: //SOUTH
      motorFrontLeft.changeDirection(0);
      motorBackLeft.changeDirection(1);//SWITCHED
      motorFrontRight.changeDirection(1);//SWITCHED
      motorBackRight.changeDirection(1);//SWITCHED

      turnOnAllMotors(speed); //turn on motors

      Serial.println("180 degrees: MOVE S");
      break;
    case 225: //SW
      motorFrontRight.changeDirection(1);//SWITCHED
      motorBackLeft.changeDirection(1);//SWITCHED

      //turn on motors
      motorFrontRight.go(speed);
      motorBackLeft.go(speed);
      motorFrontLeft.go(0);
      motorBackRight.go(0);

      Serial.println("225 degrees: MOVE SW");
      break;
    case 270: //WEST (left)
      motorFrontLeft.changeDirection(0);
      motorBackLeft.changeDirection(0);//SWITCHED
      motorFrontRight.changeDirection(0);//SWITCHED
      motorBackRight.changeDirection(1);//SWITCHED

      turnOnAllMotors(speed); //turn on motors

      Serial.println("270 degrees: MOVE W");
      break;
    case 315: // NW
      motorFrontLeft.changeDirection(1);
      motorBackRight.changeDirection(0); //SWITCHED
  

      //turn on motors
      motorFrontLeft.go(speed);
      motorBackRight.go(speed);
      motorFrontRight.go(0);
      motorBackLeft.go(0);

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

  // //SPEED Changing
  // //reading potentiometer
  // int target_speed = 9;

  // //change speed once enough time has passed
  // if(millis()-tick > TIME_INTERVAL) {
  //   tick = millis();
  //   motorA.changeSpeed(target_speed);
  //   //motorB.changeSpeed(target_speed);
  //   //motorC.changeSpeed(target_speed);
  //   //motorD.changeSpeed(target_speed);
  // }
  // //keep track of how many slots are going by
  // motorA.updateEncoder();
  // //motorB.updateEncoder();
  // //motorC.updateEncoder();
  // //motorD.updateEncoder();

  // //example of how changing direction will work
  // motorA.changeDirection(1);
  
  delay(10);
  // Serial.println(" ");
}