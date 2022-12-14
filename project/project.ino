// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIBRARIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "body.h"
#include "html510.h"
#include "Libraries/ArduinoJson.h"
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "vive510.h"
#include <math.h>
#include <esp_now.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ESPNOW SENDER SETUP: copied from canvas~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define CHANNEL 1                  // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0x84,0xF7,0x03,0xA8,0xBE,0x30} // receiver MAC address (last digit should be even for STA)

esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78}, 
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};

//game sender to required staff: taken from game-sender.ino code
void pingstaff() {
  uint8_t teamNum = 30;
  esp_now_send(staffcomm.peer_addr, &teamNum, 1);
}

//send message to staff ESP32 once per second with vive XY location
void sendXY(int team, int robotX, int robotY) {
  char msg[13];
  sprintf(msg, "%2d:%4d,%4d", team, robotX, robotY);
  esp_now_send(staffcomm.peer_addr, (uint8_t *) msg, 13);
}

// Wifi
const char *ssid = "TEAM HAWAII WIFI";
const char *password = "borabora";
HTML510Server h(80);

#define TIME_OF_FLIGHT_0_DEGREES_SCL_GPIO 1
#define TIME_OF_FLIGHT_0_DEGREES_SDA_GPIO 2

// Vive : //define int values to each of the coordinates the vive function returns
#define LEFT_X 1
#define LEFT_Y 2
#define RIGHT_X 3
#define RIGHT_Y 4

// PWM channels for ledC
#define PWM_CH_A 0
#define PWM_CH_B 1
#define PWM_CH_C 2
#define PWM_CH_D 3

// motor movement control pins
// FL
#define MOTOR_A_GPIO_ONE 8
#define MOTOR_A_GPIO_TWO 3
// FR
#define MOTOR_B_GPIO_ONE 10
#define MOTOR_B_GPIO_TWO 9
// BL
#define MOTOR_C_GPIO_ONE 14
#define MOTOR_C_GPIO_TWO 13
// BR
#define MOTOR_D_GPIO_ONE 12
#define MOTOR_D_GPIO_TWO 11

#define INFRARED_RECEIVER_GPIO 33
// #define INFRARED_RECEIVER_23HZ_GPIO 21  // TODO: JD

#define VIVE_RIGHT_GPIO 20
#define VIVE_LEFT_GPIO 19

// encoder pins
#define ENCODER_GPIO_A 1
#define ENCODER_GPIO_B 2
#define ENCODER_GPIO_C 42
#define ENCODER_GPIO_D 41

// intitial direction of motors
// 1 is CW, 0 is CCW
#define DIRECTION_A 1
#define DIRECTION_B 1
#define DIRECTION_C 1
#define DIRECTION_D 0

// PWM setup
#define PWM_RES 12
#define PWM_FREQ 25

// time keeping variable
float tick = 0.0;

// time interval for slot calculation (in milliseconds)
#define TIME_INTERVAL 200

// PID setup
#define Kp .05
#define Ki 1.1

// Setup constants
#define ROBOT_WIDTH 27
#define ROBOT_HEIGHT 28
// Length of course: 357cm
// Width of course: 145cm

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CLASSES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Class for our Motors
class Motor
{
private:
  int pwm_channel;
  int move_gpio_one;
  int move_gpio_two;
  int encoder_gpio;
  int direction;
  int encoder_state;
  int old_encoder_state;
  int curr_slot;
  int old_slot;
  float integration_sum_PID;
  int current_direction;

public:
  Motor(int pwm_channel, int move_gpio_one, int move_gpio_two, int encoder_gpio, int direction)
  {
    this->pwm_channel = pwm_channel;
    this->move_gpio_one = move_gpio_one;
    this->move_gpio_two = move_gpio_two;
    this->encoder_gpio = encoder_gpio;
    this->direction = direction;

    init();
  }
  void init()
  {
    setDirection(1); // Set initial direction

    pinMode(this->encoder_gpio, INPUT); // encoder pin setup

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
      if (this->direction == 1)
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

  void setSpeed(int targetSpeed)
  {
    ledcWrite(this->pwm_channel, targetSpeed * 400);
  }

  void setDirection(int forward)
  {
    // Store current direction
    this->current_direction = forward;

    // set the direction that the motors moves
    if (forward == this->direction)
    { // FORWARD
      // Setting pin one to be the PWM pin
      ledcAttachPin(this->move_gpio_one, this->pwm_channel);
      ledcSetup(this->pwm_channel, PWM_FREQ, PWM_RES);

      // setting the other gpio (pin two) to be grounded
      pinMode(this->move_gpio_two, OUTPUT);
      digitalWrite(this->move_gpio_two, LOW);
    }
    else
    { // BACKWARD
      // Move Backward: setting pin two to be the PWM pin
      ledcAttachPin(this->move_gpio_two, this->pwm_channel);
      ledcSetup(this->pwm_channel, PWM_FREQ, PWM_RES);

      // setting the other gpio (pin one) to be grounded
      pinMode(this->move_gpio_one, OUTPUT);
      digitalWrite(this->move_gpio_one, LOW);
    }
  }

  int getDirection()
  {
    // TODO: READ DIRECTION
    return this->current_direction;
  }

  int getSpeed()
  {
    // TODO: READ SPEED
    return ledcRead(this->pwm_channel);
  }
};

Motor motorFrontLeftA(PWM_CH_A, MOTOR_A_GPIO_ONE, MOTOR_A_GPIO_TWO, ENCODER_GPIO_A, DIRECTION_A);
Motor motorBackLeftB(PWM_CH_B, MOTOR_B_GPIO_ONE, MOTOR_B_GPIO_TWO, ENCODER_GPIO_B, DIRECTION_B);
Motor motorFrontRightC(PWM_CH_C, MOTOR_C_GPIO_ONE, MOTOR_C_GPIO_TWO, ENCODER_GPIO_C, DIRECTION_C);
Motor motorBackRightD(PWM_CH_D, MOTOR_D_GPIO_ONE, MOTOR_D_GPIO_TWO, ENCODER_GPIO_D, DIRECTION_D);

// Class for our Infrared receivers
class InfraredReceiver
{
private:
  int IR_gpio;
  int signal_state_700;
  float frequency_700;
  float current_time_700;
  float first_low_time_700;
  int signal_state_23;
  float frequency_23;
  float current_time_23;
  float first_low_time_23;

  int signal_700_present;
  int signal_23_present;

public:
  InfraredReceiver(int IR_gpio)
  {
        this->IR_gpio = IR_gpio;

        init();
  }
    void init()
    {
        Serial.print("Starting IR sensor: ");

        // assigning input pin from IR sensor board
        pinMode(this->IR_gpio, INPUT);

        // 1 is if the signal is high, 0 is if its low
        this->signal_state_700 = 1;
        this->signal_state_23 = 1;

        // used to calculate frequency
        this->frequency_700;
        this->frequency_23;

        // both used for calculating frequency
        this->current_time_700;
        this->current_time_23;
        this->first_low_time_700;
        this->first_low_time_23;

        // storing what signals are present
        this->signal_700_present = 0;
        this->signal_23_present = 0;

        Serial.println("SUCCESS!");
    }

    //outputs a 1 if 700 Hz is found
    //outputs a 0 if a 700 Hz signal is not there
    //outputs a 2 if no output(calculating in between outputs of 1 and 0)
    void measure700()
    {
      current_time_700 = millis();
      //calculate length of half a period
      frequency_700 = current_time_700 - first_low_time_700;

      //read for low to high movement
      if(digitalRead(this->IR_gpio) && signal_state_700 == 0) {
        //signal is high
        signal_state_700 = 1;

        //period right length for 700Hz
        if (frequency_700 < 15) {
          //Serial.println("700Hz");
          signal_700_present = 1;
        }

      //looks for going from high to low
      } else if (digitalRead(this->IR_gpio) == 0) {
         //mark the time when the signal is first low
         if (signal_state_700) {
          first_low_time_700 = millis();
         }
        signal_state_700 = 0;
       } else if (digitalRead(this->IR_gpio)) {
        //signal is high
        signal_state_700 = 1;

        if (frequency_700 >= 15) {
          //no 700Hz signal
          signal_700_present = 0;
        }
       }
   }




    //outputs a 1 if 23 Hz is found
    //outputs a 0 if a 23 Hz signal is not there
    //outputs a 2 if no output(calculating in between outputs of 1 and 0)
    void measure23()
    {
      current_time_23 = millis();
      //calculate length of half a period
      frequency_23 = current_time_23 - first_low_time_23;

      //read for low to high movement
      if(digitalRead(this->IR_gpio) && signal_state_23 == 0) {
        //signal is high
        signal_state_23 = 1;

        //period right length for 23Hz
        if (frequency_23 > 15 && frequency_23 < 55) {
          //Serial.print("Freq: ");
          //Serial.println(frequency_23);
          signal_23_present = 1;
        }

      //looks for going from high to low
      } else if (digitalRead(this->IR_gpio) == 0) {
         //mark the time when the signal is first low
         if (signal_state_23) {
          first_low_time_23 = millis();
         }
        signal_state_23 = 0;
       } else if (digitalRead(this->IR_gpio)) {
        //signal is high
        signal_state_23 = 1;

        if (frequency_23 >= 55 || frequency_23 <= 15) {
          //no 23Hz signal
          //Serial.println("no 23Hz signal");
          signal_23_present = 0;
        }
       }
   }

   int readIR700() 
   {
      return signal_700_present;
   }

   int readIR23() 
   {
      return signal_23_present;
   }
};

// Class for a queue to get the approx. median.
class OrestisQueue
{
private:
  int data[20];
  int size;

public:
  OrestisQueue()
  {
      this->size = sizeof(data) / sizeof(int);
  }
    void init(int num) {
        for (int i = 0; i < size; i++) {
          data[i] = num; // add same value to all
        }
    }
    
    void append(int new_num) {
        for (int i = 0; i < size - 1; i++) {
          data[i] = data[i+1]; // value of data[1] goes to data[0]
        }
        data[size - 1] = new_num;
    }
    
    void print() {
        for (int i = 0; i < size; i++) {
              Serial.println(data[i]);
        }
    }
    
    int getSpecialMedian() {
        // Duplicate array
        int data_sorted[size];
        for (int i = 0; i < size; i++) {
            data_sorted[i] = data[i];
        }

        // Sort
        bool swapped;
        do {
          swapped = false;
          for (int i = 1; i < size; i++) {
            if (data_sorted[i] < data_sorted[i - 1]) {
              int temp = data_sorted[i - 1];
              data_sorted[i - 1] = data_sorted[i];
              data_sorted[i] = temp;
              swapped = true;
            }
          }
        } while (swapped);
        
        // print results
//        for (int i = 0; i < size; i++) {
//              Serial.println(data_sorted[i]);
//        }
//        
        // Calculate average
        int sum = 0;
        for (int i = 0; i < size; i++) {
              sum = sum + data_sorted[i];
        }
        int average = sum / size;
        
        // Get average of middle X% numbers
        int avg_size = 0.5 * size;
        int special_sum = 0;
        for (int i = (size - avg_size) / 2; i < (size - avg_size) / 2 + avg_size; i++) {
//            Serial.println(data_sorted[i]);
            special_sum = special_sum + data_sorted[i];
        }
        
        return special_sum / avg_size;
    }
};

// Class for our Time Of Flight distance sensors
class TimeOfFlight
{
private:
  int degrees_pointing;
  int sda_gpio;
  int scl_gpio;
  OrestisQueue data_history;
  SFEVL53L1X distanceSensor;
  int first_calculation;

public:
  TimeOfFlight(int degrees_pointing, int sda_gpio, int scl_gpio)
  {
    this->degrees_pointing = degrees_pointing;
    this->sda_gpio = sda_gpio;
    this->scl_gpio = scl_gpio;
  }

  void init()
    {      
      Wire.begin(this->sda_gpio, this->scl_gpio);

      // I2C TimeOfFlight Sensor SETUP
      Serial.print("Starting distance sensor: ");
      delay(5000);

      if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
      {
        Serial.print("failing... ");
        while (1)
          ;
      }
      Serial.println("SUCCESS!");
      distanceSensor.setDistanceModeShort();
      // distanceSensor.setDistanceModeLong();
      
      // Creating history
      this->data_history.init(1000);

      first_calculation = 1; // intialize
  }

  void calculateDistance()
    { 

      if (first_calculation) {
        distanceSensor.startRanging(); // Start ranging! ISSUEEEE
        first_calculation = 0;
      }

      if (distanceSensor.checkForDataReady()) { // The signal came back
        int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor

        distanceSensor.clearInterrupt();
        distanceSensor.stopRanging();

        if (distance != 0) { // Not possible to read 0. It's an error.
          data_history.append(distance);
        }

        distanceSensor.startRanging(); // Start ranging again!
      }
    }

    int getDistance() {
      return this->data_history.getSpecialMedian();
    }
};

// Class for our Vive sensors
class ViveSensor
{
private:
  int is_the_left_sensor;
  int gpio;
  int x_coordinate;
  int y_coordinate;
  Vive510 viveObject; // Creating the object with dummy pin which we'll change later! Don't worry!

public:
  ViveSensor()
  {
    this->x_coordinate = 0;
    this->y_coordinate = 0;
  }
    void init(int is_the_left_sensor, int gpio)
    {
      this->is_the_left_sensor = is_the_left_sensor;
      this->gpio = gpio;

      viveObject.begin(this->gpio); // Sets pin as input
    }

    int getCoordinateHelper(int is_x){
      if (is_x) {
        return viveObject.xCoord();
      } else {
        return viveObject.yCoord();
      }
    }
    
    // Pass in whether you're asking for X (1) or Y (0) coordinate
    int readCoordinate(int is_x)
    {
      int raw_coordinate = 0; // If shit goes to hell, the value is -1.

      if (viveObject.status() == VIVE_RECEIVING)
      {
        raw_coordinate = getCoordinateHelper(is_x);
      }
      else
      {
        switch (viveObject.sync(5)) // We didn't get a read. Repeats X times syncing.
        {
            break;
            case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
              Serial.println("Weak signal");
              break;
            default:
            case VIVE_NO_SIGNAL: // nothing detected
              Serial.println("No signal");
        }
      }

      return raw_coordinate;
    }
};

// Class for our Time Of Flight distance sensors
class ViveMegaClass
{
private:
  ViveSensor LeftSensor;
  ViveSensor RightSensor;
  
  OrestisQueue left_x_coordinate; 
  OrestisQueue left_y_coordinate;
  OrestisQueue right_x_coordinate;
  OrestisQueue right_y_coordinate;

  int left_vive_gpio;
  int right_vive_gpio;

public:
  ViveMegaClass(int left_vive_gpio, int right_vive_gpio)
  {
      this->left_vive_gpio = left_vive_gpio;
      this->right_vive_gpio = right_vive_gpio;
  }

  void init() {
      Serial.print("Starting Vive x2 sensors: ");

      this->LeftSensor.init(1, this->left_vive_gpio);
      this->RightSensor.init(0, this->right_vive_gpio);

      this->left_x_coordinate.init(4000);
      this->left_y_coordinate.init(4000);
      this->right_x_coordinate.init(4000);
      this->right_y_coordinate.init(4000);

      Serial.println("SUCCESS!");
  }

  void calculateCoordinates() {
    // Read X,Y from each sensor
    int left_x = LeftSensor.readCoordinate(1); // read Left X
    int left_y = LeftSensor.readCoordinate(0); // read Left Y

    int right_x = RightSensor.readCoordinate(1); // read Right X
    int right_y = RightSensor.readCoordinate(0); // read Right Y

    // If valid, update
    if (hasValidCoordinates(left_x, left_y, right_x, right_y)) {
      this->left_x_coordinate.append(left_x);
      this->left_y_coordinate.append(left_y);
      this->right_x_coordinate.append(right_x);
      this->right_y_coordinate.append(right_y);
    }
  }

  int getLeftX(){
    return this->left_x_coordinate.getSpecialMedian();
  }

  int getLeftY(){
    return this->left_y_coordinate.getSpecialMedian();
  }

  int getRightX(){
    return this->right_x_coordinate.getSpecialMedian();
  }

  int getRightY(){
    return this->right_y_coordinate.getSpecialMedian();
  }

  int hasValidCoordinates(int left_x, int left_y, int right_x, int right_y)
  {
    if (left_x < 1000 || left_x > 7000 || right_x < 1000 || right_x > 7000) { // X out of bounds
      return 0;
    }

    if (left_y < 1000 || left_y > 7000 || right_y < 1000 || right_y > 7000) { // U out of bounds
      return 0;
    }

    int distance_between_vive_sensors = (int) sqrt(pow(left_x - right_x, 2) + pow(left_y - right_y, 2)); // =404 calculated
    if (distance_between_vive_sensors < 350 || distance_between_vive_sensors > 450)
    { // U out of bounds
      Serial.println("Distance between sensors is too small or too big: " + String(distance_between_vive_sensors));
      return 0;
    }
    
    // If passed all checks, it's valid!
    return 1;
  }
};

ViveMegaClass OneVive(VIVE_LEFT_GPIO, VIVE_RIGHT_GPIO);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void println(int x)
{
  Serial.println(x);
}

void setAllDirections(int direction_A, int direction_B, int direction_C, int direction_D)
{
  motorFrontLeftA.setDirection(direction_A);
  motorBackLeftB.setDirection(direction_B);
  motorFrontRightC.setDirection(direction_C);
  motorBackRightD.setDirection(direction_D);
}

void setAllMotorSpeeds(int speed_A, int speed_B, int speed_C, int speed_D)
{
  motorFrontLeftA.setSpeed(speed_A);
  motorBackLeftB.setSpeed(speed_B);
  motorFrontRightC.setSpeed(speed_C);
  motorBackRightD.setSpeed(speed_D);
}

// subroutine for turning each motor in the correct direction given the move_degree and look_direction
void drive(int move_degrees, int look_direction, int speed)
{

  // STOP
  if (move_degrees == -1 && look_direction == 0)
  {
    setAllMotorSpeeds(0, 0, 0, 0);
    Serial.println("STOP");
    return;
  }

  // TURN RIGHT
  if (look_direction == 1)
  { // look right: rotate CW;
    setAllDirections(1, 1, 0, 0);
    setAllMotorSpeeds(speed, speed, speed, speed);
    Serial.println("LOOKING right");
    return;
  }

  // TURN LEFT
  if (look_direction == -1)
  { // look left: rotate CCW;
    setAllDirections(0, 0, 1, 1);
    setAllMotorSpeeds(speed, speed, speed, speed);
    Serial.println("LOOKING left");
    return;
  }

  // MOVE in XY

  // Calculate speed of Back Left motor B and Front Right motor C
  int speed_BL_B_and_FR_C;
  switch (move_degrees)
  {
  case 0 ... 90:
    speed_BL_B_and_FR_C = speed - (int)speed * (float)move_degrees / 45.0;
    break;
  case 91 ... 180:
    speed_BL_B_and_FR_C = speed * -1;
    break;
  case 181 ... 270:
    speed_BL_B_and_FR_C = (int)speed * (float)(move_degrees - 180) / 45.0 - speed;
    break;
  case 271 ... 360:
    speed_BL_B_and_FR_C = speed;
    break;
  }

  // Calculate speed of Front Left motor A and Back Right motor D
  int speed_FL_A_and_BR_D;
  switch (move_degrees)
  {
  case 0 ... 90:
    speed_FL_A_and_BR_D = speed;
    break;
  case 91 ... 180:
    speed_FL_A_and_BR_D = speed - (int)speed * (float)(move_degrees - 90) / 45.0;
    break;
  case 181 ... 270:
    speed_FL_A_and_BR_D = speed * -1;
    break;
  case 271 ... 360:
    speed_FL_A_and_BR_D = (int)speed * (float)(move_degrees - 270) / 45.0 - speed;
    break;
  }

  setAllDirections(speed_FL_A_and_BR_D > 0, speed_BL_B_and_FR_C > 0, speed_BL_B_and_FR_C > 0, speed_FL_A_and_BR_D > 0);
  setAllMotorSpeeds(abs(speed_FL_A_and_BR_D), abs(speed_BL_B_and_FR_C), abs(speed_BL_B_and_FR_C), abs(speed_FL_A_and_BR_D));

  // switch (move_degrees)
  // {
  // case 0: // Forward
  //   setAllDirections(1, 1, 1, 1);
  //   setAllMotorSpeeds(speed, speed, speed, speed);
  //   Serial.println("0 degrees: MOVE N");
  //   break;

  // case 45: // NE
  //   setAllDirections(1, 1, 1, 1);
  //   setAllMotorSpeeds(speed, 0, 0, speed);
  //   Serial.println("45 degrees: MOVE NE");
  //   break;

  // case 90: // RIGHT; East
  //   setAllDirections(1, 0, 0, 1);
  //   setAllMotorSpeeds(speed, speed, speed, speed);
  //   Serial.println("90 degrees: MOVE east");
  //   break;

  // case 135: // SE
  //   setAllDirections(0, 0, 0, 0);
  //   setAllMotorSpeeds(0, speed, speed, 0);
  //   Serial.println("135 degrees: MOVE SE");
  //   break;

  // case 180: // SOUTH
  //   setAllDirections(0, 0, 0, 0);
  //   setAllMotorSpeeds(speed, speed, speed, speed);
  //   Serial.println("180 degrees: MOVE S");
  //   break;

  // case 225: // SW
  //   setAllDirections(0, 0, 0, 0);
  //   setAllMotorSpeeds(speed, 0, 0, speed);
  //   Serial.println("225 degrees: MOVE SW");
  //   break;

  // case 270: // WEST (left)
  //   setAllDirections(0, 1, 1, 0);
  //   setAllMotorSpeeds(speed, speed, speed, speed);
  //   Serial.println("270 degrees: MOVE W");
  //   break;

  // case 315: // NW
  //   setAllDirections(1, 1, 1, 1);
  //   setAllMotorSpeeds(0, speed, speed, 0);
  //   Serial.println("315 degrees: MOVE NW");
  //   break;
  // }
}

// int getRobotOrientationDegrees(ViveSensor ViveRight, ViveSensor ViveLeft)
// {
//   // TODO: Sophie
//   return 0;
// }

// int getRobotCoordinateX(ViveSensor ViveRight, ViveSensor ViveLeft)
// {
//   // TODO: Sophie
//   return 0;
// }

// int getRobotCoordinateY(ViveSensor ViveRight, ViveSensor ViveLeft)
// {
//   // TODO: Sophie
//   return 0;
// }

InfraredReceiver InfraredReceiverCenter(INFRARED_RECEIVER_GPIO);

TimeOfFlight TimeOfFlightDegrees0(0, TIME_OF_FLIGHT_0_DEGREES_SDA_GPIO, TIME_OF_FLIGHT_0_DEGREES_SCL_GPIO);

void handleRoot()
{
  h.sendhtml(body);
}

void handleKeyPressed()
{
  String keys = h.getText();
  keys.replace("%22", "\""); // Reconstruct quotations "" in JSON
  h.sendplain(keys);
  // Serial.println("Keys Pressed: " + keys);

  StaticJsonBuffer<600> jsonBuffer;                    // Create the JSON buffer
  JsonObject &keysJSON = jsonBuffer.parseObject(keys); // Parse and create json object

  // Look variables
  int look_left = keysJSON["37"];
  int look_right = keysJSON["39"];
  if (look_right == look_left == 1)
  { // If both look left+right --> cancel out
    look_right = 0;
    look_left = 0;
  }
  int look_direction = 0;
  if (look_left || look_right)
  {
    look_direction = -look_left + look_right;
  }
  if (look_direction != 0)
  {
    Serial.print("Look: ");
    Serial.print(look_direction);
    Serial.println(" direction.");
  }

  // Move variables
  int move_up = keysJSON["87"];
  int move_down = keysJSON["83"];
  int move_left = keysJSON["65"];
  int move_right = keysJSON["68"];
  if (move_up == move_down == 1)
  { // If both look up+down --> cancel out
    move_up = 0;
    move_down = 0;
  }
  if (move_right == move_left == 1)
  { // If both look left+right --> cancel out
    move_right = 0;
    move_left = 0;
  }
  int move_degrees = -1;
  if (move_up || move_down)
  {
    move_degrees = move_up * 0 + move_down * 180;
  }
  if (move_right)
  {
    move_degrees = 90 - move_up * 45 + move_down * 45;
  }
  if (move_left)
  {
    move_degrees = 270 - move_down * 45 + move_up * 45;
  }
  if (move_degrees != -1)
  {
    Serial.print("Move: ");
    Serial.print(move_degrees);
    Serial.println(" degrees.");
  }

  // Speed variables - biggest speed rules
  static int current_speed = 5;
  int speed_0 = keysJSON["48"];
  int speed_1 = keysJSON["49"];
  int speed_2 = keysJSON["50"];
  int speed_3 = keysJSON["51"];
  int speed_4 = keysJSON["52"];
  int speed_5 = keysJSON["53"];
  int speed_6 = keysJSON["54"];
  int speed_7 = keysJSON["55"];
  int speed_8 = keysJSON["56"];
  int speed_9 = keysJSON["57"];

  // Update current speed
  if (speed_0 || speed_1 || speed_2 || speed_3 || speed_4 || speed_5 || speed_6 || speed_7 || speed_8 || speed_9)
  { // If any 0-9 key is pressed
    current_speed = max(max(max(speed_0 * 0, speed_1 * 1), max(speed_2 * 2, speed_3 * 3)), max(max(max(speed_4 * 4, speed_5 * 5), max(speed_6 * 6, speed_7 * 7)), max(speed_8 * 8, speed_9 * 9)));
  }

  drive(move_degrees, look_direction, current_speed);
}

void handleStateUpdate()
{

  // Construct JSON to send to frontend
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();

  String response_json = "{  \
        'status': 'success', \
        'skip_setup': false, \
        'setup': { \
          'robot_width': " +
                         String(ROBOT_WIDTH) + ", \
          'robot_height': " +
                         String(ROBOT_HEIGHT) + ", \
          'game_width': 366, \
          'game_height': 152 \
        }, \
        'robot': { \
          'x': 100, \
          'y': 50, \
          'raw_left_x': " +
                         String(OneVive.getLeftX()) + ", \
          'raw_right_x': " +
                         String(OneVive.getRightX()) + ", \
          'raw_left_y': " +
                         String(OneVive.getLeftY()) + ", \
          'raw_right_y': " +
                         String(OneVive.getRightY()) + ", \
          'degrees': 0 \
        }, \
        'IR_sensor': { \
          'beacon_700Hz': " + String(InfraredReceiverCenter.readIR700()) + ", \
          'beacon_23Hz': " + String(InfraredReceiverCenter.readIR23()) + " \
        }, \
        'ToF_sensor': { \
          'distance': [" +
                         String(TimeOfFlightDegrees0.getDistance()) + "], \
          'degrees': [0], \
          'time': [0] \
        }, \
        'motors': { \
          'power': { \
            'front_left_A': " +
                         String(motorFrontLeftA.getSpeed()) + ", \
            'back_left_B': " +
                         String(motorBackLeftB.getSpeed()) + ", \
            'front_right_C': " +
                         String(motorFrontRightC.getSpeed()) + ", \
            'back_right_D': " +
                         String(motorBackRightD.getSpeed()) + " \
          }, \
          'direction': { \
            'front_left_A': " +
                         String(motorFrontLeftA.getDirection()) + ", \
            'back_left_B': " +
                         String(motorBackLeftB.getDirection()) + ", \
            'front_right_C': " +
                         String(motorFrontRightC.getDirection()) + ", \
            'back_right_D': " +
                         String(motorBackRightD.getDirection()) + " \
          } \
        } \
      }";

  response_json.replace(" ", "");

  // Serial.print("Sending to web this: ");
  // Serial.println(response_json);

  h.sendplain(response_json);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup()
{

  // Serial
  Serial.begin(115200);
  Serial.println(" ");

  // WIFI SETUP
  WiFi.mode(WIFI_STA);          // Set Mode to Access Point
  WiFi.softAP(ssid, password); // Define access point SSID and its password

  IPAddress myIP(192, 168, 1, 161);                                                // Define my unique Static IP (from spreadsheet on slides)
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); // Define the (local IP, Gateway, Subnet mask)

  h.begin();
  h.attachHandler("/ ", handleRoot);
  h.attachHandler("/key_pressed?val=", handleKeyPressed);
  h.attachHandler("/get_updated_state", handleStateUpdate);

  // Time Of Flight sensor
  TimeOfFlightDegrees0.init();
  delay(2000);

  OneVive.init();

  // ESP NOW
  esp_now_init();
  esp_now_add_peer(&staffcomm);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void specialDelay(int milliseconds) {
  int beginTime = millis();
  int nowTime = beginTime;

  while (millis() - beginTime < milliseconds) {
    doInLoop();
  }
}

int serveClock = millis();
int viveClock = millis();
int distanceClock = millis();
int commsClock = millis();

void doInLoop() {
  Serial.println("Start: " + String(millis()));
  OneVive.calculateCoordinates(); // 0ms
  Serial.println(millis());
  // h.serve(); // 47ms
  Serial.println(millis());
  // if (millis() % 300 < 10) {
  // TimeOfFlightDegrees0.calculateDistance(); // 34ms
  // }
  Serial.println(millis());
  InfraredReceiverCenter.measure700(); // 0ms
  Serial.println(millis());
  InfraredReceiverCenter.measure23(); // 0ms
  // Serial.println(millis());
  // pingstaff(); // 0ms
  // Serial.println(millis());
  // sendXY(30, 1000, 1000); // 0ms
  // Serial.println(millis());
  // Serial.println(millis());
  // Serial.println();

  if (millis() - serveClock > 10) {
    h.serve(); // 47ms
    serveClock = millis();
  }

  if (millis() - distanceClock > 50) {
    TimeOfFlightDegrees0.calculateDistance(); // 34ms
    distanceClock = millis();
  }

  if (millis() - commsClock > 1000) {
    // pingstaff(); // 0ms
    sendXY(30, 1212, 5637); // 0ms
    commsClock = millis();
  }
}

void loop()
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LEFT CORNER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // drive(0, 0, 8); // move straight
  // specialDelay(900);

  // drive(90, 0, 8); // jam right
  // specialDelay(1500);

  // drive(0, 0, 5); // move straight
  // specialDelay(400);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(100);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 5); // move straight
  // specialDelay(300);

  // drive(-1, 1, 5); // turn right 45 degrees
  // specialDelay(300);

  // drive(0, 0, 8); // move straight
  // specialDelay(1200);

  // drive(180, 0, 8); // drive back a bit
  // specialDelay(500);

  // drive(-1, 1, 8); // turn right 180 degrees
  // specialDelay(1800);

  // drive(180, 0, 8); // jam back
  // specialDelay(1500);

  // drive(270, 0, 8); // shift right
  // specialDelay(2000);

  // drive(180, 0, 8); // jam back
  // specialDelay(1500);

  // drive(270, 0, 8); // shift right
  // specialDelay(3000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(1000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(1000);

  // drive(0, 0, 8); // move straight
  // specialDelay(3000);

  // drive(90, 0, 8); // shift RIGHT
  // specialDelay(3500);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(270, 0, 8); // shift right
  // specialDelay(2000);

  // drive(-1, 0, 0);
  // specialDelay(20000000);
  


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RIGHT CORNER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  drive(0, 0, 8); // move straight
  specialDelay(900);

  drive(270, 0, 8); // jam left
  specialDelay(1500);

  drive(0, 0, 5); // move straight
  specialDelay(400);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 5); // move straight
  specialDelay(300);

  drive(-1, -1, 5); // turn right 45 degrees
  specialDelay(300);

  drive(0, 0, 8); // move straight
  specialDelay(1200);

  drive(180, 0, 8); // drive back a bit
  specialDelay(500);

  drive(-1, -1, 8); // turn right 180 degrees
  specialDelay(1800);

  drive(180, 0, 8); // jam back
  specialDelay(1500);

  drive(90, 0, 8); // shift right
  specialDelay(2000);

  drive(180, 0, 8); // jam back
  specialDelay(1500);

  drive(90, 0, 8); // shift right
  specialDelay(3000);

  drive(0, 0, 8); // move straight
  specialDelay(2000);

  drive(90, 0, 8); // shift right
  specialDelay(1000);

  drive(0, 0, 8); // move straight
  specialDelay(2000);

  drive(90, 0, 8); // shift right
  specialDelay(1000);

  drive(0, 0, 8); // move straight
  specialDelay(2200);

  drive(270, 0, 8); // shift left TO GO TO THE BOX!!!
  specialDelay(3000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(90, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(90, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(90, 0, 8); // shift right
  // specialDelay(2000);

  // drive(0, 0, 8); // move straight
  // specialDelay(2000);

  // drive(90, 0, 8); // shift right
  // specialDelay(2000);

  drive(-1, 0, 0);
  specialDelay(20000000);
}
