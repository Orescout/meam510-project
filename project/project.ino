// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LIBRARIES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "body.h"
#include "html510.h"
#include "Libraries/ArduinoJson.h"
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "vive510.h"
#include <math.h>

// Hello!

HTML510Server h(80);

// Wifi
const char *ssid = "TEAM HAWAII WIFI";
const char *password = "borabora";

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
// class InfraredReceiver
// {
// private:
//   int IR_gpio;

// public:
//   InfraredReceiver(int IR_gpio)
//   {
//         this->IR_gpio = IR_gpio;

//         init();
//   }
//     void init()
//     {
//       //assigning input pin from IR sensor board
//       pinMode(this->IR_gpio, INPUT);

//     }

//     //reads given frequency input
//     //frequency_search is the desired frequency(23 or 700)
//     //outputs a 1 if that frequency_search you want is found, 0 if not
//     int read(int frequency_search)
//     {
//       //used to calculate frequency
//       float frequency;
//       //both used for calculating frequency
//       float current_time;
//       float first_low_time;
//       //1 is if the signal is high, 0 is if its low
//       int signal_state = 1;

//       //calculating the frequency
//       for (int i = 1; i < 50; ++i) {
//         current_time = millis();

//         //read for a high signal
//         if(digitalRead(this->IR_gpio)) {
//             if (signal_state == 0) {
//               //calculate length of half a period
//               frequency = current_time - first_low_time;

//               //assign frequency based off of frequency
//               //700 Hz
//               if (frequency < 15 && frequency_search == 700) {
//                 return 1;
//               }
//               //23Hz
//               if (frequency > 15 && frequency < 50 && frequency_search == 23) {
//                 return 1;
//               }
//           }
//           signal_state = 1;

//         } else {
//           //mark the time when the signal is first low
//           if (signal_state) {
//             first_low_time = millis();
//           }
//           signal_state = 0;
//         }
//         delay(1);
//       }
//       //if no signal found return 0
//       return 0;
//     }
// };

// Class for our Time Of Flight distance sensors
class TimeOfFlight
{
private:
  int degrees_pointing;
  int sda_gpio;
  int scl_gpio;
  SFEVL53L1X distanceSensor;

public:
  TimeOfFlight(int degrees_pointing, int sda_gpio, int scl_gpio)
  {
    this->degrees_pointing = degrees_pointing;
    this->sda_gpio = sda_gpio;
    this->scl_gpio = scl_gpio;
  }
  void init()
  {
    // SFEVL53L1X distanceSensor; // I2C for TimeOfFlight Sensor

    Wire.begin(this->sda_gpio, this->scl_gpio);

    // I2C TimeOfFlight Sensor SETUP
    Serial.println("VL53L1X Qwiic Test");
    delay(5000);

    if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
    {
      Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor online!");

    distanceSensor.setDistanceModeShort();
    // distanceSensor.setDistanceModeLong();
  }

  int getDistance()
  {
    distanceSensor.startRanging(); // Write configuration bytes to initiate measurement

    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }

    int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    // Serial.print("Reading distance"); Serial.println(distance);

    return distance;
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
  ViveSensor(int is_the_left_sensor, int gpio)
  {
    this->is_the_left_sensor = is_the_left_sensor;
    this->gpio = gpio;
    this->x_coordinate = 0;
    this->y_coordinate = 0;
  }
    void init()
    {
      viveObject.begin(gpio); // Sets pin as input
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

ViveSensor ViveRightSensor(0, VIVE_RIGHT_GPIO);
ViveSensor ViveLeftSensor(1, VIVE_LEFT_GPIO);

// Class for our Time Of Flight distance sensors
// class ViveMegaClass
// {
// private:
//   ViveSensor LeftSensor;
//   ViveSensor RightSensor;
//   int left_x_coordinate;
//   int left_y_coordinate;
//   int right_x_coordinate;
//   int right_y_coordinate;
//   // int last_left_X_coordinates[10];


// public:
//   ViveMegaClass(ViveSensor LeftSensor, ViveSensor RightSensor)
//   {
//       this->LeftSensor = LeftSensor;
//       this->RightSensor = RightSensor;

//   }

//   void init() {
//     this->left_x_coordinate = 0;
//     this->left_y_coordinate = 0;
//     this->right_x_coordinate = 0;
//     this->right_y_coordinate = 0;
//   }

//   int calculateCoordinates() {
//     // Read X,Y from each sensor
//     int left_x = LeftSensor.readCoordinate(1); // read Left X
//     int left_y = LeftSensor.readCoordinate(0); // read Left Y

//     int right_x = LeftSensor.readCoordinate(1); // read Right X
//     int right_y = LeftSensor.readCoordinate(0); // read Right Y

//     // If valid, update
//     if (hasValidCoordinates(left_x, left_y, right_x, right_y)) {
//       this->left_x_coordinate = left_x;
//       this->left_y_coordinate = left_y;
//       this->right_x_coordinate = right_x;
//       this->right_y_coordinate = right_y;
//     }
//   }

//   int getLeftX(){
//     return this->left_x_coordinate;
//   }

//   int getLeftY(){
//     return this->left_y_coordinate;
//   }

//   int getRightX(){
//     return this->right_x_coordinate;
//   }

//   int getRightY(){
//     return this->right_y_coordinate;
//   }

//   int hasValidCoordinates(int left_x, int left_y, int right_x, int right_y)
//   {
//     if (left_x < 1000 || left_x > 7000 || right_x < 1000 || right_x > 7000) { // X out of bounds
//       return 0;
//     }

//     if (left_y < 1000 || left_y > 7000 || right_y < 1000 || right_y > 7000) { // U out of bounds
//       return 0;
//     }

//     int distance_between_vive_sensors = (int)sqrt(pow(left_x - right_x, 2) + pow(left_y - right_y, 2)); // =404 calculated
//     if (distance_between_vive_sensors < 350 || distance_between_vive_sensors > 450)
//     { // U out of bounds
//       Serial.println("Distance between sensors is too small");
//       return 0;
//     }
    
//     // If passed all checks, it's valid!
//     return 1;
//   }
// };

// ViveMegaClass OneVive(ViveLeftSensor, ViveRightSensor);

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

// InfraredReceiver InfraredReceiverCenter(INFRARED_RECEIVER_GPIO);

// SOPHIE CODE
// Vive510 viveRight(VIVE_RIGHT_GPIO);
// Vive510 viveLeft(VIVE_LEFT_GPIO);

// function to return median of 3 values
// int med3Filt(int a, int b, int c)
// {
//   int mid;
//   if ((a <= b) && (a <= c))
//   {
//     mid = (b <= c) ? b : c;
//   }
//   else if ((b <= a) && (b <= c))
//   {
//     mid = (a <= c) ? a : c;
//   }
//   else
//   {
//     mid = (a <= b) ? a : b;
//   }
//   return mid;
// }

// int lx = 0;
// int ly = 0;
// int rx = 0;
// int ry = 0;

// int getVive(int viveCoord)
// {
//   int avg_lx = 0;
//   int avg_ly = 0;
//   int avg_rx = 0;
//   int avg_ry = 0;
//   switch (viveCoord)
//   {
//   case LEFT_X:
//     if (viveLeft.status() == VIVE_RECEIVING)
//     {
//       // implement med filter
//       int r1 = viveLeft.xCoord();
//       int r2 = viveLeft.xCoord();
//       int r3 = viveLeft.xCoord();
//       avg_lx = med3Filt(r1, r2, r3);
//       Serial.printf("LeftX %d ", avg_lx);
//       return avg_lx;
//     }
//     else
//     {
//       switch (viveLeft.sync(5))
//       {
//         break;
//       case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
//         Serial.println("Left vive: weak signal");
//         break;
//       default:
//       case VIVE_NO_SIGNAL: // nothing detected
//         Serial.println("Left vive: no signal");
//       }
//     }
//     return avg_lx;
//     // TODO: DO FOR ALL 4 CASES
//     break;
//   case LEFT_Y:
//     Serial.print("VIVELEFT STATUS:");
//     Serial.println(viveLeft.status());
//     if (viveLeft.status() == VIVE_RECEIVING)
//     {
//       int r1 = viveLeft.yCoord();
//       int r2 = viveLeft.yCoord();
//       int r3 = viveLeft.yCoord();
//       avg_ly = med3Filt(r1, r2, r3);
//       Serial.printf("         LeftY %d ", avg_ly);
//       return avg_ly;
//     }
//     else
//     {
//       switch (viveLeft.sync(5))
//       {
//         break;
//       case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
//         Serial.println("Left vive: weak signal");
//         break;
//       default:
//       case VIVE_NO_SIGNAL: // nothing detected
//         Serial.println("Left vive: no signal");
//       }
//     }
//     return avg_lx;
//     break;
//   case RIGHT_X:
//     Serial.print("VIVERIGHT STATUS:");
//     Serial.println(viveRight.status());
//     if (viveRight.status() == VIVE_RECEIVING)
//     {
//       int r1 = viveRight.xCoord();
//       int r2 = viveRight.xCoord();
//       int r3 = viveRight.xCoord();
//       avg_rx = med3Filt(r1, r2, r3);
//       Serial.printf("                            RightX %d ", avg_rx);
//       return avg_rx;
//     }
//     else
//     {
//       switch (viveRight.sync(5))
//       {
//         break;
//       case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
//         Serial.println("Right vive: weak signal");
//         break;
//       default:
//       case VIVE_NO_SIGNAL: // nothing detected
//         Serial.println("Right vive: no signal");
//       }
//     }
//     return avg_rx;
//     // TODO: DO FOR ALL 4 CASES
//     break;
//   case RIGHT_Y:
//     Serial.print("VIVERIGHT STATUS:");
//     Serial.println(viveRight.status());
//     if (viveRight.status() == VIVE_RECEIVING)
//     {
//       int r1 = viveRight.yCoord();
//       int r2 = viveRight.yCoord();
//       int r3 = viveRight.yCoord();
//       avg_ry = med3Filt(r1, r2, r3);
//       Serial.printf("                                                          RightY %d ", avg_ry);
//       return avg_ry;
//     }
//     else
//     {
//       switch (viveRight.sync(5))
//       {
//         break;
//       case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
//         Serial.println("Right vive: weak signal");
//         break;
//       default:
//       case VIVE_NO_SIGNAL: // nothing detected
//         Serial.println("Right vive: no signal");
//       }
//     }
//     return avg_ry;
//     break;
//   } // end of switch
//   Serial.println("end of getVive f'n");
// }

// ViveSensor ViveLeft(1, VIVE_LEFT_GPIO);

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
          'raw_left_x': 0, \
          'raw_right_x':  0, \
          'raw_left_y':  0, \
          'raw_right_y':  0, \
          'degrees': 0 \
        }, \
        'IR_sensor': { \
          'beacon_700Hz': 0, \
          'beacon_23Hz': 0 \
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

void logicMode(int mode)
{
  if (mode == 1)
  { // Logic for Wall Follow: turn when distance gets <300mm:
    if (TimeOfFlightDegrees0.getDistance() < 400)
    {
      drive(-1, -1, 4);
    }
    else
    {
      drive(0, 0, 4);
    }
  }
  else
  {
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup()
{
  Serial.begin(115200);

  // WIFI SETUP
  Serial.print("Access Point SSID: ");
  Serial.print(ssid);
  WiFi.mode(WIFI_AP);          // Set Mode to Access Point
  WiFi.softAP(ssid, password); // Define access point SSID and its password

  IPAddress myIP(192, 168, 1, 161);                                                // Define my unique Static IP (from spreadsheet on slides)
  WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0)); // Define the (local IP, Gateway, Subnet mask)

  Serial.print("Use this URL to connect: http://");
  Serial.print(myIP);
  Serial.println("/"); // Print the IP to connect to

  h.begin();

  h.attachHandler("/ ", handleRoot);
  h.attachHandler("/key_pressed?val=", handleKeyPressed);
  h.attachHandler("/get_updated_state", handleStateUpdate);

  TimeOfFlightDegrees0.init();
  delay(2000);
  // viveRight.begin();
  // viveLeft.begin();
  
  // ViveLeftSensor.init();
  // ViveRightSensor.init();
  // OneVive.init();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop()
{
  // OneVive.calculateCoordinates();

  h.serve(); // listen to the frontend commands
  // delay(10);
}
