//PWM output pins
#define MOTOR_PWM_GPIO_A 4
#define MOTOR_PWM_GPIO_B 5
#define MOTOR_PWM_GPIO_C 6
#define MOTOR_PWM_GPIO_D 7
//PWM channels for ledC
#define PWM_CH_A 0
#define PWM_CH_B 1
#define PWM_CH_C 2
#define PWM_CH_D 3
//direction pins
#define DIRECTION_GPIO_A 14
#define DIRECTION_GPIO_B 13
#define DIRECTION_GPIO_C 12
#define DIRECTION_GPIO_D 11
//encoder pins
#define ENCODER_GPIO_A 1
#define ENCODER_GPIO_B 2
#define ENCODER_GPIO_C 42
#define ENCODER_GPIO_D 41
//intitial direction of motors
//1 is CW, 0 is CCW
#define DIR_A 1
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


class Motor {
  private:
    int motor_pwm_gpio;
    int pwm_ch;
    int direction_gpio;
    int encoder_gpio;
    int dir;
    int encoder_state;
    int old_encoder_state;
    int curr_slot;
    int old_slot;
    float integration_sum_PID;
  public:
    Motor(int motor_pwm_gpio,int pwm_ch,int direction_gpio,int encoder_gpio,int dir) {
      this->motor_pwm_gpio = motor_pwm_gpio;
      this->pwm_ch = pwm_ch;
      this->direction_gpio = direction_gpio;
      this->encoder_gpio = encoder_gpio;
      this->dir = dir;
      
      init();
    }
    void init() {
      //ledC setup
      ledcAttachPin(this->motor_pwm_gpio,this->pwm_ch);
      ledcSetup(this->pwm_ch,PWM_FREQ,PWM_RES);

      //direction pin setup
      pinMode(this->direction_gpio, OUTPUT);

      //encoder pin setup
      pinMode(this->encoder_gpio, INPUT);

      //various initializations
      this->curr_slot = 0;
      this->old_slot = 0;
      this->encoder_state = 1;
      this->old_encoder_state = 0;
      this->integration_sum_PID = 0;
    }


    //calculates velocity using how many slots have moved in the time interval
    int getVel() {
      int vel = this->curr_slot - this->old_slot;
      this->old_slot = this->curr_slot;
      return vel;
    }
    
    //increments/decrements pos each time encoder goes
    //constantly keeping track of the number of slots gone by
    void updateEncoder () {
      //read encoder
      this->encoder_state = digitalRead(this->encoder_gpio);
      //if in new hole update
      if (this->encoder_state != this->old_encoder_state) {
        this->old_encoder_state = this->encoder_state;
        if (this->dir == 1) {
          //CW
          this->curr_slot = this->curr_slot + 1;
        } else {
          //CCW
          this->curr_slot = this->curr_slot - 1;
        }
        
      }
    }

    //PID goes here!!!!!
    void changeSpeed(int targetSpeed) {
      //this is the number of slots per time interval it is moving
      int currentSpeed = this->getVel();
      
      Serial.print("Target: ");
      Serial.println(targetSpeed);
      Serial.print("Current: ");
      Serial.println(currentSpeed);

      //PID will be here
      int error = targetSpeed - currentSpeed;  //[slots/timeinterval]
      float proportional = Kp * error;
      Serial.print("proportional: "); Serial.println(proportional);
      Serial.print("Existing integration sum: "); Serial.println(integration_sum_PID);
      this->integration_sum_PID = Ki * (this->integration_sum_PID + error * TIME_INTERVAL / 1000);
      Serial.print("integration_sum_PID: "); Serial.println(integration_sum_PID);
      
      //total_PI must be between 0 and 1
      float total_PI = this->integration_sum_PID + proportional;
      Serial.print("total_PI: ");
      Serial.println(total_PI);
      //TODO round it instead of cast
      if (total_PI < (float) 0.0) {
        total_PI = 0.0;
      } 
      else if (total_PI > (float) 1.0) {
        total_PI = 1.0;
      }
      //int new_duty_cycle = (int) min(max(total_PI, (float) 0.0), (float) 1.0) * (float) 4095.0;
      int new_duty_cycle = (int) (total_PI * (float) 4095.0);
      Serial.print("DC: ");
      Serial.println(new_duty_cycle);
      
      //this is where the speed of the motor is changed based off of duty cycle 0-4095
      ledcWrite(this->pwm_ch, new_duty_cycle);
    }


    //change the direction that the motor is going 
    //forwards or backwards
    void changeDirection(int new_dir) {
    //set the direction that the motors moves
      if (new_dir == 1) {
       //CW
       digitalWrite(this->direction_gpio,LOW);
        
     } else {
       //CCW
       digitalWrite(this->direction_gpio,HIGH);
     }
    }
    
}; 


//creating all the motor objects
Motor motorA(MOTOR_PWM_GPIO_A, PWM_CH_A, DIRECTION_GPIO_A, ENCODER_GPIO_A, DIR_A);
Motor motorB(MOTOR_PWM_GPIO_B, PWM_CH_B, DIRECTION_GPIO_B, ENCODER_GPIO_B, DIR_B);
Motor motorC(MOTOR_PWM_GPIO_C, PWM_CH_C, DIRECTION_GPIO_C, ENCODER_GPIO_C, DIR_C);
Motor motorD(MOTOR_PWM_GPIO_D, PWM_CH_D, DIRECTION_GPIO_D, ENCODER_GPIO_D, DIR_D);

void setup() {
  //for printing
  Serial.begin(9600);
  
}

// the loop function runs over and over again forever
void loop() {

  //SPEED Changing
  //reading potentiometer
  int target_speed = 9;

  //change speed once enough time has passed
  if(millis()-tick > TIME_INTERVAL) {
    tick = millis();
    motorA.changeSpeed(target_speed);
    //motorB.changeSpeed(target_speed);
    //motorC.changeSpeed(target_speed);
    //motorD.changeSpeed(target_speed);
  }
  //keep track of how many slots are going by
  motorA.updateEncoder();
  //motorB.updateEncoder();
  //motorC.updateEncoder();
  //motorD.updateEncoder();

  //example of how changing direction will work
  motorA.changeDirection(1);
  
  //delay
  delay(10);
  Serial.println("");
}
