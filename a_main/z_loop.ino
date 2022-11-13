void loop() {
  h.serve(); // listen to the frontend commands

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