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

  StaticJsonBuffer<600> jsonBuffer; // Create the JSON buffer
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
  Serial.print("Current speed: "); println(current_speed);
  Serial.println(" ");
}
