const char body[] PROGMEM = R"===(

<!DOCTYPE html>
<html>

<head>
  <style>
    /* Inspired by https://codepen.io/HJ-b/pen/jbPqWO */
    body {
      background: rgb(35, 35, 35);
      color: rgb(235, 235, 235)
    }

    .canvas {
      position: absolute;
      left: 0px;
      top: 400px;
    }

    .canvas-vive-raw {
      position: absolute;
      left: 950px;
      top: 100px;
    }

    .arrowbtn {
      position: absolute;
      width: 50px;
      height: 50px;
      background: rgba(17, 17, 17, 0);
      border: 2px solid hotpink;
      border-radius: 100px;
      color: hotpink;
      cursor: pointer;
      left: 50%;
      line-height: 100px;
      margin-left: -50px;
      transition: all 0.01s ease-in-out;
    }

    /* .arrowbtn:hover {
        background: white;
        border-color: white;
        color: #111;
      } */
    .arrowbtn:after {
      position: absolute;
      display: inline-block;
      content: "";
      width: 10px;
      height: 10px;
      top: 65%;
      left: 60%;
    }

    /* ~~~~~~~~ LOOK UP/DOWN/LEFT/RIGHT CSS ~~~~~~~~~~~~ */

    .arrowbtn-lookup {
      /* UP */
      left: 720px;
      top: 20px;
    }

    .arrowbtn-lookup:after {
      margin-left: -12.5px;
      margin-top: -6.25px;
      border-top: 2px solid;
      border-left: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-lookdown {
      /* DOWN */
      left: 720px;
      top: 200px;
    }

    .arrowbtn-lookdown:after {
      margin-left: -12.5px;
      margin-top: -18.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-lookleft {
      /* LEFT */
      top: 100px;
      left: 400px;
    }

    .arrowbtn-lookleft:after {
      margin-left: -8.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(135deg);
    }

    .arrowbtn-lookright {
      /* RIGHT */
      top: 100px;
      left: 500px;
    }

    .arrowbtn-lookright:after {
      margin-left: -17.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(315deg);
    }

    /* ~~~~~~~~ MOVE UP/DOWN/LEFT/RIGHT CSS ~~~~~~~~~~~~ */

    .arrowbtn-moveup {
      /* UP */
      left: 720px;
      top: 20px;
    }

    .arrowbtn-moveup:after {
      margin-left: -12.5px;
      margin-top: -6.25px;
      border-top: 2px solid;
      border-left: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-movedown {
      /* DOWN */
      left: 720px;
      top: 100px;
    }

    .arrowbtn-movedown:after {
      margin-left: -12.5px;
      margin-top: -18.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-moveleft {
      /* LEFT */
      top: 60px;
      left: 650px;
    }

    .arrowbtn-moveleft:after {
      margin-left: -8.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(135deg);
    }

    .arrowbtn-moveright {
      /* RIGHT */
      top: 60px;
      left: 790px;
    }

    .arrowbtn-moveright:after {
      margin-left: -17.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(315deg);
    }
  </style>
</head>

<body>
  <b>Change Speed (type 0-9): </b>
  <input type="range" min="0" max="9" value="5" id="speed_slider">
  <span id="speed_display_text"> </span> <br> <br>

  <b>Key Pressed: </b>
  <span id="last_key_pressed_text"> </span> <br>

  <b>Server Response from keys pressed: </b>
  <span id="response_text"> </span> <br>

  <b>Raw JSON from ESP response: </b>
  <span id="response_JSON"> </span> <br>

  <b>Raw Vive Coordinates - Vive #1 Blue: </b>
  <span id="vive-one-coordinates"> </span> <br>

  <b>Raw Vive Coordinates - Vive #2 Red: </b>
  <span id="vive-two-coordinates"> </span> <br>

  <span class="arrowbtn arrowbtn-lookleft" id="arrow-lookleft"></span>
  <span class="arrowbtn arrowbtn-lookright" id="arrow-lookright"></span>

  <span class="arrowbtn arrowbtn-moveup" id="arrow-moveup"></span>
  <span class="arrowbtn arrowbtn-movedown" id="arrow-movedown"></span>
  <span class="arrowbtn arrowbtn-moveleft" id="arrow-moveleft"></span>
  <span class="arrowbtn arrowbtn-moveright" id="arrow-moveright"></span>

  <canvas class="canvas" id="canvas-background" width="705" height="285" style="border:1px solid #ffffff;"></canvas>
  <canvas class="canvas" id="canvas-robot" width="705" height="285"></canvas>
  <canvas class="canvas-vive-raw" id="canvas-vive-raw" width="700" height="700" style="border:1px solid #ffa9a9;"></canvas>

</body>

<script>

  async function toggleColor(element_ID, condition) {
    if (condition) { // ON
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 1)";
    } else { // OFF
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 0)";
    }
  }

  async function setSpeed(speed) {
    document.getElementById("speed_slider").value = speed;
    document.getElementById("speed_display_text").innerHTML = speed.toString();
  }

  async function sendToESP(text) {
    var xhttp = new XMLHttpRequest();
    var url = "key_pressed?val=" + text;

    xhttp.onreadystatechange = function () {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("response_text").innerHTML = this.responseText;
      }
    };

    xhttp.open("GET", url, true);
    xhttp.send();

  }

  var map = {}; // You could also use an array

  onkeydown = onkeyup = async function (e) {
    e = e || event; // to deal with IE
    previousMapString = JSON.stringify(map); // save the previous map
    map[e.keyCode] = e.type == 'keydown'; // mark true the index of any key that's pressed.

    if (JSON.stringify(map) !== previousMapString) {
      document.getElementById("last_key_pressed_text").innerHTML = JSON.stringify(map);
      console.log(JSON.stringify(map));
      sendToESP(JSON.stringify(map));
    }

    // Look
    toggleColor("arrow-lookleft", map[37]); // ArrowLeft
    toggleColor("arrow-lookright", map[39]); // ArrowRight

    // Move
    toggleColor("arrow-moveup", map[87]); // W
    toggleColor("arrow-movedown", map[83]); // S
    toggleColor("arrow-moveleft", map[65]); // A
    toggleColor("arrow-moveright", map[68]); // D

    // Speed
    if (map[48] || map[49] || map[50] || map[51] || map[52] || map[53] || map[54] || map[55] || map[56] || map[57]) {
      setSpeed(e.key);
    }
  }

  const scale_multiplier = 2.5;
  var robot_width = 50;
  var robot_height = 50;
  var canvas_already_setup = false;
  var delay_per_update_request = 100;

  // Declare drawing objects
  var context_background = document.getElementById("canvas-background").getContext("2d");
  var context_robot = document.getElementById("canvas-robot").getContext("2d");
  var context_raw_vive = document.getElementById("canvas-vive-raw").getContext("2d");

  // Re-draw canvas from scratch.
  async function drawCanvasBackground() {
    var h = document.getElementById("canvas-background").clientHeight;
    var w = document.getElementById("canvas-background").clientWidth;

    context_background.fillStyle = "#003366";
    context_background.fillRect(0, 0, w / 2, h); // draw our side

    context_background.fillStyle = "#660033";
    context_background.fillRect(w / 2, 0, w / 2, h); // draw opposite side

    context_background.strokeStyle = "rgba(0, 0, 0, 0.8)";
    context_background.strokeRect(w / 4, h / 5, w / 8, h * (3 / 5)); // left 2x points box
    context_background.strokeRect(w * (7.5 / 12), h / 5, w / 8, h * (3 / 5)); // right 2x points box

  }

  // Repeat asking for updated state from ESP
  let repeating_state_ask = setInterval(updateState, delay_per_update_request);

  // Ask for updated state from ESP
  async function updateState() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "get_updated_state", true);
    xhttp.onload = (e) => {
      if (xhttp.readyState === 4) {
        if (xhttp.status === 200) {
          
          // EXECUTE THIS WHEN SUCCESSFUL RESPONSE!
          var response = xhttp.responseText;
          response = response.replace(/'/g, '"');
          var ESP_response = JSON.parse(response);
          console.log(ESP_response);
          // console.log("Updating Frontend");
          // console.log(ESP_response.setup.game_width);

          if (!canvas_already_setup && !ESP_response.skip_setup) {
            // Perform setup - done once. We use the multiplier to go from cm to desired pixel sizes
      
            document.getElementById("canvas-background").width = ESP_response.setup.game_width * scale_multiplier;
            document.getElementById("canvas-background").height = ESP_response.setup.game_height * scale_multiplier;
      
            document.getElementById("canvas-robot").width = ESP_response.setup.game_width * scale_multiplier;
            document.getElementById("canvas-robot").height = ESP_response.setup.game_height * scale_multiplier;
      
            robot_width = ESP_response.setup.robot_width * scale_multiplier;
            robot_height = ESP_response.setup.robot_height * scale_multiplier;
            
            drawCanvasBackground();
      
            canvas_already_setup = true;
      
          } else if (ESP_response.status == "success") {
            // update the state of the robot, by using the multiplier to go from cm to desired pixel sizes
            updateRobotState(ESP_response.robot.x * scale_multiplier, ESP_response.robot.y * scale_multiplier, 
              ESP_response.robot.raw_left_x, ESP_response.robot.raw_left_y, ESP_response.robot.raw_right_x, ESP_response.robot.raw_right_y, // Comment line when done debugging
              ESP_response.robot.degrees,
              ESP_response.IR_sensor.beacon_700Hz, ESP_response.IR_sensor.beacon_23Hz,
              ESP_response.ToF_sensor.distance * scale_multiplier,
              ESP_response.motors.power.front_left_A, ESP_response.motors.power.back_left_B, ESP_response.motors.power.front_right_C, ESP_response.motors.power.back_right_D,
              ESP_response.motors.direction.front_left_A, ESP_response.motors.direction.back_left_B, ESP_response.motors.direction.front_right_C, ESP_response.motors.direction.back_right_D);
      
          } else {
            console.log("ERROR: NO RESPONSE FROM ESP32 RECEIVED!!!");
          }

        } else {
          console.error(xhttp.statusText);
        }
      }
    };
    xhttp.onerror = (e) => {
      console.error(xhttp.statusText);
    }
    xhttp.send(null);
    
  };

  //moveRobot(100, 100, 110, 
  //              1, 1,
  //              3, 
  //              1.0, 1.0, 1.0, 0.3,
  //              1, 1, 1, 0);

  var vive_clear_counter = 101;

  async function updateRobotState(x_coordinate, y_coordinate, 
    raw_left_x, raw_left_y, raw_right_x, raw_right_y,
    degrees_facing, 
    see_700Hz_beacon, see_23Hz_beacon, distance,
    motor_power_A, motor_power_B, motor_power_C, motor_power_D,
    motor_dir_A, motor_dir_B, motor_dir_C, motor_dir_D) {

    // reset robot canvas
    context_robot.setTransform(1, 0, 0, 1, 0, 0); // Use the identity matrix while clearing the canvas
    context_robot.clearRect(0, 0, document.getElementById("canvas-background").clientWidth, document.getElementById("canvas-background").clientHeight); // Clean it
    
    // Set calculating costants, for our translation and rotation later
    var offset_degrees = 135; // Our starting point is top left which is 10.30 o'clock, whereas the cos/sin clock starts from 3 o'clock. That's a difference of 135 degrees.
    var radius = Math.sqrt((robot_width / 2) ** 2 + (robot_height / 2) ** 2);

    // Get new coordinates of top left corner, after displacement rcos(theta) and rsin(theta)
    var top_left_corner_x_coordinates = x_coordinate + radius * Math.cos((degrees_facing - offset_degrees) * Math.PI / 180);
    var top_left_corner_y_coordinates = y_coordinate + radius * Math.sin((degrees_facing - offset_degrees) * Math.PI / 180);

    // Draw robot location rectangle
    context_robot.fillStyle = "rgba(255, 255, 255, 0.6)";
    context_robot.translate(top_left_corner_x_coordinates, top_left_corner_y_coordinates);
    context_robot.rotate(degrees_facing * Math.PI / 180);
    context_robot.fillRect(0, 0, robot_width, robot_height);

    // Draw robot direction triangle
    context_robot.fillStyle = "rgba(255, 255, 255, 0.8)";
    context_robot.beginPath();
    context_robot.moveTo(robot_width * 0.2, robot_width / 5);
    context_robot.lineTo(robot_width * 0.5, 0);
    context_robot.lineTo(robot_width * 0.8, robot_width / 5);
    context_robot.fill();

    // Draw where robot sees with Infrared Receiver 700Hz
    (see_700Hz_beacon) ? context_robot.fillStyle = "rgba(255, 255, 0, 0.6)" : context_robot.fillStyle = "rgba(255, 255, 0, 0.1)";
    context_robot.beginPath();
    context_robot.moveTo(robot_width * 0.2, 0);
    context_robot.lineTo(robot_width * 0.4, -document.getElementById("canvas-background").clientWidth);
    context_robot.lineTo(robot_width * 0.5, 0);
    context_robot.fill();

    // Draw where robot sees with Infrared Receiver 23Hz
    (see_23Hz_beacon) ? context_robot.fillStyle = "rgba(0, 255, 0, 0.6)" : context_robot.fillStyle = "rgba(0, 255, 0, 0.1)";
    context_robot.beginPath();
    context_robot.moveTo(robot_width * 0.8, 0);
    context_robot.lineTo(robot_width * 0.6, -document.getElementById("canvas-background").clientWidth);
    context_robot.lineTo(robot_width * 0.5, 0);
    context_robot.fill();

    // Draw power of wheels
    context_robot.fillStyle = "rgba(" + motor_dir_A * 255 + ", " + motor_dir_A * 255 + ", " + motor_dir_A * 255 + "," + motor_power_A + ")"; // Front Left Motor A
    context_robot.fillRect(robot_width * 0.05, robot_height * 0.1, robot_width * 0.1, robot_height * 0.3);

    context_robot.fillStyle = "rgba(" + motor_dir_B * 255 + ", " + motor_dir_B * 255 + ", " + motor_dir_B * 255 + "," + motor_power_B + ")"; // Back Left Motor B
    context_robot.fillRect(robot_width * 0.05, robot_height * 0.6, robot_width * 0.1, robot_height * 0.3);

    context_robot.fillStyle = "rgba(" + motor_dir_C * 255 + ", " + motor_dir_C * 255 + ", " + motor_dir_C * 255 + "," + motor_power_C + ")"; // Front Right Motor C
    context_robot.fillRect(robot_width * 0.85, robot_height * 0.1, robot_width * 0.1, robot_height * 0.3);

    context_robot.fillStyle = "rgba(" + motor_dir_D * 255 + ", " + motor_dir_D * 255 + ", " + motor_dir_D * 255 + "," + motor_power_D + ")"; // Back Right Motor D
    context_robot.fillRect(robot_width * 0.85, robot_height * 0.6, robot_width * 0.1, robot_height * 0.3);
    
    // Part II: Update Vive Canvas

    if (vive_clear_counter > 20) {
      // reset vive canvas
      context_raw_vive.setTransform(1, 0, 0, 1, 0, 0); // Use the identity matrix while clearing the canvas
      context_raw_vive.clearRect(0, 0, document.getElementById("canvas-vive-raw").clientWidth, document.getElementById("canvas-vive-raw").clientHeight); // Clean it

      var p = 0;
      var bw = document.getElementById("canvas-vive-raw").clientWidth;
      var bh = document.getElementById("canvas-vive-raw").clientHeight;
      
      for (var x = 0; x <= bw; x += 100) {
          context_raw_vive.moveTo(0.5 + x + p, p);
          context_raw_vive.lineTo(0.5 + x + p, bh + p);
      }
  
      for (var x = 0; x <= bh; x += 100) {
          context_raw_vive.moveTo(p, 0.5 + x + p);
          context_raw_vive.lineTo(bw + p, 0.5 + x + p);
      }
      context_raw_vive.strokeStyle = "white";
      context_raw_vive.stroke();

      vive_clear_counter = 0;
    } else {
      vive_clear_counter = vive_clear_counter + 1;
    }

    // Print vive coordinates
    document.getElementById("vive-one-coordinates").innerHTML = "(" + String(raw_left_x) + ", " + String(raw_left_y) + ")";
    document.getElementById("vive-two-coordinates").innerHTML = "(" + String(raw_right_x) + ", " + String(raw_right_y) + ")";

    // Plot vive sensor #1
    context_raw_vive.fillStyle = "rgba(255, 0, 0, 1)";
    context_raw_vive.fillRect(Math.round(raw_left_x / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(raw_left_y / 10), 2, 2);

    // Plot vive sensor #2
    context_raw_vive.fillStyle = "rgba(0, 255, 255, 1)";
    context_raw_vive.fillRect(Math.round(raw_right_x / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(raw_right_y / 10), 2, 2);
    
    // Plot corners CHANGE HERE SOPHIE
    context_raw_vive.fillStyle = "rgba(0, 0, 255, 1)";
    context_raw_vive.fillRect(Math.round(6351 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(3200/ 10), 5, 5);
    context_raw_vive.fillRect(Math.round(3602 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(3607 / 10), 5, 5);

    context_raw_vive.fillRect(Math.round(1333 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(4570 / 10), 5, 5);
    context_raw_vive.fillRect(Math.round(1311 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(4974 / 10), 5, 5);

    context_raw_vive.fillRect(Math.round(1506 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(2371 / 10), 5, 5);
    context_raw_vive.fillRect(Math.round(1479 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(2700 / 10), 5, 5);

    context_raw_vive.fillRect(Math.round(6211/ 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(5431/ 10), 5, 5);
    context_raw_vive.fillRect(Math.round(6178 / 10), document.getElementById("canvas-vive-raw").clientHeight - Math.round(5698 / 10), 5, 5);


  }

</script>

)===";
