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

    .arrowbtn {
      position: absolute;
      width: 100px;
      height: 100px;
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
      width: 25px;
      height: 25px;
      top: 50%;
      left: 50%;
    }

    /* ~~~~~~~~ LOOK UP/DOWN/LEFT/RIGHT CSS ~~~~~~~~~~~~ */

    .arrowbtn-lookup { /* UP */
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

    .arrowbtn-lookdown { /* DOWN */
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

    .arrowbtn-lookleft { /* LEFT */
      top: 100px;
      left: 590px;
    }
    .arrowbtn-lookleft:after {
      margin-left: -8.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(135deg);
    }

    .arrowbtn-lookright { /* RIGHT */
      top: 100px;
      left: 850px;
    }
    .arrowbtn-lookright:after {
      margin-left: -17.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(315deg);
    }

    /* ~~~~~~~~ MOVE UP/DOWN/LEFT/RIGHT CSS ~~~~~~~~~~~~ */

    .arrowbtn-moveup { /* UP */
      left: 720px;
      top: 420px;
    }
    .arrowbtn-moveup:after {
      margin-left: -12.5px;
      margin-top: -6.25px;
      border-top: 2px solid;
      border-left: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-movedown { /* DOWN */
      left: 720px;
      top: 600px;
    }
    .arrowbtn-movedown:after {
      margin-left: -12.5px;
      margin-top: -18.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(45deg);
    }

    .arrowbtn-moveleft { /* LEFT */
      top: 500px;
      left: 590px;
    }
    .arrowbtn-moveleft:after { 
      margin-left: -8.5px;
      margin-top: -13.75px;
      border-bottom: 2px solid;
      border-right: 2px solid;
      transform: rotateZ(135deg);
    }

    .arrowbtn-moveright { /* RIGHT */
      top: 500px;
      left: 850px;
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

  <b>Server Response: </b>
  <span id="response_text"> </span> <br>

  <span class="arrowbtn arrowbtn-lookup" id="arrow-lookup"></span>
  <span class="arrowbtn arrowbtn-lookdown" id="arrow-lookdown"></span>
  <span class="arrowbtn arrowbtn-lookleft" id="arrow-lookleft"></span>
  <span class="arrowbtn arrowbtn-lookright" id="arrow-lookright"></span>

  <span class="arrowbtn arrowbtn-moveup" id="arrow-moveup"></span>
  <span class="arrowbtn arrowbtn-movedown" id="arrow-movedown"></span>
  <span class="arrowbtn arrowbtn-moveleft" id="arrow-moveleft"></span>
  <span class="arrowbtn arrowbtn-moveright" id="arrow-moveright"></span>
</body>

<script>

  function toggleColor(element_ID, condition) {
    if (condition) { // ON
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 1)";
    } else { // OFF
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 0)";
    }
  }

  function setSpeed(speed) {
    document.getElementById("speed_slider").value = speed;
    document.getElementById("speed_display_text").innerHTML = speed.toString();
  }

  function sendToESP(text) {
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

  onkeydown = onkeyup = function (e) {
    e = e || event; // to deal with IE
    previousMapString = JSON.stringify(map); // save the previous map
    map[e.keyCode] = e.type == 'keydown'; // mark true the index of any key that's pressed.

    if (JSON.stringify(map) !== previousMapString) {
      document.getElementById("last_key_pressed_text").innerHTML = JSON.stringify(map);
      console.log(JSON.stringify(map));
      sendToESP(JSON.stringify(map));
    }

    // Look
    toggleColor("arrow-lookup", map[38]); // ArrowUp
    toggleColor("arrow-lookdown", map[40]); // ArrowDown
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

    /* Turbo Mode
    if (map[16]) {
      document.style.background = "rgb(255, 51, 51)";
    } else {
      document.style.background = "rgb(35, 35, 35)";
    }
    */
  }


</script>

)===";
