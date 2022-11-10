const char body[] PROGMEM = R"===(

<!DOCTYPE html>
<html>
  <head>
    <style> /* Inspired by https://codepen.io/HJ-b/pen/jbPqWO */
      body {
        background: rgb(35, 35, 35);
        color:rgb(235, 235, 235)
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
      
      .arrowbtn-up {
        top: 20px;
      }
      .arrowbtn-up:after {
        margin-left: -12.5px;
        margin-top: -6.25px;
        border-top: 2px solid;
        border-left: 2px solid;
        transform: rotateZ(45deg);
      }
      
      .arrowbtn-down {
        top: 200px;
      }
      .arrowbtn-down:after {
        margin-left: -12.5px;
        margin-top: -18.75px;
        border-bottom: 2px solid;
        border-right: 2px solid;
        transform: rotateZ(45deg);
      }
      
      .arrowbtn-left {
        top: 100px;
        left: 590px;
      }
      .arrowbtn-left:after {
        margin-left: -8.5px;
        margin-top: -13.75px;
        border-bottom: 2px solid;
        border-right: 2px solid;
        transform: rotateZ(135deg);
      }

      .arrowbtn-right {
        top: 100px;
        left: 850px;
      }
      .arrowbtn-right:after {
        margin-left: -17.5px;
        margin-top: -13.75px;
        border-bottom: 2px solid;
        border-right: 2px solid;
        transform: rotateZ(315deg);
      }
    </style>
  </head>
  <body>
    <b>Select X: </b>
    <input type="range" min="0" max="100" value="50" id="X_slider">
    <span id="frequency_display_text"> </span> <br> <br>

    <b>Key Pressed: </b>
    <span id="last_key_pressed_text"> </span> <br>

    <b>Server Response: </b>
    <span id="response_text"> </span> <br>

    <div id="element"></div>

    <span class="arrowbtn arrowbtn-up" id="arrow-up"></span>
    <span class="arrowbtn arrowbtn-down" id="arrow-down"></span>  
    <span class="arrowbtn arrowbtn-left" id="arrow-left"></span>
    <span class="arrowbtn arrowbtn-right" id="arrow-right"></span>  
  </body>

<script>

   function toggleColor(element_ID, condition) {
    if (condition) { // ON
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 1)";
    } else { // OFF
      document.getElementById(element_ID).style.background = "rgba(255, 255, 255, 0)";
    }
  }

  function sendToESP(text) {
    var xhttp = new XMLHttpRequest();
    var url = "key_pressed?val=" + text;

    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("response_text").innerHTML = this.responseText;
      }
    };

    xhttp.open("GET", url, true);
    xhttp.send(); 
  }

  var map = {}; // You could also use an array
  
  onkeydown = onkeyup = function(e){
      e = e || event; // to deal with IE
      map[e.keyCode] = e.type == 'keydown';
      /* insert conditional here */
      document.getElementById("last_key_pressed_text").innerHTML = JSON.stringify(map);
      console.log(JSON.stringify(map));
      sendToESP(JSON.stringify(map));

      toggleColor("arrow-up", map[38] || map[87]); // ArrowUp OR W
      toggleColor("arrow-down", map[40] || map[83]); // ArrowDown OR S
      toggleColor("arrow-left", map[37] || map[65]); // ArrowLeft OR A
      toggleColor("arrow-right", map[39] || map[68]); // ArrowRight OR D

  }

//  duty_cycle_slider.onchange = function() {
//    var xhttp = new XMLHttpRequest();
//    xhttp.onreadystatechange = function() {
//      if (this.readyState == 4 && this.status == 200) {
//        document.getElementById("duty_cycle_display_text").innerHTML = this.responseText;
//      }
//    };
//    var str = "duty_cycle_slider?val=";
//    var res = str.concat(this.value);
//    xhttp.open("GET", res, true);
//    xhttp.send(); 
//  }

  
</script>

)===";
