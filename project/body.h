const char body[] PROGMEM = R"===(

<!DOCTYPE html>
<html><body>
  <b>Select X: </b>
  <input type="range" min="0" max="100" value="50" id="X_slider">
  <span id="frequency_display_text"> </span> <br> <br>

  <b>Key Pressed: </b>
  <span id="last_key_pressed_text"> </span> <br>

  <b>Server Response: </b>
  <span id="response_text"> </span> <br>
</body>

<script>
  // Press any key and the result will display on the page.
  // Make sure your cursor is focused in the preview window first.
  window.addEventListener('keydown', function (e) {
    var key = String(e.key);
    
    // Make changes on the website
         document.getElementById("last_key_pressed_text").innerHTML = key;

    // Send information via GET request back to server (robot)
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
         // Typical action to be performed when the document is ready:
         document.getElementById("response_text").innerHTML = xhttp.responseText;
      }
    };
    var res = "key_pressed?val=" + key;
    xhttp.open("GET", res, true);
    xhttp.send(); 
  }, false);

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
