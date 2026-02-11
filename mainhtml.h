#pragma once

const char htmlfile[] PROGMEM = R"rawliteral(


<!DOCTYPE html>
<html>
<head>
<title>Hand movement detection</title>
</head>
<body>

<h1 style="text-align:center; font-size: 65px; color: black">Hand region movement indicator</h1>
<p style="text-align:center; font-size: 19px;">When you move your fingers or rotate your wrist, the corresponding region(s) will be highlighted</p>

<div style="position:relative; width:503px; height:743px; margin: 0 auto;">
<img id ="nothing" src="http://<PYTHON_HTTPSERVER_IP>/Downloads/nothing.png" style="display:block; width:503px; height:743px; margin: 0 auto" alt="No movement detectec">

<div style="position:absolute; display:flex; inset:0; width:503px; height:385px; margin: 0 auto;">
<img id="overlay" src="http://<PYTHON_HTTPSERVER_IP>/Downloads/nothing.png" alt="Finger Movement detected" style="display:block; width:503px; height:743px; margin: 0 auto;">
</div>

<div style="position:absolute; display:flex; inset:0; width:503px; height:743px; margin: 0 auto;">
<img id="wristoverlay" src="http://<PYTHON_HTTPSERVER_IP>/Downloads/nothing.png" alt="Wrist Movement detected" style="display:block; width:503px; height:743px; margin: 0 auto;">
</div>

</div>

<script>

const img = document.getElementById("overlay")
const wristimg = document.getElementById("wristoverlay")

const imgMapping = {
	  thumb: "http://<PYTHON_HTTPSERVER_IP>/Downloads/thumb.png",
    indexfingerhtml: "http://<PYTHON_HTTPSERVER_IP>/Downloads/index.png",
    middle: "http://<PYTHON_HTTPSERVER_IP>/Downloads/middle.png",
    ring: "http://<PYTHON_HTTPSERVER_IP>/Downloads/ring.png",
    pinky: "http://<PYTHON_HTTPSERVER_IP>/Downloads/pinky.png",
    leftwrist: "http://<PYTHON_HTTPSERVER_IP>/Downloads/leftwrist.png",
    rightwrist: "http://<PYTHON_HTTPSERVER_IP>/Downloads/rightwrist.png"
};

async function poll() {
  
  const [fingerText, wristText] = await Promise.all([fetch('/select', {cache: 'no-store'}).then(r => r.text()),
                                                     fetch('/wrists', {cache: 'no-store'}).then(r => r.text()),   
  ]);

  const newFT = fingerText.trim();
  const newWT = wristText.trim();

  const display = imgMapping[newFT];
  const wristdisplay = imgMapping[newWT];
  
  if (display) {
  	img.src = display;
  }
  else {
	img.src = "http://<PYTHON_HTTPSERVER_IP>/Downloads/nothing.png";
  }

  if (wristdisplay) {
  	wristimg.src = wristdisplay;
  }
  else {
	wristimg.src = "http://<PYTHON_HTTPSERVER_IP>/Downloads/nothing.png";
  }
  
}

setInterval(poll, 100);
poll();

</script>

</body>
</html>


)rawliteral";