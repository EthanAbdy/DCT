//________________ARROWS________________//

function voltageGridSwitchL() {
  var x = document.getElementById("voltageTextID");
  if (x.innerHTML === "1.5 V") {
    x.innerHTML = "1.0 V";
  } 
  else if (x.innerHTML === "1.0 V") {
    x.innerHTML = "0.5 V";
  } 
  else {
    x.innerHTML = "0.1 V";
  }
}

function voltageGridSwitchR() {
  var x = document.getElementById("voltageTextID");
  if (x.innerHTML === "0.1 V") {
    x.innerHTML = "0.5 V";
  } 
  else if (x.innerHTML === "0.5 V") {
    x.innerHTML = "1.0 V";
  } 
  else {
    x.innerHTML = "1.5 V";
  } 
}

function waveTypeSwitchR() {
    var x = document.getElementById("waveTypeID");
    if (x.innerHTML === "Sine") {
      x.innerHTML = "Square";
      waveTypeID.style.left = "906px";
    }
  }
  
function waveTypeSwitchL() {
  var x = document.getElementById("waveTypeID");
  if (x.innerHTML === "Square") {
    x.innerHTML = "Sine";
    waveTypeID.style.left = "917px";
   } 
  }

  function nanosecondSwitchL() {
    var x = document.getElementById("waveType2ID");
    if (x.innerHTML === "y ns") {
      x.innerHTML = "100 ns";
    }
  }
  
function nanosecondSwitchR() {
  var x = document.getElementById("waveType2ID");
  if (x.innerHTML === "100 ns") {
    x.innerHTML = "y ns";
   } 
  }

//________________AC / DC SWITCH________________//

  function switchACDC() {
    var checkBox = document.getElementById("acDcID");
    var text = document.getElementById("text");
    if (checkBox.checked == true){ //WHEN AC IS CHOSEN//
      document.getElementsByTagName("div2")[0].style.backgroundColor = "#2D343A";
      document.getElementsByTagName("div3")[0].style.backgroundColor = "#2D343A";
      document.getElementsByTagName("div4")[0].style.backgroundColor = "#2D343A";
      document.getElementsByTagName("div5")[0].style.backgroundColor = "#2D343A";
      document.getElementsByTagName("div6")[0].style.backgroundColor = "#2D343A";
     
      ACLineOneID.style.display = "block";
      DCLineOneID.style.display = "none";

      gridVoltageTextID.style.color = "white";
      gridWaveFormTextID.style.color = "white";

    } else { //WHEN DC IS CHOSEN//
      document.getElementsByTagName("div2")[0].style.backgroundColor = "#363C3F";
      document.getElementsByTagName("div3")[0].style.backgroundColor = "#363C3F";
      document.getElementsByTagName("div4")[0].style.backgroundColor = "#363C3F";
      document.getElementsByTagName("div5")[0].style.backgroundColor = "#363C3F";
      document.getElementsByTagName("div6")[0].style.backgroundColor = "#363C3F";
      
      ACLineOneID.style.display = "none";
      DCLineOneID.style.display = "block";

      gridVoltageTextID.style.color = "grey";
      gridWaveFormTextID.style.color = "grey";
    }
  }

function mouseDown() {
}

function mouseUp() {
}

function updatePointerPosition( e )
{
    var avatar = document.getElementById("pointerID");
    avatar.style.left = e.x + "px";

    if (e.x > 863) {
      avatar.style.left = 863 + "px";
     }   
    if (e.x < 16) {
      avatar.style.left = 16 + "px";
    }
}

document.onmousemove = updatePointerPosition;
