const savedArmPositions = [
  "HIGH_SCORE",
  "MID_SCORE",
  "LOW_SCORE",
  "HANDOFF",
  "STOWED"
];

const commandmap = new Map([
  [
    "followtrajectory",
    [
      { paramname: "trajectoryPath", paramvalue: trajectoryfiles },
      { paramname: "resetOdometry", paramvalue: ["false", "true"] },
    ],
  ],
  ["wait", [{ paramname: "duration_in_seconds", paramvalue: "0" }]],
  ["stop", []],
  ["drivetopose",
    [{ paramname: "robotPose", paramvalue: savedRobotPoses }]
  ],
  [
    "alignto",
    [{ paramname: "robotPose", paramvalue: savedRobotPoses }]
  ],
  [
    "movearm",
    [{ paramname: "arpPresetPosition", paramvalue: savedArmPositions }]
  ],
  ["score",[]],
  ["conehandoff",[]],
  ["grabcone",[]],
  ["grip",[]],
  ["grabecube",[]],
  ["cubehandoff",[]]
]);

function buildForm() {
  var commanddropdown = document.getElementById("commandnamefield");
  // array1.forEach(element => console.log(element));
  for (const thiscommand of commandmap.keys()) {
    commanddropdown.options[commanddropdown.options.length] = new Option(
      thiscommand,
      thiscommand
    );
  }
  buildParameterInputs();
}

function buildParameterInputs() {
  var br = document.createElement("br");
  var commanddropdown = document.getElementById("commandnamefield");
  var parametersdiv = document.getElementById("parametersfields");
  var selectedcommand =
    commanddropdown.options[commanddropdown.selectedIndex].value;
  parametersdiv.innerHTML = "";
  for (const param of commandmap.get(selectedcommand)) {
    if (Array.isArray(param.paramvalue)) {
      var droplable = document.createElement("label");
      droplable.setAttribute("for", param.paramname);
      droplable.innerHTML = param.paramname;
      var dropdown = document.createElement("select");
      dropdown.setAttribute("id", param.paramname);
      dropdown.setAttribute("name", param.paramname);
      for (const thisparamopt of param.paramvalue) {
        dropdown.options[dropdown.options.length] = new Option(
          thisparamopt,
          thisparamopt
        );
      }

      var container = document.createElement("div");
      container.classList.add("parameter");
      container.appendChild(droplable);
      container.appendChild(dropdown);
      parametersdiv.appendChild(container);
    } else {
      var inputlabel = document.createElement("label");
      inputlabel.setAttribute("for", param.paramname);
      inputlabel.innerHTML = param.paramname;
      var inputfield = document.createElement("input");
      inputfield.setAttribute("type", "text");
      inputfield.setAttribute("name", param.paramname);
      inputfield.setAttribute("placeholder", param.paramvalue);

      var container = document.createElement("div");
      container.classList.add("parameter");
      container.appendChild(inputlabel);
      container.appendChild(inputfield);
      parametersdiv.appendChild(container);
    }
  }
}
function addLineToScript() {
  var paramvalues = Array.from(
    document.forms.paramsform.elements,
    (formElmt) => formElmt.value
  );

  var commandToAdd = document.getElementById("commandnamefield").value + "(";
  commandToAdd += paramvalues.join(",") + ");\n";

  document.getElementById("AutoScript").value += commandToAdd;
}

function addCommandToLine() {
  var paramvalues = Array.from(
    document.forms.paramsform.elements,
    (formElmt) => formElmt.value
  );

  var commandToAdd = document.getElementById("commandnamefield").value + "(";
  commandToAdd += paramvalues.join(",") + ")+";

  document.getElementById("AutoScript").value += commandToAdd;
}

function copycode() {
  document.getElementById("AutoScript").select();
  document.execCommand("copy");
}

function savetext(s) {
  function dataUrl(data) {
    return "data:x-application/text," + escape(data);
  }
  window.open(dataUrl(s));
}
