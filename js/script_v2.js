var interfaceClient;
var markerClient;
var markerTopic;

function init() {
  // Connect to ROS.
  var ros = new ROSLIB.Ros({
    url: "ws://localhost:9090",
  });

  // If there is an error on the backend, an 'error' emit will be emitted.
  ros.on("error", function (error) {
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "none";
    document.getElementById("error").style.display = "inline";
    console.log(error);
  });

  // Find out exactly when we made a connection.
  ros.on("connection", function () {
    console.log("Connection made!");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("error").style.display = "none";
    document.getElementById("closed").style.display = "none";
    document.getElementById("connected").style.display = "inline";
  });

  ros.on("close", function () {
    console.log("Connection closed.");
    document.getElementById("connecting").style.display = "none";
    document.getElementById("connected").style.display = "none";
    document.getElementById("closed").style.display = "inline";
  });

  // The ActionClient
  // ----------------

  interfaceClient = new ROSLIB.ActionClient({
    ros: ros,
    serverName: "/interface_controller",
    actionName: "prog_support_backend/InterfaceRequestAction",
  });

  // Activate teaching mode on the Kinova arm
  var teach_goal = new ROSLIB.Goal({
    actionClient: interfaceClient,
    goalMessage: {
      request: 0,
      optional_index_1: 0,
      optional_index_2: 0,
      optional_index_3: 0,
    },
  });
  teach_goal.on("feedback", function (feedback) {
    console.log("Feedback: " + feedback.status);
  });
  teach_goal.on("result", function (result) {
    console.log("Final Result: " + result.conclusion);
  });

  // Send the goal to the action server.
  teach_goal.send();

  // Create the main viewer.
  var viewer = new ROS3D.Viewer({
    divID: "program-visualization",
    width: 600,
    height: 650,
    displayPanAndZoomFrame: false,
    antialias: true,
  });

  // Add a grid.
  viewer.addObject(new ROS3D.Grid());

  viewer.cameraControls.zoomIn(2);

  // Setup a client to listen to TFs.
  var tfClient = new ROSLIB.TFClient({
    ros: ros,
    angularThres: 0.01,
    transThres: 0.01,
    rate: 10.0,
  });

  // Setup the URDF client.
  var urdfClient = new ROS3D.UrdfClient({
    ros: ros,
    tfClient: tfClient,
    //path : 'https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/noetic-devel/',
    path: "/ros_kortex",
    rootObject: viewer.scene,
  });

  // Setup the marker client.
  markerClient = new ROS3D.MarkerClient({
    ros: ros,
    tfClient: tfClient,
    topic: "/visualization_marker",
    lifetime: 0,
    rootObject: viewer.scene,
  });

  markerTopic = new ROSLIB.Topic({
    ros: ros,
    name: "waypoint",
    messageType: "marker_package/Waypoint",
  });
}

function showTextBubble(buttonType) {
  var textBubble = document.getElementById("textBubble");
  textBubble.style.display = "block";
  if (buttonType == "runProgram") {
    textBubble.style.backgroundColor = "yellow";
    textBubble.textContent =
      "Remember to set any objects back to their original positions before running the program!";
  } else {
    textBubble.style.backgroundColor = "#f08080";
    textBubble.textContent = "Warning: You are about to erase your program!";
  }
}

function hideTextBubble() {
  var textBubble = document.getElementById("textBubble");
  textBubble.style.display = "none";
}

let start = false;

function removeHighlightFromButton(buttonId) {
  const button = document.getElementById(buttonId);
  button.classList.remove("highlight");
}

function removeHighlightLastWaypointHandle() {
  var lastWaypoint = document.getElementById("draggable-list").lastElementChild;
  if (lastWaypoint) {
    var lastDragHandle = lastWaypoint.querySelector(".drag-handle");
    if (lastDragHandle) {
      lastDragHandle.classList.remove("highlight-handle");
    }
  }
}

function removeHighlightLastWaypointLabel() {
  // Get the last drag handle and add the highlight class
  var lastWaypoint = document.getElementById("draggable-list").lastElementChild;
  if (lastWaypoint) {
    var lastWaypointLabel = lastWaypoint.querySelector(".waypoint-label");
    if (lastWaypointLabel) {
      lastWaypointLabel.classList.remove("highlight-handle");
    }
  }
}

var retractItems;
var numWaypointsBeforeRetract;
var automaticProceedChat = false;

var currentProgAssistanceState = "none";
var lastResponseContainer;

//new functions
function chatbot(intro, topics, scripts) {
  startAssistant();
  startChat(intro, topics, scripts);
}

async function addMessage(message, responseOptions, isUser = 0, scripts) {
  const chatContainer = document.getElementById("chat-container");
  const chatMessages = document.getElementById("chat-messages");
  const messageElement = document.createElement("div");
  if (message.trim() === "") {
  } else {
    if (!isMinimized) {
      chatContainer.style.display = "block";
    }
    messageElement.classList.add("chat-message");
    switch (isUser) {
      case -1:
      case 0:
        messageElement.classList.add("assistant-message");
        break;
      case 1:
        messageElement.classList.add("user-message");
        break;
      case 2:
        messageElement.classList.add("auto-assistant-message");
        break;
    }
    const messageText = document.createElement("span");
    messageText.innerText = message;
    messageElement.appendChild(messageText);
    chatMessages.appendChild(messageElement);
    chatMessages.scrollTop = chatMessages.scrollHeight;
  }

  // Create buttons for response options
  if (responseOptions && responseOptions.length > 0) {
    const responseContainer = document.createElement("div");
    const responseContainerId = `response-container-${Date.now()}`;
    responseContainer.id = responseContainerId;
    responseContainer.classList.add("response-options");
    responseContainer.style.marginTop = "25px"; // Adjust the value as needed
    responseOptions.forEach((option) => {
      const button = document.createElement("button");
      button.classList.add("response-button");
      button.innerText = option.label;
      if (option.label == "exit chat") {
        button.addEventListener("click", () => {
          clearChat();
          clearAllGuides();
          closeAssistant();
        });
      } else {
        button.addEventListener("click", () => {
          conversation(option.next, scripts, responseContainerId, scripts);
        });
      }
      currentProgAssistanceState = option.value;
      lastResponseContainer = responseContainerId;
      responseContainer.appendChild(button);
    });
    chatMessages.appendChild(responseContainer);
  }

  const delay = 1000; // 1 second (adjust the value as needed)
  await new Promise((resolve) => setTimeout(resolve, delay));

  chatContainer.scrollTop = chatContainer.scrollHeight;
  adjustChatContainerHeight();
}

function adjustChatContainerHeight() {
  const chatContainer = document.getElementById("chat-container");
  const chatMessages = document.getElementById("chat-messages");
  if (chatMessages.scrollHeight > 300) {
    chatContainer.style.height = "300px";
    chatContainer.style.overflowY = "scroll";
  } else {
    chatContainer.style.height = "auto";
    chatContainer.style.overflowY = "visible";
  }
}

async function conversation(conv, conversations, responseContainerId) {
  const responseContainer = document.getElementById(responseContainerId);
  if (responseContainer) {
    responseContainer.remove();
  }
  clearAllGuides();
  for (var i = 0; i < conv["user"].length; i++) {
    await addMessage(conv["user"][i], [], 1, conversations);
  }
  for (var j = 0; j < conv["bot"].length; j++) {
    await addMessage(conv["bot"][j], [], 0, conversations);
  }
  var nextMessage = [];
  for (var l = 0; l < conv["buttons"].length; l++) {
    for (var k = 0; k < conv["buttons"].length; k++) {
      nextMessage[k] = {
        label: conv["buttons"][k]["label"],
        next: conversations[conv["buttons"][k]["next"]],
      };
    }
  }
  for (var k = 0; k < conv["highlight"].length; k++) {
    if (
      conv["highlight"][k] == "add-waypoint" ||
      conv["highlight"][k] == "run-program" ||
      conv["highlight"][k] == "new-program"
    ) {
      document.getElementById(conv["highlight"][k]).classList.add("highlight");
    } else {
      document
        .getElementById(conv["highlight"][k])
        .classList.add("highlight-border");
    }
  }
  await addMessage("", nextMessage, -1, conversations);
}

async function startChat(intro, topics, scripts) {
  const assistantContainer = document.getElementById("assistant-container");
  const chatContainer = document.getElementById("chat-container");
  const chatMessages = document.getElementById("chat-messages");
  chatContainer.style.display = "block";
  assistantContainer.classList.remove("chat");
  chatContainer.classList.remove("chat");
  const messageElement = document.createElement("div");
  var startTopics = [];
  for (var i = 0; i < topics.length; i++) {
    startTopics[i] = { label: topics[i], next: scripts[i]["1"] };
  }
  if (!isMinimized) {
    chatContainer.style.display = "block";
  }
  messageElement.classList.add("chat-message");
  messageElement.classList.add("assistant-message");
  const messageText = document.createElement("span");
  messageText.innerText = intro;
  messageElement.appendChild(messageText);
  chatMessages.appendChild(messageElement);
  chatMessages.scrollTop = chatMessages.scrollHeight;
  // Create buttons for response options
  const responseContainer = document.createElement("div");
  const responseContainerId = `response-container-${Date.now()}`;
  responseContainer.id = responseContainerId;
  responseContainer.classList.add("response-options");
  responseContainer.style.marginTop = "25px"; // Adjust the value as needed
  startTopics.forEach((option, index) => {
    const button = document.createElement("button");
    button.classList.add("response-button");
    button.innerText = option.label;
    button.addEventListener("click", () => {
      console.log("check", scripts[index]);
      conversation(option.next, scripts[index], responseContainerId);
    });

    currentProgAssistanceState = option.value;
    lastResponseContainer = responseContainerId;
    responseContainer.appendChild(button);
  });

  chatMessages.appendChild(responseContainer);
  const delay = 2000; // 1 second (adjust the value as needed)
  await new Promise((resolve) => setTimeout(resolve, delay));
  adjustChatContainerHeight();
}

function startAssistant() {
  clearChat();
  clearAllGuides();
  let assistantContainer = document.getElementById("assistant-container");
  let calloutBubble = document.getElementById("callout-bubble");
  let minimizeButton = document.getElementById("minimize-button");
  let restartButton = document.getElementById("restart-button");
  let assistantIcon = document.getElementById("assistant-icon");

  assistantIcon.classList.add("active");
  assistantContainer.classList.add("active");
  minimizeButton.style.display = "block";
  restartButton.style.display = "block";
  calloutBubble.style.display = "none";
  atAssistantHome = true;
  assistantIcon.removeEventListener("click", startAssistant);
}

function closeAssistant() {
  console.log("Close");
  let assistantContainer = document.getElementById("assistant-container");
  let calloutBubble = document.getElementById("callout-bubble");
  let minimizeButton = document.getElementById("minimize-button");
  let restartButton = document.getElementById("restart-button");
  let assistantIcon = document.getElementById("assistant-icon");

  assistantContainer.classList.remove("active");
  assistantIcon.classList.remove("active");
  minimizeButton.style.display = "none";
  restartButton.style.display = "none";
  calloutBubble.style.display = "block";
  assistantIcon.addEventListener("click", startAssistant);
  currentProgAssistanceState = "none";
}

var atAssistantHome = true;
var isMinimized = false;

// Function to minimize the assistant
function restoreAssistant() {
  // JavaScript variables
  let minimizedAssistantContainer = document.getElementById(
    "minimized-assistant-container"
  );
  let minimizeButton = document.getElementById("minimize-button");
  let minimizeButtonContainer = document.getElementById(
    "minimize-button-container"
  );
  let chatContainer = document.getElementById("chat-container");
  let assistantContainer = document.getElementById("assistant-container");
  let restartButton = document.getElementById("restart-button");

  if (atAssistantHome) {
    assistantContainer.classList.remove("chat");
    chatContainer.style.display = "none";
    closeAssistant();
  } else {
    clearAllGuides();
    assistantContainer.classList.remove("chat");
    chatContainer.classList.remove("chat");
    assistantContainer.classList.toggle("minimized");
    if (chatContainer.style.display == "block") {
      chatContainer.style.display = "none";
      isMinimized = true;
      // Get the pseudo-element style
      const assistantIconStyle = getComputedStyle(
        document.querySelector("#assistant-container"),
        "::after"
      );

      // Convert the font size to a number
      const currentFontSize = parseInt(assistantIconStyle.fontSize);

      // Decrease the font size by a certain amount
      const newSize = currentFontSize - 5; // Decrease font size by 5px

      // Apply the new font size to the pseudo-element
      document
        .querySelector("#assistant-container")
        .style.setProperty("--icon-font-size", newSize + "px");
      restartButton.style.display = "none";
      minimizeButtonContainer.classList.add("minimized");
    } else {
      assistantContainer.classList.add("chat");
      chatContainer.classList.add("chat");
      chatContainer.style.display = "block";
      isMinimized = false;
      // Get the pseudo-element style
      const assistantIconStyle = getComputedStyle(
        document.querySelector("#assistant-container"),
        "::after"
      );

      // Convert the font size to a number
      const currentFontSize = parseInt(assistantIconStyle.fontSize);

      // Decrease the font size by a certain amount
      const newSize = currentFontSize + 5; // Decrease font size by 5px

      // Apply the new font size to the pseudo-element
      document
        .querySelector("#assistant-container")
        .style.setProperty("--icon-font-size", newSize + "px");

      restartButton.style.display = "block";
      minimizeButtonContainer.classList.remove("minimized");
    }
  }
}

var clearedChat = false;
function clearChat() {
  const assistantContainer = document.getElementById("assistant-container");
  const chatContainer = document.getElementById("chat-container");
  chatContainer.style.display = "none"; // Hide the chat container

  const chatMessages = document.getElementById("chat-messages");
  while (chatMessages.firstChild) {
    chatMessages.removeChild(chatMessages.firstChild); // Remove all child elements
  }

  assistantContainer.classList.remove("chat");
  chatContainer.classList.remove("chat");
}

function clearAllGuides() {
  //document.getElementById('col-2').classList.remove('highlight-border');
  document
    .getElementById("draggable-list")
    .classList.remove("highlight-border");
  document.getElementById("run-program").classList.remove("highlight-border");
  removeHighlightFromButton("add-waypoint");
  removeHighlightFromButton("run-program");
  removeHighlightFromButton("new-program");
  removeHighlightLastWaypointHandle();
  removeHighlightLastWaypointLabel();
}

function createButtonWithTooltip(
  containerId,
  iconClasses,
  tooltipText,
  dynamicTooltip = false
) {
  const container = document.getElementById(containerId);
  const button = document.createElement("i");
  button.className = iconClasses + " fa-xl";
  button.style.display = "none";
  container.appendChild(button);

  const tooltip = document.createElement("div");
  tooltip.classList.add("custom-tooltip");
  tooltip.textContent = tooltipText;
  container.appendChild(tooltip);

  if (dynamicTooltip) {
    const assistantContainer = document.getElementById("assistant-container");
    container.addEventListener("mouseenter", () => {
      const isMinimized = assistantContainer.classList.contains("minimized");
      tooltip.textContent = isMinimized ? "Maximize" : "Minimize";
      tooltip.style.display = "block";
    });
  } else {
    container.addEventListener("mouseenter", () => {
      tooltip.style.display = "block";
    });
  }

  container.addEventListener("mouseleave", () => {
    tooltip.style.display = "none";
  });
}

document.addEventListener("DOMContentLoaded", function () {
  init();

  var draggableList = document.getElementById("draggable-list");
  var addWaypointBtn = document.getElementById("add-waypoint");
  var runProgramBtn = document.getElementById("run-program");
  let assistantIcon = document.getElementById("assistant-icon");
  let minimizeButton = document.getElementById("minimize-button");
  var newProgramBtn = document.getElementById("new-program");
  //var refreshBtn = document.getElementById('refresh');
  var deleteWaypointOption = document.getElementById("delete-waypoint");
  var modifyWaypointOption = document.getElementById("modify-waypoint");
  var runWaypointOption = document.getElementById("exec-waypoint");
  singleItemContextMenu = document.getElementById("single-item-context-menu");
  multiItemContextMenu = document.getElementById("multiple-items-context-menu");
  const assistantContainer = document.getElementById("assistant-container");

  assistantIcon.addEventListener("click", startAssistant);
  minimizeButton.addEventListener("click", restoreAssistant);

  // Create the Minimize/Maximize button
  createButtonWithTooltip(
    "minimize-button-container",
    "far fa-window-restore",
    "Minimize/Maximize",
    true
  );

  // Create the Restart button
  createButtonWithTooltip(
    "restart-button-container",
    "fas fa-sync-alt",
    "Restart"
  );

  let isDragging = false;
  let startPosY = 0;
  let startOffsetY = 0;

  assistantContainer.addEventListener("mousedown", (e) => {
    isDragging = true;
    startPosY = e.clientY;
    startOffsetY = assistantContainer.offsetTop;
  });

  document.addEventListener("mousemove", (e) => {
    if (!isDragging) return;

    const offsetY = e.clientY - startPosY;
    const newPositionY = startOffsetY + offsetY;

    // Get the height of the assistantContainer
    const containerHeight = assistantContainer.offsetHeight;

    // Get the height of the browser viewport
    const viewportHeight = window.innerHeight;

    // Calculate the maximum position the container can be dragged to (within the viewport)
    const maxPositionY = viewportHeight - containerHeight;

    // Ensure the new position is within the allowed range
    const finalPositionY = Math.min(Math.max(newPositionY, 0), maxPositionY);

    // Set the new position of the assistantContainer
    assistantContainer.style.top = finalPositionY + "px";
  });

  document.addEventListener("mouseup", () => {
    isDragging = false;
  });

  var nextIndex = 1; // Initial index for waypoints

  addWaypointBtn.addEventListener("click", function () {
    createWaypoint();
    if (
      currentProgAssistanceState == "editExistingAdd4" ||
      currentProgAssistanceState == "pickAndMoveHelp6_2" ||
      currentProgAssistanceState == "pickAndMove5_2" ||
      currentProgAssistanceState == "pickAndPlaceHelp6_2" ||
      currentProgAssistanceState == "pickAndHoldHelp6_2" ||
      currentProgAssistanceState == "pickAndMove2" ||
      currentProgAssistanceState == "pickAndPlace2" ||
      currentProgAssistanceState == "pickAndMoveHelp3_2" ||
      currentProgAssistanceState == "pickAndPlaceHelp3_2" ||
      currentProgAssistanceState == "pickAndPlace5_2" ||
      currentProgAssistanceState == "pickAndHold2" ||
      currentProgAssistanceState == "pickAndHoldHelp3_2" ||
      currentProgAssistanceState == "pickAndHold5_2" ||
      currentProgAssistanceState == "pickAndPlace10"
    ) {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }
  });

  runProgramBtn.addEventListener("click", function () {
    // Display a confirmation popup
    const confirmed = confirm("Are you sure you want to run the program?");

    // If the user confirmed, execute the runProgram() function
    if (confirmed) {
      runProgram();
    }
  });

  newProgramBtn.addEventListener("click", function () {
    newProgram();
  });

  function applyLabelBehavior(waypointLabel, saveButton) {
    const originalLabelStyles = {
      color: "gray",
      backgroundColor: "#f1f1f1",
      padding: "0px",
      border: "0px",
    };

    function resetLabelStyles() {
      waypointLabel.style.color = originalLabelStyles.color;
      waypointLabel.style.backgroundColor = originalLabelStyles.backgroundColor;
      waypointLabel.style.padding = originalLabelStyles.padding;
      waypointLabel.style.border = originalLabelStyles.border;
    }

    waypointLabel.addEventListener("click", function () {
      if (
        this.classList.contains("placeholder") ||
        saveButton.style.display === "none"
      ) {
        this.classList.remove("placeholder");
        this.innerText = ""; // Clear the content if it's the placeholder text
        saveButton.style.display = "inline-block"; // Show the save button when label is clicked
        this.style.color = "black"; // Change the font color to black
        this.style.backgroundColor = "white"; // Change the background color to white
        this.style.padding = "5px"; // Add padding to the label
        this.style.border = "1px solid black"; // Add a border to the label
      }
    });

    saveButton.addEventListener("click", function () {
      if (waypointLabel.innerText.trim() === "") {
        waypointLabel.classList.add("placeholder"); // Add the placeholder class back
      }
      resetLabelStyles();
      saveButton.style.display = "none"; // Hide the save button after saving

      if (
        currentProgAssistanceState == "pickAndPlace6" ||
        currentProgAssistanceState == "pickAndHold6" ||
        currentProgAssistanceState == "pickAndMove6" ||
        currentProgAssistanceState == "pickAndPlace11"
      ) {
        automaticProceedChat = true;
        handleResponseOptionClick(
          currentProgAssistanceState,
          lastResponseContainer
        );
      }
    });

    // Add a click event listener to the document
    document.addEventListener("click", function (event) {
      // Check if the click target is not the label or the save button
      if (
        event.target !== waypointLabel &&
        event.target !== saveButton &&
        !waypointLabel.contains(event.target) // Check if the click is within the label
      ) {
        if (waypointLabel.innerText.trim() === "") {
          waypointLabel.classList.add("placeholder"); // Add the placeholder class back
        }
        // Reset the label's styles
        resetLabelStyles();

        // Hide the save button
        saveButton.style.display = "none";
      }
    });
  }

  function applyTooltipBehavior(element, tooltipText) {
    const tooltip = document.createElement("div");
    tooltip.classList.add("custom-tooltip");
    tooltip.textContent = tooltipText;
    document.body.appendChild(tooltip); // Append tooltip to body

    element.addEventListener("mouseenter", () => {
      const rect = element.getBoundingClientRect();
      const tooltipWidth = tooltip.offsetWidth;
      const tooltipHeight = tooltip.offsetHeight;

      tooltip.style.left = rect.left + rect.width / 2 - tooltipWidth / 2 + "px";
      tooltip.style.bottom = window.innerHeight - rect.top + "px"; // Position above the symbol

      tooltip.style.zIndex = "9999"; // Increase z-index value
      tooltip.style.display = "block";
    });

    element.addEventListener("mouseleave", () => {
      tooltip.style.display = "none";
    });

    // Attach the tooltip element to the element
    element.tooltipElement = tooltip;
  }

  // Create a new waypoint item in the draggable list
  function createWaypoint() {
    var add_goal = new ROSLIB.Goal({
      actionClient: interfaceClient,
      goalMessage: {
        request: 2,
        optional_index_1: 0,
        optional_index_2: 0,
        optional_index_3: 0,
      },
    });
    add_goal.on("feedback", function (feedback) {
      console.log("Feedback: " + feedback.status);
    });
    add_goal.on("result", function (result) {
      console.log("Final Result: " + result.conclusion);
    });

    // Send the goal to the action server.
    add_goal.send();

    var waypointItem = document.createElement("li");

    var waypointText = document.createElement("span");
    waypointText.innerText = "Waypoint " + nextIndex + " ";
    waypointText.classList.add("waypoint-text"); // Add a class to identify the waypoint text
    waypointItem.dataset.index = nextIndex - 1;

    waypointItem.appendChild(waypointText); // Add the waypoint text

    var waypointLabel = document.createElement("span");
    waypointLabel.classList.add("waypoint-label");
    waypointLabel.classList.add("placeholder"); // Add a class for styling the placeholder appearance

    waypointLabel.setAttribute("contenteditable", "true");
    waypointLabel.setAttribute("placeholder", "Click to add optional label");

    var saveButton = document.createElement("button");
    saveButton.classList.add("waypoint-save-button");
    saveButton.innerText = "Save";
    saveButton.style.display = "none"; // Hide save button initially

    const originalLabelStyles = {
      color: waypointLabel.style.color,
      backgroundColor: waypointLabel.style.backgroundColor,
      padding: waypointLabel.style.padding,
      border: waypointLabel.style.border,
    };

    applyLabelBehavior(waypointLabel, saveButton);

    waypointItem.appendChild(waypointLabel);
    waypointItem.appendChild(saveButton);

    var dragHandle = document.createElement("span");
    dragHandle.className = "drag-handle";
    //dragHandle.innerHTML = '<i class="uil uil-draggabledots"></i>';
    waypointItem.appendChild(dragHandle);

    var deleteIcon = document.createElement("i");
    deleteIcon.className = "fas fa-trash-alt delete-icon";
    deleteIcon.addEventListener("click", deleteWaypoint);
    waypointItem.appendChild(deleteIcon);

    waypointItem.draggable = true;
    addEventsDragAndDrop(waypointItem);
    waypointItem.addEventListener("contextmenu", showContextMenu);
    waypointItem.addEventListener("contextmenu", handleItemClick);
    waypointItem.addEventListener("click", handleItemClick);

    applyTooltipBehavior(dragHandle, "Drag");
    applyTooltipBehavior(deleteIcon, "Delete waypoint");

    draggableList.appendChild(waypointItem);
    nextIndex++;
  }

  function removeDeleteIconFromWaypoint(waypoint) {
    const deleteIcon = waypoint.querySelector(".delete-icon");
    if (deleteIcon) {
      deleteIcon.removeEventListener("click", deleteWaypoint);
      waypoint.removeChild(deleteIcon); // This line removes the icon element
    }
  }

  function deleteWaypoint() {
    if (currentProgAssistanceState == "editExistingDelete2") {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }
    isDeleteClick = true;
    const targetItem = event.target.parentNode;
    if (targetItem) {
      const listItem = event.target.closest("li");
      const draggableList = document.getElementById("draggable-list");
      //const index = Array.from(draggableList.children).indexOf(listItem);
      const index = parseInt(listItem.dataset.index);
      //console.log("Index: " + index);
      // Find and remove the tooltip element
      // Check if the listItem has a tooltipElement
      const deleteIcon = listItem.querySelector(".delete-icon");
      if (
        deleteIcon.tooltipElement &&
        deleteIcon.tooltipElement.parentElement
      ) {
        // Remove the tooltip element from the DOM
        deleteIcon.tooltipElement.parentElement.removeChild(
          deleteIcon.tooltipElement
        );
      }
      targetItem.remove();
      if (index >= 0) {
        /*markerClient.removeMarker("sphere" + (index + 1));
          markerClient.removeMarker("text" + (index + 1));*/
        var del_goal = new ROSLIB.Goal({
          actionClient: interfaceClient,
          goalMessage: {
            request: 3,
            optional_index_1: index,
            optional_index_2: 0,
            optional_index_3: 0,
          },
        });
        del_goal.on("feedback", function (feedback) {
          console.log("Feedback: " + feedback.status);
        });
        del_goal.on("result", function (result) {
          console.log("Final Result: " + result.conclusion);
        });

        // Send the goal to the action server.
        del_goal.send();

        // Deselect the item
        selectedItems = selectedItems.filter(
          (selectedItem) => selectedItem !== index
        );
        //sendDeselectGoal(index);
      }
      updateIndices();
    } else {
      console.log("Target item is false");
    }
  }

  function runProgram() {
    var run_goal = new ROSLIB.Goal({
      actionClient: interfaceClient,
      goalMessage: {
        request: 6,
        optional_index_1: 0,
        optional_index_2: 0,
        optional_index_3: 0,
      },
    });
    run_goal.on("feedback", function (feedback) {
      console.log("Feedback: " + feedback.status);
    });
    run_goal.on("result", function (result) {
      console.log("Final Result: " + result.conclusion);
      speak("The program is finished running.");
    });

    // Send the goal to the action server.
    run_goal.send();
  }

  var selectedItems = [];
  var highlightedItems = [];
  var lastHighlighted = -1;
  var isDeleteClick = false;

  function handleItemClick(event) {
    //console.log("handle item click");
    var item = event.target.closest("li");
    const index = Array.from(draggableList.children).indexOf(item);
    const highlightIndex = parseInt(item.dataset.index, 10);
    lastHighlighted = highlightIndex;
    //console.log("index " + highlightIndex);

    // Check if the right mouse button is pressed
    var isRightClick = event.button === 2 || event.which === 3 || isDeleteClick;

    if (item && !isRightClick) {
      var isHighlighted = item.classList.contains("highlighted");

      if (
        currentProgAssistanceState == "editExistingReplace2" ||
        currentProgAssistanceState == "editExistingMove2"
      ) {
        automaticProceedChat = true;
        handleResponseOptionClick(
          lastResponseContainer,
          currentProgAssistanceState
        );
      }

      if (isHighlighted) {
        item.classList.remove("highlighted");
        selectedItems = selectedItems.filter(
          (selectedItem) => selectedItem !== index
        );
        highlightedItems = highlightedItems.filter(
          (highlightedItem) => highlightedItem !== highlightIndex
        );
        sendDeselectGoal(highlightIndex);
      } else {
        item.classList.add("highlighted");
        selectedItems.push(index);
        highlightedItems.push(highlightIndex);
        sendHighlightGoal(highlightIndex);
      }
    }
    var numItems = draggableList.querySelectorAll(
      "li:not(.module-container)"
    ).length;
    if (
      (selectedItems.length == numItems &&
        (currentProgAssistanceState == "pickAndPlace3_2" ||
          currentProgAssistanceState == "pickAndMove3_2")) ||
      ((currentProgAssistanceState == "pickAndPlace7_2" ||
        currentProgAssistanceState == "pickAndMove7_2" ||
        currentProgAssistanceState == "repeatMotion3" ||
        currentProgAssistanceState == "singleMotion3") &&
        selectedItems.length == retractItems) ||
      (selectedItems.length == numItems &&
        currentProgAssistanceState == "pickAndHold3_2") ||
      (currentProgAssistanceState == "pickAndHold7_2" &&
        selectedItems.length == retractItems)
    ) {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }
    isDeleteClick = false;
  }

  function showContextMenu(event) {
    event.preventDefault(); // Prevent the default context menu from appearing

    if (selectedItems.length === 0) {
      console.log("No waypoints selected");
      return; // No waypoints selected, exit early
    }

    var contextMenu;

    if (
      selectedItems.length === 1 &&
      checkIfItemsAreAdjacent(highlightedItems)
    ) {
      contextMenu = document.getElementById("single-item-context-menu");
      handleSingleItemContextMenu(event);
      contextMenu.style.display = "block";
      contextMenu.style.left = event.pageX + "px";
      contextMenu.style.top = event.pageY + "px";
    } else if (
      selectedItems.length > 1 &&
      checkIfItemsAreAdjacent(highlightedItems)
    ) {
      contextMenu = document.getElementById("multiple-items-context-menu");
      handleMultiItemContextMenu(event);
      contextMenu.style.display = "block";
      contextMenu.style.left = event.pageX + "px";
      contextMenu.style.top = event.pageY + "px";
    } else {
      alert(
        "Please click a single item or items that are next to each other to access the menu."
      );
      hideContextMenu();
      return; // Non-adjacent items, exit early
    }

    document.addEventListener("click", hideContextMenu);
  }

  function checkIfItemsAreAdjacent(selectedItems) {
    // Sort the selected items in ascending order
    var sortedItems = selectedItems.slice().sort(function (a, b) {
      return a - b;
    });

    // Check if each item is adjacent to the next one
    for (var i = 0; i < sortedItems.length - 1; i++) {
      if (sortedItems[i] + 1 !== sortedItems[i + 1]) {
        return false;
      }
    }

    return true;
  }

  function hideContextMenu() {
    var contextMenus = document.getElementsByClassName("context-menu");
    Array.from(contextMenus).forEach(function (contextMenu) {
      contextMenu.style.display = "none";
    });

    document.removeEventListener("click", hideContextMenu);
  }

  function sendHighlightGoal(index) {
    var highlightGoal = new ROSLIB.Goal({
      actionClient: interfaceClient,
      goalMessage: {
        request: 7,
        optional_index_1: (index + 1) * -1,
        optional_index_2: 1,
        optional_index_3: 0,
      },
    });

    highlightGoal.on("feedback", function (feedback) {
      console.log("Feedback: " + feedback.status);
    });

    highlightGoal.on("result", function (result) {
      console.log("Final Result: " + result.conclusion);
    });

    // Send the goal to the action server.
    highlightGoal.send();
  }

  function sendDeselectGoal(index) {
    console.log("Deselecting " + index);
    var deselectGoal = new ROSLIB.Goal({
      actionClient: interfaceClient,
      goalMessage: {
        request: 7,
        optional_index_1: (index + 1) * -1,
        optional_index_2: 0,
        optional_index_3: 0,
      },
    });

    deselectGoal.on("feedback", function (feedback) {
      console.log("Feedback: " + feedback.status);
    });

    deselectGoal.on("result", function (result) {
      console.log("Final Result: " + result.conclusion);
    });

    // Send the goal to the action server.
    deselectGoal.send();
  }

  // Function to dehighlight all waypoints
  function dehighlightAllWaypoints() {
    const draggableList = document.getElementById("draggable-list");
    const items = draggableList.querySelectorAll("li");

    items.forEach(function (item) {
      if (item.classList.contains("highlighted")) {
        item.classList.remove("highlighted");
      }
    });

    var clearMarkersMessage = new ROSLIB.Message({
      num: 0,
      swap_id1: 0,
      swap_id2: 0,
      extra_num: -1,
    });

    markerTopic.publish(clearMarkersMessage);
    selectedItems = [];
    highlightedItems = [];
  }

  document.body.addEventListener("click", function (event) {
    var isBackgroundClicked = isBackgroundElement(event.target);

    // Exclude the draggable list, second column, third column, assistant icon, and speech bubble from triggering dehighlighting
    var isDraggableListClicked =
      event.target.closest("#draggable-list") !== null;
    var isSecondColumnClicked =
      event.target.closest("#col-2") !== null ||
      event.target.closest("#program-visualization") !== null;
    var isThirdColumnClicked =
      event.target.closest(".column:not(#col-1):not(#col-2)") !== null;
    var isAssistantIconClicked = event.target.classList.contains("fa-user");
    var isAssistantBubbleClicked =
      event.target.closest(".callout-container") !== null;

    if (
      isBackgroundClicked &&
      !isDraggableListClicked &&
      !isSecondColumnClicked &&
      !isThirdColumnClicked &&
      !isAssistantIconClicked &&
      !isAssistantBubbleClicked
    ) {
      dehighlightAllWaypoints();
    }
  });

  function isBackgroundElement(element) {
    if (element.tagName === "BODY" || element.tagName === "HTML") {
      return true;
    }

    if (element.tagName === "P") {
      return true;
    }

    if (element.parentElement) {
      return isBackgroundElement(element.parentElement);
    }

    return false;
  }

  function addEventsDragAndDrop(el) {
    el.addEventListener("dragstart", dragStart, false);
    el.addEventListener("dragenter", dragEnter, false);
    el.addEventListener("dragover", dragOver, false);
    el.addEventListener("dragleave", dragLeave, false);
    el.addEventListener("drop", dragDrop, false);
    el.addEventListener("dragend", dragEnd, false);
  }

  function removeEventsDragAndDrop(el) {
    el.removeEventListener("dragstart", dragStart, false);
    el.removeEventListener("dragenter", dragEnter, false);
    el.removeEventListener("dragover", dragOver, false);
    el.removeEventListener("dragleave", dragLeave, false);
    el.removeEventListener("drop", dragDrop, false);
    el.removeEventListener("dragend", dragEnd, false);
  }

  // Global variables to store the initial and new index
  var initialIndex;
  var newIndex;

  function dragStart(e) {
    e.stopPropagation(); // Prevent event propagation
    if (this.closest(".module-container") || this.closest(".loop-container")) {
      e.preventDefault();
      return;
    }

    // Disable content editing on all user input labels
    var userInputLabels = document.querySelectorAll("waypoint-label");
    userInputLabels.forEach(function (label) {
      label.contentEditable = "false";
    });

    this.style.opacity = "0.4";
    dragSrcEl = this;
    initialIndex = parseInt(dragSrcEl.dataset.index, 10);
    //initialIndex = Array.from(draggableList.children).indexOf(dragSrcEl);
    e.dataTransfer.effectAllowed = "move";
    //e.dataTransfer.setData('text/html', this.innerHTML);
  }

  function dragEnter(e) {
    e.stopPropagation(); // Prevent event propagation
    this.classList.add("over");
  }

  function dragLeave(e) {
    e.stopPropagation();
    this.classList.remove("over");
  }

  function dragOver(e) {
    e.stopPropagation(); // Prevent event propagation
    e.preventDefault();
    e.dataTransfer.dropEffect = "move";
    //return false;
  }

  function shiftContainerDown(container) {
    const parentContainer = container.parentElement;
    const nextContainer = container.nextElementSibling;

    if (nextContainer) {
      parentContainer.removeChild(container);
      parentContainer.insertBefore(container, nextContainer.nextSibling);
    }
  }

  function shiftContainersBelowIndex(startIndex) {
    // Select all module containers
    var moduleContainers = document.querySelectorAll(".module-container");

    // Calculate the number of containers
    var numContainers = moduleContainers.length;

    // Iterate through module containers from the end
    for (var i = numContainers - 1; i >= 0; i--) {
      var currentContainer = moduleContainers[i];
      var loopWaypointsList =
        currentContainer.querySelector(".module-waypoints");
      // Get all waypoints within the loop container
      var loopWaypoints = loopWaypointsList.querySelectorAll("li");

      // Find the index of the list item within the container
      var listItemIndex = Array.from(loopWaypoints).findIndex(function (item) {
        return parseInt(item.dataset.index) > startIndex;
      });

      // Check if the list item index is greater than or equal to 0
      // Check if the list item index is greater than or equal to the clicked index
      if (listItemIndex > -1) {
        shiftContainerDown(currentContainer);
      } else {
        // Stop iterating when a container above the specified index is encountered
        break;
      }
    }
  }

  function dragDrop(e) {
    e.stopPropagation(); // Prevent event propagation
    // Check if the drop target is a module or loop container
    var isModuleContainer = this.classList.contains("module-container");
    var isLoopContainer = this.classList.contains("loop-container");

    // If the drop target is a module or loop container, cancel the drop operation
    if (isModuleContainer || isLoopContainer) {
      return;
    }

    if (dragSrcEl != this) {
      // Determine the position where the dragged item will be dropped
      var dropPosition = parseInt(this.dataset.index, 10);

      var lbl = dragSrcEl.querySelector(".waypoint-label");
      if (lbl) {
        lbl.classList.remove("highlight-handle");
      }

      shiftLabels(initialIndex, dropPosition);
      shiftContainersBelowIndex(dropPosition);
      newIndex = dropPosition;

      console.log(
        "Item dragged from index",
        initialIndex,
        "to index",
        newIndex
      );

      if (newIndex != -1) {
        var move_goal = new ROSLIB.Goal({
          actionClient: interfaceClient,
          goalMessage: {
            request: 4,
            optional_index_1: initialIndex,
            optional_index_2: newIndex,
            optional_index_3: 0,
          },
        });

        move_goal.on("feedback", function (feedback) {
          console.log("Feedback: " + feedback.status);
        });
        move_goal.on("result", function (result) {
          console.log("Final Result: " + result.conclusion);
        });

        // Send the goal to the action server.
        move_goal.send();
      }
    }
    return false;
  }

  function dragEnd(e) {
    e.stopPropagation(); // Prevent event propagation
    var listItens = document.querySelectorAll(".draggable");
    [].forEach.call(listItens, function (item) {
      item.classList.remove("over");
    });
    this.style.opacity = "1";
    removeEventsDragAndDrop(this); // Remove event listeners
    updateIndices();
    addEventsDragAndDrop(this); // Re-add event listeners

    var userInputLabels = document.querySelectorAll("waypoint-label");
    userInputLabels.forEach(function (label) {
      label.contentEditable = "true";
    });

    dragSrcEl = null; // Clear the dragSrcEl reference
  }

  function updateIndices() {
    console.log("updateIndices");
    var items = draggableList.querySelectorAll("li:not(.module-container)");
    for (var i = 0; i < items.length; i++) {
      var waypoint = items[i];
      var waypointText = waypoint.querySelector(".waypoint-text");
      if (waypointText) {
        waypointText.innerText = "Waypoint " + (i + 1) + " ";
      }
      waypoint.dataset.index = i;

      // Remove the module label if present
      var moduleLabel = waypoint.querySelector(".module-label");
      if (moduleLabel) {
        waypoint.removeChild(moduleLabel);
      }

      // var dragHandle = document.createElement('span');
      // dragHandle.className = 'drag-handle';
      // //dragHandle.innerHTML = '<i class="uil uil-draggabledots"></i>';
      // waypoint.appendChild(dragHandle);

      /*var deleteIcon = document.createElement('i');
    deleteIcon.className = 'fas fa-trash-alt delete-icon';
    deleteIcon.addEventListener('click', deleteWaypoint);
    waypoint.appendChild(deleteIcon);*/
    }
    if (items) {
      nextIndex = items.length + 1;
    }
  }

  function shiftLabels(initialIndex, targetIndex) {
    var items = draggableList.querySelectorAll("li:not(.module-container)");

    var topOfLoop;
    var bottomOfLoop;
    var shiftDir;

    if (initialIndex < targetIndex) {
      topOfLoop = initialIndex;
      bottomOfLoop = targetIndex;
      shiftDir = "up";
    } else {
      topOfLoop = targetIndex;
      bottomOfLoop = initialIndex;
      shiftDir = "down";
    }

    if (shiftDir == "down") {
      // Store the contents of the last waypoint
      var lastItem = items[bottomOfLoop];
      // Check if the previous item is inside a container
      var j = bottomOfLoop;
      while (lastItem && lastItem.closest(".module-container")) {
        lastItem = items[j--];
      }
      var lastLabel = lastItem.querySelector(".waypoint-label");
      var lastSave = lastItem.querySelector(".waypoint-save-button");

      // Create a temporary storage element to hold the last contents
      var tempStorage = document.createElement("div");
      tempStorage.appendChild(lastLabel.cloneNode(true));
      tempStorage.appendChild(lastSave.cloneNode(true));

      // Loop through the waypoints and shift contents
      for (var i = bottomOfLoop; i > topOfLoop; i--) {
        var currentItem = items[i];
        var prevItem = items[i - 1];

        var currentLabel = currentItem.querySelector(".waypoint-label");
        var currentSave = currentItem.querySelector(".waypoint-save-button");

        var prevLabel, prevSave;
        var newLabel, newSave;

        // Check if the previous item is inside a container
        var j = i - 1;
        while (prevItem && prevItem.closest(".module-container")) {
          console.log("Previous item is in a container with index " + (i - 1));
          prevItem = items[j--];
        }

        // If prevItem is still a valid item, get its label and save button
        if (prevItem) {
          prevLabel = prevItem.querySelector(".waypoint-label");
          prevSave = prevItem.querySelector(".waypoint-save-button");
        }

        // Skip processing only if currentItem is inside a container
        if (
          prevItem &&
          currentItem &&
          !currentItem.closest(".module-container")
        ) {
          console.log("Index at " + i);
          console.log("Previous item index at " + prevItem.dataset.index);
          if (currentLabel && currentSave && prevLabel && prevSave) {
            // Move contents from previous item to the current item
            currentItem.removeChild(currentLabel);
            currentItem.removeChild(currentSave);
            newLabel = prevLabel.cloneNode(true);
            newSave = prevSave.cloneNode(true);
            applyLabelBehavior(newLabel, newSave);
            currentItem.appendChild(newLabel);
            currentItem.appendChild(newSave);
          }
        }
      }

      // Move the stored contents from tempStorage to the first waypoint
      var firstItem = items[topOfLoop];
      var j = topOfLoop;
      while (firstItem && firstItem.closest(".module-container")) {
        firstItem = items[j++];
      }

      // Check if the previous item is inside a container

      newLabel = tempStorage.firstChild;
      newSave = tempStorage.lastChild;
      applyLabelBehavior(newLabel, newSave);
      firstItem.removeChild(firstItem.querySelector(".waypoint-label"));
      firstItem.removeChild(firstItem.querySelector(".waypoint-save-button"));
      firstItem.appendChild(newLabel);
      firstItem.appendChild(newSave);

      // Clean up tempStorage
      tempStorage = null;
    } else if (shiftDir == "up") {
      console.log("Shifting up");
      // Store the contents of the last waypoint
      var firstItem = items[topOfLoop];
      // Check if the previous item is inside a container
      var j = topOfLoop;
      while (firstItem && firstItem.closest(".module-container")) {
        firstItem = items[j++];
      }
      var firstLabel = firstItem.querySelector(".waypoint-label");
      var firstSave = firstItem.querySelector(".waypoint-save-button");

      // Create a temporary storage element to hold the last contents
      var tempStorage = document.createElement("div");
      tempStorage.appendChild(firstLabel.cloneNode(true));
      tempStorage.appendChild(firstSave.cloneNode(true));

      // Loop through the waypoints and shift contents
      for (var i = topOfLoop; i < bottomOfLoop; i++) {
        var currentItem = items[i];
        var prevItem = items[i + 1];

        var currentLabel = currentItem.querySelector(".waypoint-label");
        var currentSave = currentItem.querySelector(".waypoint-save-button");

        var prevLabel, prevSave;
        var newLabel, newSave;

        // Check if the previous item is inside a container
        var j = i + 1;
        while (prevItem && prevItem.closest(".module-container")) {
          console.log("Previous item is in a container with index " + (i + 1));
          prevItem = items[j++];
        }

        // If prevItem is still a valid item, get its label and save button
        if (prevItem) {
          prevLabel = prevItem.querySelector(".waypoint-label");
          prevSave = prevItem.querySelector(".waypoint-save-button");
        }

        // Skip processing only if currentItem is inside a container
        if (
          prevItem &&
          currentItem &&
          !currentItem.closest(".module-container")
        ) {
          console.log("Index at " + i);
          console.log("Previous item index at " + prevItem.dataset.index);
          if (currentLabel && currentSave && prevLabel && prevSave) {
            // Move contents from previous item to the current item
            currentItem.removeChild(currentLabel);
            currentItem.removeChild(currentSave);
            newLabel = prevLabel.cloneNode(true);
            newSave = prevSave.cloneNode(true);
            applyLabelBehavior(newLabel, newSave);
            currentItem.appendChild(newLabel);
            currentItem.appendChild(newSave);
          } else {
            // Handle missing elements
            //console.log("Some elements are missing in item " + i);
          }
        }
      }

      // Move the stored contents from tempStorage to the first waypoint
      var lastItem = items[bottomOfLoop];
      var j = bottomOfLoop;
      while (lastItem && lastItem.closest(".module-container")) {
        lastItem = items[j--];
      }

      // Check if the previous item is inside a container
      lastItem.removeChild(lastItem.querySelector(".waypoint-label"));
      lastItem.removeChild(lastItem.querySelector(".waypoint-save-button"));
      newLabel = tempStorage.firstChild;
      newSave = tempStorage.lastChild;
      applyLabelBehavior(newLabel, newSave);
      lastItem.appendChild(newLabel);
      lastItem.appendChild(newSave);

      // Clean up tempStorage
      tempStorage = null;
    }
  }

  var alreadySentRun = false;
  var alreadySentRep = false;
  var alreadySentDel = false;

  // Context menu event handler
  function handleSingleItemContextMenu(event) {
    // Delete previous click event handlers
    deleteWaypointOption.removeEventListener("click", handleDeleteWaypoint);
    modifyWaypointOption.removeEventListener("click", handleModifyWaypoint);
    runWaypointOption.removeEventListener("click", handleRunWaypoint);

    event.preventDefault();
    var targetItem = event.target.closest("li");

    // Show the context menu at the click position
    singleItemContextMenu.style.display = "block";
    singleItemContextMenu.style.left = event.pageX + "px";
    singleItemContextMenu.style.top = event.pageY + "px";

    // Delete Waypoint option click event handler
    function handleDeleteWaypoint() {
      if (targetItem) {
        //const listItem = event.target.closest('li');
        //const draggableList = document.getElementById('draggable-list');
        //const index = Array.from(draggableList.children).indexOf(listItem);
        //const index = parseInt(targetItem.dataset.index);
        const index = lastHighlighted;

        targetItem.remove();
        if ((index >= 0) & !alreadySentDel) {
          console.log("Index: " + index);
          alreadySentDel = true;
          /*markerClient.removeMarker("sphere" + (index + 1));
          markerClient.removeMarker("text" + (index + 1));*/
          var del_goal = new ROSLIB.Goal({
            actionClient: interfaceClient,
            goalMessage: {
              request: 3,
              optional_index_1: index,
              optional_index_2: 0,
              optional_index_3: 0,
            },
          });
          del_goal.on("feedback", function (feedback) {
            console.log("Feedback: " + feedback.status);
          });
          del_goal.on("result", function (result) {
            console.log("Final Result: " + result.conclusion);
            alreadySentDel = false;
          });

          // Send the goal to the action server.
          del_goal.send();

          // Deselect the item
          selectedItems = selectedItems.filter(
            (selectedItem) => selectedItem !== index
          );
          //sendDeselectGoal(index);
        }
        updateIndices();
      } else {
        console.log("Target item is false");
      }
      singleItemContextMenu.style.display = "none";
    }

    // Modify Waypoint option click event handler
    function handleModifyWaypoint() {
      // if (currentAssistanceState == "replaceWaypoint")
      // {
      //   initializeAssistant();
      // }

      if (targetItem) {
        if (currentProgAssistanceState == "exitEditExistingReplace") {
          automaticProceedChat = true;
          handleResponseOptionClick(
            lastResponseContainer,
            currentProgAssistanceState
          );
        }
        //const listItem = event.target.closest('li');
        //const draggableList = document.getElementById('draggable-list');
        //const index = Array.from(draggableList.children).indexOf(listItem);
        //const index = parseInt(listItem.dataset.index);
        //console.log("Index: " + index);
        const index = lastHighlighted;
        if ((index >= 0) & !alreadySentRep) {
          alreadySentRep = true;
          var rep_goal = new ROSLIB.Goal({
            actionClient: interfaceClient,
            goalMessage: {
              request: 5,
              optional_index_1: index,
              optional_index_2: 0,
              optional_index_3: 0,
            },
          });
          rep_goal.on("feedback", function (feedback) {
            console.log("Feedback: " + feedback.status);
          });
          rep_goal.on("result", function (result) {
            console.log("Final Result: " + result.conclusion);
            alreadySentRep = false;
          });

          // Send the goal to the action server.
          rep_goal.send();
        }
      }
      singleItemContextMenu.style.display = "none";
    }

    // Execute Waypoint option click event handler
    function handleRunWaypoint() {
      if (targetItem) {
        if (currentProgAssistanceState == "exitEditExistingMove") {
          automaticProceedChat = true;
          handleResponseOptionClick(
            lastResponseContainer,
            currentProgAssistanceState
          );
        }
        const index = lastHighlighted;
        //console.log("Index: " + index);
        if (index >= 0 && !alreadySentRun) {
          alreadySentRun = true;
          console.log("Sending run to goal");
          var run_to_goal = new ROSLIB.Goal({
            actionClient: interfaceClient,
            goalMessage: {
              request: 8,
              optional_index_1: index,
              optional_index_2: 0,
              optional_index_3: 0,
            },
          });
          run_to_goal.on("feedback", function (feedback) {
            console.log("Feedback: " + feedback.status);
          });
          run_to_goal.on("result", function (result) {
            console.log("Final Result: " + result.conclusion);
            speak("The robot is done moving.");
            alreadySentRun = false;
          });

          // Send the goal to the action server.
          run_to_goal.send();
          runWaypointOption.removeEventListener("click", handleRunWaypoint);
        }
      }
      singleItemContextMenu.style.display = "none";
    }

    // Delete previous click event handlers
    deleteWaypointOption.removeEventListener("click", handleDeleteWaypoint);
    modifyWaypointOption.removeEventListener("click", handleModifyWaypoint);
    runWaypointOption.removeEventListener("click", handleRunWaypoint);

    // Add new click event handlers
    deleteWaypointOption.addEventListener("click", handleDeleteWaypoint);
    modifyWaypointOption.addEventListener("click", handleModifyWaypoint);
    runWaypointOption.addEventListener("click", handleRunWaypoint);

    // Hide the context menu when clicking outside of it
    document.addEventListener(
      "click",
      function () {
        singleItemContextMenu.style.display = "none";
      },
      { once: true }
    );
  }

  // Add click event listener to the "Add waypoints to a module" option
  var addModuleOption = document.getElementById("add-module");
  addModuleOption.addEventListener("click", handleAddModuleOption);

  var draggableList = document.getElementById("draggable-list");

  function createModuleContainer() {
    // Create the module container element
    var moduleContainer = document.createElement("div");
    moduleContainer.className = "module-container";

    // Create the trash icon element
    var trashIcon = document.createElement("span");
    trashIcon.className = "trash-icon";
    trashIcon.innerHTML = '<i class="fas fa-trash"></i>';
    trashIcon.addEventListener("click", function (event) {
      event.stopPropagation();

      // Get the module waypoints list
      var moduleWaypointsList =
        moduleContainer.querySelector(".module-waypoints");

      // Detach the waypoints from the module container
      var waypoints = Array.from(moduleWaypointsList.childNodes);
      waypoints.forEach(function (waypoint) {
        moduleWaypointsList.removeChild(waypoint);
      });

      // Find the draggable list
      var draggableList = document.getElementById("draggable-list");

      // Find the index of the module container within the draggable list
      var moduleIndex = Array.from(draggableList.children).indexOf(
        moduleContainer
      );

      // Insert the waypoints back into the draggable list at their original indices
      var insertIndex = moduleIndex;
      waypoints.forEach(function (waypoint) {
        waypoint.draggable = true;
        addEventsDragAndDrop(waypoint);
        waypoint.removeEventListener(
          "contextmenu",
          handleModuleWaypointRightClick
        );
        waypoint.addEventListener("contextmenu", showContextMenu);
        waypoint.addEventListener("contextmenu", handleItemClick);
        waypoint.addEventListener("click", handleItemClick);
        var deleteIcon = document.createElement("i");
        deleteIcon.className = "fas fa-trash-alt delete-icon";
        deleteIcon.addEventListener("click", deleteWaypoint);
        var dragHandle = waypoint.querySelector(".drag-handle");
        applyTooltipBehavior(dragHandle, "Drag");
        applyTooltipBehavior(deleteIcon, "Delete waypoint");
        waypoint.appendChild(deleteIcon);
        draggableList.insertBefore(
          waypoint,
          draggableList.children[insertIndex]
        );
        insertIndex++;
      });

      // Remove the module container
      moduleContainer.remove();
    });
    moduleContainer.appendChild(trashIcon);

    // Create the module name label element
    var moduleNameLabel = document.createElement("label");
    moduleNameLabel.className = "module-name-label";
    moduleNameLabel.textContent = "Module Name";
    moduleContainer.appendChild(moduleNameLabel);

    // Create the module name input field element
    var moduleNameInput = document.createElement("input");
    moduleNameInput.className = "module-name-input";
    moduleNameInput.type = "text";
    moduleNameInput.value = "Module Name";
    moduleNameInput.style.display = "none";
    moduleContainer.appendChild(moduleNameInput);

    // Create the save button element
    var saveButton = document.createElement("button");
    saveButton.className = "module-save-button";
    saveButton.textContent = "Save";
    saveButton.style.display = "none";
    saveButton.addEventListener("click", handleSaveButtonClick);
    moduleContainer.appendChild(saveButton);

    // Add click event listener to the module name label
    moduleNameLabel.addEventListener("click", handleLabelClick);

    // Create the module waypoints list element
    var moduleWaypointsList = document.createElement("ul");
    moduleWaypointsList.className = "module-waypoints";
    moduleContainer.appendChild(moduleWaypointsList);

    moduleContainer.setAttribute("data-droppable", "false");

    return moduleContainer;
  }

  function handleModuleWaypointRightClick(event) {
    // Handle right-click on waypoint
    event.preventDefault();
    alert(
      "Editing waypoints inside modules is current disabled. Please remove the module container to edit this waypoint. You can add the module container back after making the edit."
    );
  }

  function handleAddModuleOption(event) {
    event.preventDefault();

    if (
      currentProgAssistanceState == "singleMotion4" ||
      currentProgAssistanceState == "pickAndMove7_3" ||
      currentProgAssistanceState == "pickAndMove3_3" ||
      currentProgAssistanceState == "pickAndPlace3_3" ||
      currentProgAssistanceState == "pickAndPlace7_3" ||
      currentProgAssistanceState == "pickAndHold3_3" ||
      currentProgAssistanceState == "pickAndHold7_3"
    ) {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }

    // Create a new module container
    var moduleContainer = createModuleContainer();

    // Add selected waypoints to the module container
    var waypoints = selectedItems.map(function (index) {
      var waypoint = draggableList.children[index].cloneNode(true);

      waypoint.draggable = false;
      removeEventsDragAndDrop(waypoint);
      /*waypoint.addEventListener('contextmenu', showContextMenu);*/
      waypoint.addEventListener("contextmenu", handleItemClick);
      waypoint.addEventListener("click", handleItemClick);

      waypoint.addEventListener("contextmenu", handleModuleWaypointRightClick);
      removeDeleteIconFromWaypoint(waypoint);

      // Update the list label of the cloned waypoint
      var waypointText = waypoint.querySelector(".waypoint-text");
      if (waypointText) {
        waypointText.innerText = "Waypoint " + (index + 1) + " ";
      }
      var waypointLabel = waypoint.querySelector(".waypoint-label");
      var saveButton = waypoint.querySelector(".waypoint-save-button");

      // Call applyLabelBehavior with the obtained elements
      applyLabelBehavior(waypointLabel, saveButton);
      waypoint.dataset.index = index;
      // Add the selected waypoint to the module waypoints list
      return waypoint;
    });

    // Remove the selected waypoints from the draggable list
    highlightedItems
      .sort(function (a, b) {
        return b - a; // Sort the indices in descending order
      })
      .forEach(function (index) {
        var waypoints = draggableList.querySelectorAll(
          "li:not(.module-container)"
        );
        if (index >= 0 && index < waypoints.length) {
          var waypoint = waypoints[index];
          draggableList.removeChild(waypoint);
        }
      });

    // Determine the insert index for the module container
    var insertIndex = Math.min.apply(null, selectedItems);

    // Insert the module container at the correct position in the draggable list
    draggableList.insertBefore(
      moduleContainer,
      draggableList.children[insertIndex]
    );

    // Append the waypoints to the module container
    var moduleWaypointsList =
      moduleContainer.querySelector(".module-waypoints");
    waypoints.forEach(function (waypoint) {
      moduleWaypointsList.appendChild(waypoint);
    });

    // Clear the selected items array
    dehighlightAllWaypoints();
    updateIndices();
  }

  function hideModuleContainer() {
    var moduleContainer = document.getElementById("module-container");
    moduleContainer.style.display = "none";
  }

  function handleMultiItemContextMenu(event) {
    event.preventDefault();

    var targetItem = event.currentTarget;

    // Hide the context menu when clicking outside of it
    document.addEventListener(
      "click",
      function () {
        multiItemContextMenu.style.display = "none";
      },
      { once: true }
    );
  }

  // Function to handle the save button click event
  function handleSaveButtonClick(event) {
    var saveButton = event.currentTarget;
    var moduleNameInput = saveButton.previousElementSibling;
    var moduleNameLabel = moduleNameInput.previousElementSibling;

    if (
      currentProgAssistanceState == "pickAndMove7_4" ||
      currentProgAssistanceState == "pickAndMove3_4" ||
      currentProgAssistanceState == "pickAndPlace3_4" ||
      currentProgAssistanceState == "pickAndPlace7_4" ||
      currentProgAssistanceState == "pickAndHold3_4" ||
      currentProgAssistanceState == "pickAndHold7_4" ||
      currentProgAssistanceState == "singleMotion5"
    ) {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }

    // Get the entered module name from the input field
    var moduleName = moduleNameInput.value;

    // Update the module name label with the entered name
    moduleNameLabel.textContent = moduleName;

    // Toggle the visibility of the input field and label
    moduleNameInput.style.display = "none";
    moduleNameLabel.style.display = "inline";

    // Toggle the visibility of the save button
    saveButton.style.display = "none";
  }

  // Function to handle the label click event
  function handleLabelClick(event) {
    var moduleNameLabel = event.currentTarget;
    var moduleNameInput = moduleNameLabel.nextElementSibling;
    var saveButton = moduleNameInput.nextElementSibling;

    // Toggle the visibility of the input field and label
    moduleNameInput.style.display = "inline";
    moduleNameLabel.style.display = "none";

    // Toggle the visibility of the save button
    saveButton.style.display = "inline";

    // Set focus on the input field
    moduleNameInput.focus();
  }

  function createLoopContainer() {
    // Create the loop container element
    var loopContainer = document.createElement("div");
    loopContainer.className = "module-container loop-container";

    // Set the container color to purple
    loopContainer.style.backgroundColor = "#CF9FFF";

    // Create the trash icon element
    var trashIcon = document.createElement("span");
    trashIcon.className = "trash-icon";
    trashIcon.innerHTML = '<i class="fas fa-trash"></i>';
    trashIcon.addEventListener("click", function (event) {
      event.stopPropagation();

      updateLoopIndices(loopContainer);

      // Get the loop waypoints list
      var loopWaypointsList = loopContainer.querySelector(".module-waypoints");

      // Detach the waypoints from the loop container
      var waypoints = Array.from(loopWaypointsList.childNodes);
      waypoints.forEach(function (waypoint) {
        loopWaypointsList.removeChild(waypoint);
      });

      // Find the draggable list
      var draggableList = document.getElementById("draggable-list");

      // Find the index of the loop container within the draggable list
      var loopIndex = Array.from(draggableList.children).indexOf(loopContainer);

      // Insert the waypoints back into the draggable list at their original indices
      var insertIndex = loopIndex;
      waypoints.forEach(function (waypoint) {
        waypoint.draggable = true;
        addEventsDragAndDrop(waypoint);
        waypoint.removeEventListener(
          "contextmenu",
          handleLoopWaypointRightClick
        );
        waypoint.addEventListener("contextmenu", showContextMenu);
        waypoint.addEventListener("contextmenu", handleItemClick);
        waypoint.addEventListener("click", handleItemClick);
        var deleteIcon = document.createElement("i");
        deleteIcon.className = "fas fa-trash-alt delete-icon";
        deleteIcon.addEventListener("click", deleteWaypoint);
        var dragHandle = waypoint.querySelector(".drag-handle");
        applyTooltipBehavior(dragHandle, "Drag");
        applyTooltipBehavior(deleteIcon, "Delete waypoint");
        waypoint.appendChild(deleteIcon);
        draggableList.insertBefore(
          waypoint,
          draggableList.children[insertIndex]
        );
        insertIndex++;
      });

      // Remove the loop container
      loopContainer.remove();

      var del_loop_goal = new ROSLIB.Goal({
        actionClient: interfaceClient,
        goalMessage: {
          request: 12,
          optional_index_1: parseInt(loopContainer.dataset.topIndex),
          optional_index_2: parseInt(loopContainer.dataset.bottomIndex),
          optional_index_3: 0,
        },
      });
      del_loop_goal.on("feedback", function (feedback) {
        console.log("Feedback: " + feedback.status);
      });
      del_loop_goal.on("result", function (result) {
        console.log("Final Result: " + result.conclusion);
      });

      // Send the goal to the action server.
      del_loop_goal.send();
    });
    loopContainer.appendChild(trashIcon);

    // Create the loop label element
    var loopLabel = document.createElement("label");
    loopLabel.className = "module-name-label loop-label";
    loopLabel.textContent = "Repeat";
    loopContainer.appendChild(loopLabel);

    // Add space after the loop label
    loopLabel.style.marginRight = "0.5rem";

    // Create the loop input element
    var loopInput = document.createElement("input");
    loopInput.className = "module-name-input loop-input";
    loopInput.type = "number";
    loopInput.min = "1";
    loopInput.value = "1";
    loopInput.style.fontSize = "1.2rem"; // Adjust the font size
    loopInput.style.display = "inline-block"; // Adjust the input field width
    loopContainer.appendChild(loopInput);

    // Add space after the loop input
    loopInput.style.marginRight = "0.5rem";

    // Create the loop times label element
    var loopTimesLabel = document.createElement("label");
    loopTimesLabel.className = "module-name-label loop-times-label";
    loopTimesLabel.textContent = "times";
    loopContainer.appendChild(loopTimesLabel);

    // Hidden span to calculate the width based on the entered value
    var hiddenSpan = document.createElement("span");
    hiddenSpan.style.visibility = "hidden";
    hiddenSpan.style.whiteSpace = "pre";
    hiddenSpan.style.fontSize = "1.2rem";
    hiddenSpan.textContent = loopInput.value;
    document.body.appendChild(hiddenSpan);

    hiddenSpan.textContent = loopInput.value;
    loopInput.style.width = hiddenSpan.offsetWidth + 35 + "px";

    // Update the input field width based on the entered value
    loopInput.addEventListener("input", function () {
      hiddenSpan.textContent = loopInput.value;
      loopInput.style.width = hiddenSpan.offsetWidth + 35 + "px";
    });

    // Set data attributes to store topmost and bottommost indices
    loopContainer.dataset.topIndex = -1;
    loopContainer.dataset.bottomIndex = -1;

    // Create the loop waypoints list element
    var loopWaypointsList = document.createElement("ul");
    loopWaypointsList.className = "module-waypoints loop-waypoints";
    loopContainer.appendChild(loopWaypointsList);

    loopInput.addEventListener("input", function (event) {
      updateLoopIndices(loopContainer);
      var loopCountValue = parseInt(event.target.value);
      console.log("Loop Count Updated:", loopCountValue);
      console.log("Loop Count Updated top:", loopContainer.dataset.topIndex);
      console.log(
        "Loop Count Updated bottom:",
        loopContainer.dataset.bottomIndex
      );

      // Send the loop to the backend
      console.log("Sending loop from createLoopContainer");
      var send_loop_goal = new ROSLIB.Goal({
        actionClient: interfaceClient,
        goalMessage: {
          request: 11,
          optional_index_1: loopCountValue,
          optional_index_2: parseInt(loopContainer.dataset.topIndex),
          optional_index_3: parseInt(loopContainer.dataset.bottomIndex),
        },
      });
      send_loop_goal.on("feedback", function (feedback) {
        console.log("Feedback: " + feedback.status);
      });
      send_loop_goal.on("result", function (result) {
        console.log("Final Result: " + result.conclusion);
      });

      // Send the goal to the action server.
      send_loop_goal.send();

      if (currentProgAssistanceState == "finishedMotion") {
        handleItemClick(lastResponseContainer, currentProgAssistanceState);
      }
    });

    loopContainer.setAttribute("data-droppable", "false");

    return loopContainer;
  }

  function updateLoopIndices(loopContainer) {
    // Get the loop waypoints list
    var loopWaypointsList = loopContainer.querySelector(".module-waypoints");

    // Get all waypoints within the loop container
    var loopWaypoints = loopWaypointsList.querySelectorAll("li");

    if (loopWaypoints.length > 0) {
      // Extract the indices from data attributes and convert them to integers
      var indices = Array.from(loopWaypoints, (waypoint) =>
        parseInt(waypoint.dataset.index, 10)
      );

      // Check if any of the indices are NaN
      if (indices.some(isNaN)) {
        console.error("Error: Some indices are NaN");
        console.log("Indices:", indices);
      }

      // Sort the indices to get the topmost and bottommost values
      indices.sort((a, b) => a - b);

      // Update the data attributes with the indices of the topmost and bottommost items
      var topIndex = indices[0];
      var bottomIndex = indices[indices.length - 1];

      loopContainer.dataset.topIndex = parseInt(topIndex);
      loopContainer.dataset.bottomIndex = parseInt(bottomIndex);
    } else {
      // No waypoints in the loop container, set indices to -1
      loopContainer.dataset.topIndex = -1;
      loopContainer.dataset.bottomIndex = -1;
    }

    /*// Logging the indices for debugging purposes
  console.log('Loop Container Top Index:', loopContainer.dataset.topIndex);
  console.log('Loop Container Bottom Index:', loopContainer.dataset.bottomIndex);*/
  }

  function handleLoopContainerClick(event) {
    event.stopPropagation();

    var loopContainer = event.currentTarget;

    // Toggle the input field visibility
    var loopCountInput = loopContainer.querySelector(".module-name-input");
    loopCountInput.style.display =
      loopCountInput.style.display === "none" ? "inline-block" : "none";

    // Toggle the label visibility
    var loopCountLabel = loopContainer.querySelector(".module-name-label");
    loopCountLabel.style.display =
      loopCountLabel.style.display === "none" ? "inline-block" : "none";
  }

  function handleLoopWaypointRightClick(event) {
    // Handle right-click on waypoint
    event.preventDefault();
    alert(
      "Editing waypoints inside loops is current disabled. Please remove the loop container to edit this waypoint. You can add the loop container back after making the edit."
    );
  }

  function handleAddLoopOption(event) {
    event.preventDefault();

    // Create a new loop container
    var loopContainer = createLoopContainer();

    var waypoints = selectedItems.map(function (index) {
      var waypoint = draggableList.children[index].cloneNode(true);
      waypoint.draggable = false;
      removeEventsDragAndDrop(waypoint);
      /*addEventsDragAndDrop(waypoint);
    waypoint.addEventListener('contextmenu', showContextMenu);*/
      waypoint.addEventListener("contextmenu", handleItemClick);
      waypoint.addEventListener("click", handleItemClick);
      waypoint.addEventListener("contextmenu", handleLoopWaypointRightClick);
      removeDeleteIconFromWaypoint(waypoint);

      // Update the list label of the cloned waypoint
      var waypointText = waypoint.querySelector(".waypoint-text");
      if (waypointText) {
        waypointText.innerText = "Waypoint " + (index + 1) + " ";
      }
      var waypointLabel = waypoint.querySelector(".waypoint-label");
      var saveButton = waypoint.querySelector(".waypoint-save-button");

      // Call applyLabelBehavior with the obtained elements
      applyLabelBehavior(waypointLabel, saveButton);
      waypoint.dataset.index = index;

      // Add the selected waypoint to the loop waypoints list
      return waypoint;
    });

    // Remove the selected waypoints from the draggable list
    highlightedItems
      .sort(function (a, b) {
        return b - a; // Sort the indices in descending order
      })
      .forEach(function (index) {
        var waypoints = draggableList.querySelectorAll(
          "li:not(.module-container)"
        );
        if (index >= 0 && index < waypoints.length) {
          var waypoint = waypoints[index];
          draggableList.removeChild(waypoint);
        }
      });

    // Determine the insert index for the loop container
    var insertIndex = Math.min.apply(null, selectedItems);

    // Insert the loop container at the correct position in the draggable list
    draggableList.insertBefore(
      loopContainer,
      draggableList.children[insertIndex]
    );

    // Append the waypoints to the loop container
    var loopWaypointsList = loopContainer.querySelector(".module-waypoints");
    waypoints.forEach(function (waypoint) {
      loopWaypointsList.appendChild(waypoint);
    });

    // Use setTimeout to ensure dataset.index is updated before updating loop indices
    setTimeout(function () {
      // Update the topmost and bottommost indices after adding waypoints
      updateLoopIndices(loopContainer);
    }, 0);

    // Get the loopCount input element from the newly created loopContainer
    var loopCountInput = loopContainer.querySelector(".module-name-input");

    // Get the current value of the loopCount input
    var loopCountValue = parseInt(loopCountInput.value);

    setTimeout(function () {
      // Send the loop to the backend
      var send_loop_goal = new ROSLIB.Goal({
        actionClient: interfaceClient,
        goalMessage: {
          request: 11,
          optional_index_1: loopCountValue,
          optional_index_2: parseInt(loopContainer.dataset.topIndex),
          optional_index_3: parseInt(loopContainer.dataset.bottomIndex),
        },
      });
      send_loop_goal.on("feedback", function (feedback) {
        console.log("Feedback: " + feedback.status);
      });
      send_loop_goal.on("result", function (result) {
        console.log("Final Result: " + result.conclusion);
      });

      // Send the goal to the action server.
      send_loop_goal.send();
    }, 0);

    // Clear the selected items array
    dehighlightAllWaypoints();
    updateIndices();

    if (currentProgAssistanceState == "repeatMotion4") {
      automaticProceedChat = true;
      handleResponseOptionClick(
        lastResponseContainer,
        currentProgAssistanceState
      );
    }
  }

  // Add click event listener to the "Add waypoints to a loop" option
  var addLoopOption = document.getElementById("add-loop");
  addLoopOption.addEventListener("click", handleAddLoopOption);

  function clearProgram() {
    var clear_goal = new ROSLIB.Goal({
      actionClient: interfaceClient,
      goalMessage: {
        request: 9,
        optional_index_1: 0,
        optional_index_2: 0,
        optional_index_3: 0,
      },
    });
    clear_goal.on("feedback", function (feedback) {
      console.log("Feedback: " + feedback.status);
    });
    clear_goal.on("result", function (result) {
      console.log("Final Result: " + result.conclusion);
    });

    // Send the goal to the action server.
    clear_goal.send();

    var clearMarkersMessage = new ROSLIB.Message({
      num: 0,
      swap_id1: 0,
      swap_id2: 0,
      extra_num: 0,
    });

    markerTopic.publish(clearMarkersMessage);

    // Get the parent container element of the draggable list
    var draggableList = document.getElementById("draggable-list");

    // Remove all child elements (list items) from the container
    while (draggableList.firstChild) {
      draggableList.removeChild(draggableList.firstChild);
    }

    nextIndex = 1;
  }

  function newProgram() {
    // Display a confirmation popup
    const confirmed = confirm(
      "Are you sure you want to erase your current program?"
    );

    // If the user confirmed, execute the runProgram() function
    if (confirmed) {
      if (
        currentProgAssistanceState == "startPickAndPlace" ||
        currentProgAssistanceState == "startPickAndHold" ||
        currentProgAssistanceState == "startPickAndMove"
      ) {
        automaticProceedChat = true;
        handleResponseOptionClick(
          lastResponseContainer,
          currentProgAssistanceState
        );
      }
      clearProgram();
    }
  }
});

//switch tab
function switchTab() {
  const targets = document.getElementsByClassName("tabcontent");
  for (var i = 0; i < targets.length; i++) {
    targets[i].style.display = "none";
  }
}

//check if button
function buttonCheck(item) {
  if (
    item == "run-program" ||
    item == "add-waypoint" ||
    item == "new-program"
  ) {
    return true;
  } else {
    return false;
  }
}

// open instruction
function instructionHandler(
  event,
  instructionText,
  instructions,
  i,
  highlights,
  toggles
) {
  const buttonContainer = event.target.parentNode;
  const instructionContainer = buttonContainer.nextElementSibling;
  const toggle = instructionContainer.childNodes[1];
  const toggleInfo = instructionContainer.childNodes[2];

  if (toggles[i][0].length != 0) {
    toggle.style.dispay = "block";
    for (var l = 0; l < toggles[i][0].length; l++) {
      toggle.textContent = toggles[i][0][l][0];
      toggleInfo.textContent = toggles[i][0][l][1];
    }
  }

  if (highlights[i][0].length != 0) {
    for (var j = 0; j < highlights[i][0].length; j++) {
      if (buttonCheck(highlights[i][0][j]) == true) {
        document.getElementById(highlights[i][0][j]).classList.add("highlight");
      } else {
        document
          .getElementById(highlights[i][0][j])
          .classList.add("highlight-border");
      }
    }
  }

  buttonContainer.style.display = "none";
  instructionContainer.style.display = "block";
}

// clear all previous highlights and toggles for instructions
function clear() {
  const highlightedElements = document.querySelectorAll(
    ".highlight, .highlight-border"
  );

  const toggleElements = document.querySelectorAll(".toggle"); // Select all elements with class 'toggle'

  toggleElements.forEach((element) => {
    element.style.display = "none"; // Hide each toggle element
  });

  highlightedElements.forEach((element) => {
    element.classList.remove("highlight");
    element.classList.remove("highlight-border");
  });
}

// tab handler
function tabHandler(
  event,
  topics,
  instructions,
  highlights,
  contentsContainer,
  toggles
) {
  clear();
  switchTab();

  let currentStepIndex = 0;
  let currentTopicId = -1;

  const toggle = document.createElement("a");
  toggle.id = toggle;
  toggle.className = "toggle";
  toggle.href = "#";

  const toggleInfo = document.createElement("p");
  toggleInfo.id = "toggleInfo";
  toggleInfo.className = "toggle";
  toggleInfo.style.display = "none";

  toggle.addEventListener("click", () => {
    if (toggleInfo.style.display === "block") {
      toggleInfo.style.display = "none";
    } else {
      toggleInfo.style.display = "block";
    }
  });

  const tabContent = document.createElement("div");
  tabContent.className = "tabcontent";
  tabContent.id = "tabcontent";
  tabContent.style.display = "block";

  const buttonContainer = document.createElement("div");
  buttonContainer.className = "button-container";
  buttonContainer.id = "button-container";
  buttonContainer.style.display = "block";

  const instructionContainer = document.createElement("div");
  instructionContainer.className = "instruction-container";
  instructionContainer.id = "instruction-container";
  instructionContainer.style.display = "none";

  const instructionText = document.createElement("p");
  instructionText.className = "instruction-text";

  for (var i = 0; i < topics.length; i++) {
    const topic = document.createElement("button");
    topic.id = i;
    topic.className = "button button2";
    topic.textContent = topics[i];

    const clickHandler = (topicId) => {
      return (event) => {
        currentTopicId = topicId;
        instructionText.textContent = instructions[currentTopicId][0];
        instructionHandler(
          event,
          instructionText,
          instructions,
          topicId,
          highlights,

          toggles
        );
      };
    };

    topic.onclick = clickHandler(i);
    buttonContainer.appendChild(topic);
  }

  const assistButtonContainer = document.createElement("div");
  assistButtonContainer.className = "assist-button-container";

  const backButton = document.createElement("button");
  backButton.className = "button button1";
  backButton.textContent = "←";

  const nextButton = document.createElement("button");
  nextButton.className = "button button1";
  nextButton.textContent = "→";

  instructionContainer.appendChild(instructionText);
  instructionContainer.appendChild(toggle);
  instructionContainer.appendChild(toggleInfo);
  instructionContainer.appendChild(assistButtonContainer);

  assistButtonContainer.appendChild(backButton);
  assistButtonContainer.appendChild(nextButton);

  tabContent.appendChild(buttonContainer);
  tabContent.appendChild(instructionContainer);
  contentsContainer.appendChild(tabContent);

  backButton.onclick = () => {
    if (currentStepIndex == 0) {
      instructionContainer.style.display = "none";
      buttonContainer.style.display = "block";
      clear();
    } else {
      currentStepIndex--;
      clear();
      updateInstructionText(currentTopicId);
    }
  };

  nextButton.onclick = () => {
    if (currentStepIndex == instructions[currentTopicId].length - 1) {
      instructionContainer.style.display = "none";
      buttonContainer.style.display = "block";
      currentStepIndex = 0;
      clear();
    } else {
      currentStepIndex++;
      clear();

      updateInstructionText(currentTopicId);
    }
    console.log(highlights[currentTopicId]);
  };

  function updateInstructionText(topicId) {
    clear();
    instructionText.textContent = instructions[topicId][currentStepIndex];

    if (toggles[topicId][currentStepIndex].length != 0) {
      for (var l = 0; l < toggles[topicId][currentStepIndex].length; l++) {
        toggle.style.display = "block";
        console.log(toggle);
        toggle.textContent = toggles[topicId][currentStepIndex][l][0];
        toggleInfo.textContent = toggles[topicId][currentStepIndex][l][1];
      }
    }

    if (highlights[topicId][currentStepIndex].length !== 0) {
      for (var j = 0; j < highlights[topicId][currentStepIndex].length; j++) {
        if (buttonCheck(highlights[topicId][currentStepIndex][j]) == true) {
          document
            .getElementById(highlights[topicId][currentStepIndex][j])
            .classList.add("highlight");
        } else {
          document
            .getElementById(highlights[topicId][currentStepIndex][j])
            .classList.add("highlight-border");
        }
      }
    } //else {
    //   clear();
    // }
  }
}

const beforeUnloadListener = (event) => {
  setTimeout(() => alert("hi!"));
  event.preventDefault();
  return (event.returnValue = "Are you sure you want to exit?");
};

window.onbeforeunload = function (event) {
  // Deactivate teaching mode on the Kinova arm
  var stop_teach_goal = new ROSLIB.Goal({
    actionClient: interfaceClient,
    goalMessage: {
      request: 1,
      optional_index_1: 0,
      optional_index_2: 0,
      optional_index_3: 0,
    },
  });
  stop_teach_goal.on("feedback", function (feedback) {
    console.log("Feedback: " + feedback.status);
  });
  stop_teach_goal.on("result", function (result) {
    console.log("Final Result: " + result.conclusion);
  });

  // Send the goal to the action server.
  stop_teach_goal.send();

  var clear_goal = new ROSLIB.Goal({
    actionClient: interfaceClient,
    goalMessage: {
      request: 9,
      optional_index_1: 0,
      optional_index_2: 0,
      optional_index_3: 0,
    },
  });
  clear_goal.on("feedback", function (feedback) {
    console.log("Feedback: " + feedback.status);
  });
  clear_goal.on("result", function (result) {
    console.log("Final Result: " + result.conclusion);
  });

  // Send the goal to the action server.
  clear_goal.send();

  var clearMarkersMessage = new ROSLIB.Message({
    num: 0,
    swap_id1: 0,
    swap_id2: 0,
    extra_num: 0,
  });

  markerTopic.publish(clearMarkersMessage);
};

// Get all waypoints inside module or loop containers
const moduleWaypoints = document.querySelectorAll(
  ".module-container li, .loop-container li"
);

// Add event listeners for mouseover and mouseout
moduleWaypoints.forEach((waypoint) => {
  waypoint.addEventListener("mouseover", () => {
    waypoint.style.cursor = "pointer";
  });

  waypoint.addEventListener("mouseout", () => {
    waypoint.style.cursor = "default";
  });
});

function speak(text) {
  const utterance = new SpeechSynthesisUtterance(text);
  const voices = speechSynthesis.getVoices();
  utterance.voice = voices[2];
  speechSynthesis.speak(utterance);
}
