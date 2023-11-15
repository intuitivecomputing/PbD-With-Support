var interfaceClient;
var markerClient;
var markerTopic;

function init() 
{
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
      document.getElementById('connecting').style.display = 'none';
      document.getElementById('connected').style.display = 'none';
      document.getElementById('closed').style.display = 'none';
      document.getElementById('error').style.display = 'inline';
      console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
      console.log('Connection made!');
      document.getElementById('connecting').style.display = 'none';
      document.getElementById('error').style.display = 'none';
      document.getElementById('closed').style.display = 'none';
      document.getElementById('connected').style.display = 'inline';
    });

    ros.on('close', function() {
    console.log('Connection closed.');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'inline';
    });

    // The ActionClient
    // ----------------

    interfaceClient = new ROSLIB.ActionClient({
      ros : ros,
      serverName : '/interface_controller',
      actionName : 'prog_support_backend/InterfaceRequestAction'
    });

    // Activate teaching mode on the Kinova arm
    var teach_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 0,
        optional_index_1: 0,
        optional_index_2: 0
      }
    });
    teach_goal.on('feedback', function(feedback) {
      console.log('Feedback: ' + feedback.status);
    });
    teach_goal.on('result', function(result) {
      console.log('Final Result: ' + result.conclusion);
    });

    // Send the goal to the action server.
    teach_goal.send();

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'program-visualization',
      width : 590,
      height : 600,
      displayPanAndZoomFrame : false,
      antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    viewer.cameraControls.zoomIn(2);

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/noetic-devel/',
      rootObject : viewer.scene,
    });

    // Setup the marker client.
    markerClient = new ROS3D.MarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/visualization_marker',
      lifetime : 0,
      rootObject : viewer.scene
    });


    markerTopic = new ROSLIB.Topic({
      ros : ros,
      name : 'waypoint',
      messageType : 'marker_package/Waypoint'
    });

    initializeAssistant();
}

let currentInstructionIndex = 0;
let currentAssistanceState = "none";

function initializeAssistant()
{
  stopSpeech();
  clearAllGuides();
  currentInstructionIndex = 0;
  var textBubbleContent = document.getElementById('text-bubble-content');
  textBubbleContent.textContent = "Hello, I'm here to help you!";
  document.getElementById("instruction-container").style.display = "none";

  if (currentAssistanceState != "none")
  {
    if (currentAssistanceState == "createProgram")
    {
      updateTextBubble("Great job! You created a program.");
    }
    else if (currentAssistanceState == "replaceWaypoint")
    {
      updateTextBubble("Great job! You made a new waypoint.");
    }
    else if (currentAssistanceState == "addWaypoint")
    {
      updateTextBubble("Great job! You added a waypoint.");
    }
    else if (currentAssistanceState == "deleteWaypoint")
    {
      updateTextBubble("Great job! You deleted a waypoint.");
    }
    else if (currentAssistanceState == "runProgram")
    {
      updateTextBubble("Great job! You ran a program.");
    }
    else if (currentAssistanceState == "runToWaypoint")
    {
      updateTextBubble("Great job! You moved the robot to a waypoint.");
    }

    /*const button = document.getElementById(currentAssistanceState);
    button.style.display = 'block';
    button.style.height = '';
    button.style.padding = '';
    button.style.margin = '';*/
    document.getElementById('button-container').innerHTML =
      `<button id="createProgram" onclick="handleButtonClick('createProgram')">Help me with creating a program</button><br>
       <button id="editProgram" onclick="handleButtonClick('editProgram')">Help me with editing a program</button>
       <button id="runProgram" onclick="handleButtonClick('runProgram')">Help me with running a program</button><br>`;
  }
}

function clearAllGuides()
{
  document.getElementById('col-2').classList.remove('highlight-border');
  document.getElementById('draggable-list').classList.remove('highlight-border');
  document.getElementById('run-program').classList.remove('highlight-border');
  removeHighlightFromButton("add-waypoint");
  removeHighlightFromButton("run-program");
  removeHighlightLastWaypointHandle();
}

function adjustTextBubblePosition() {
    document.getElementById("text-bubble").style.marginTop = '10px';
  }

// Arrays of instructions
const programmingInstructions = [
  "Move to the robot and guide it by hand to the position you want it to go to first.",
  "Move the robot's gripper using the button on the robot if you need to.",
  "Click Add Waypoint to save the point into your program.",
  // Add more instructions as needed
];

const replacingInstructions = [
  "Right click on the waypoint you want to replace. Move the robot to the new position you want, set the gripper the way you want, and then click on the Replace Waypoint option."
  // Add more instructions as needed
];

const deletingInstructions = [
  "Right click on the waypoint you want to delete. Then, click the Delete Waypoint option."
  // Add more instructions as needed
];

const runningInstructions = [
  "Take the emergency button in your hand. You can press it if the robot is about to do something dangerous.",
  "Move a safe way away from the robot.",
  "Click on run program."
  // Add more instructions as needed
];

const runningToInstructions = [
  "Take the emergency button in your hand. You can press it if the robot is about to do something dangerous.",
  "Move a safe way away from the robot.",
  "Right click on the waypoint you want the robot to move to. Then, click the Run Program to Here option."
];

// Arrays of instructions
const addingInstructions = [
  "Move to the robot and guide it by hand to the position you want it to go to first.",
  "Move the robot's gripper using the button on the robot if you need to.",
  "Click Add Waypoint to save the point into your program.",
  "Drag the waypoint you added to its correct spot in the sequence if needed."
  // Add more instructions as needed
];


function handleButtonClick(assistanceType) 
{
  currentInstructionIndex = 0;
  if (assistanceType == 'editProgram') 
  {
    var textBubbleContent = document.getElementById('text-bubble-content');
    textBubbleContent.textContent = "Hello, I'm here to help you!";
    document.getElementById("instruction-container").style.display = "none";
    // Replace the button container with the new buttons
    document.getElementById('button-container').innerHTML =
      `<button id="addWaypoint" onclick="handleButtonClick('addWaypoint')">Help me with adding a waypoint</button>
       <button id="replaceWaypoint" onclick="handleButtonClick('replaceWaypoint')">Help me with replacing a waypoint</button>
       <button id="removeWaypoint" onclick="handleButtonClick('removeWaypoint')">Help me with removing a waypoint</button>
       <button id="runToWaypoint" onclick="handleButtonClick('runToWaypoint')">Help me with moving the robot</button>
       <div style="height: 10px;"></div> <!-- Add a spacer with desired height -->
       <button id="createProgram" onclick="handleButtonClick('createProgram')">Help me with creating a program</button>
       <button id="runProgram" onclick="handleButtonClick('runProgram')">Help me with running a program</button>`;
  }
  else if (assistanceType != "none")
  {
    // Hide the clicked button
    if (currentAssistanceState != "none" && currentAssistanceState != "editProgram" && currentAssistanceState != "replaceWaypoint" && currentAssistanceState != "removeWaypoint" && currentAssistanceState != "runToWaypoint")
    {
      document.getElementById('button-container').innerHTML =
        `<button id="createProgram" onclick="handleButtonClick('createProgram')">Help me with creating a program</button>
         <button id="editProgram" onclick="handleButtonClick('editProgram')">Help me with editing a program</button>
         <button id="runProgram" onclick="handleButtonClick('runProgram')">Help me with running a program</button>`;

      const button1 = document.getElementById(currentAssistanceState);
      if (button1)
      {
        button1.style.display = 'block';
        button1.style.height = '';
        button1.style.padding = '';
        button1.style.margin = '';
      }
    }

    const button2 = document.getElementById(assistanceType);
    if (button2)
    {
      button2.style.display = 'none';
      button2.style.height = '0';
      button2.style.padding = '0';
      button2.style.margin = '0';
    }

    document.getElementById("instruction-container").style.display = "block";
  }

  currentAssistanceState = assistanceType;

  stopSpeech();
  clearAllGuides();
  // Hide the button container
  //document.getElementById("button-container").style.display = "none";


  // Set the initial instruction
  //document.getElementById("instruction-text").textContent = programmingInstructions[0];

  switch (currentAssistanceState) 
  {
      case "createProgram":
        updateTextBubble(programmingInstructions[0]);
        document.getElementById('col-2').classList.add('highlight-border');
        break;
      case "addWaypoint":
        updateTextBubble(addingInstructions[0]);
        document.getElementById('col-2').classList.add('highlight-border');
        break;
      case "replaceWaypoint":
        updateTextBubble(replacingInstructions[0]);
        document.getElementById('draggable-list').classList.add('highlight-border');
        currentInstructionIndex = -2;
        break;
      case "removeWaypoint":
        updateTextBubble(deletingInstructions[0]);
        currentInstructionIndex = -2;
        document.getElementById('draggable-list').classList.add('highlight-border');
        break;
      case "runToWaypoint":
        updateTextBubble(runningToInstructions[0]);
        break;
      case "runProgram":
        updateTextBubble(runningInstructions[0]);
        break;
      default:
        break;
  }

  adjustTextBubblePosition(); 
}

function stopSpeech() {
  // Check browser compatibility
  const speechSynthesis = window.speechSynthesis || window.webkitSpeechSynthesis;

  // Stop the speech synthesis
  if (speechSynthesis && speechSynthesis.speaking) {
    speechSynthesis.cancel();
  }
}

  function nextInstruction() {
    stopSpeech();

    if (currentInstructionIndex >= 0)
    {
      if (currentAssistanceState == "createProgram")
      {
        currentInstructionIndex = (currentInstructionIndex+ 1) % 3;
        updateTextBubble(programmingInstructions[currentInstructionIndex]);

        switch (currentInstructionIndex) {
          case 0:
            removeHighlightFromButton("add-waypoint");
            //document.getElementById('col-2').classList.add('highlight-border');
            //break;
          case 1:
            document.getElementById('col-2').classList.add('highlight-border');
            break;
          case 2:
            document.getElementById('col-2').classList.remove('highlight-border');
            addHighlightToButton("add-waypoint");
            break;
          default:
            break;
          }
      }
      else if (currentAssistanceState == "runProgram")
      {
        currentInstructionIndex++;
        updateTextBubble(runningInstructions[currentInstructionIndex]);
        if (currentInstructionIndex == 2)
        {
          addHighlightToButton("run-program");
          currentInstructionIndex = -2;
          return;
        }
      }
      else if (currentAssistanceState == "runToWaypoint")
      {
        currentInstructionIndex++;
        updateTextBubble(runningToInstructions[currentInstructionIndex]);
        if (currentInstructionIndex == 2)
        {
          document.getElementById('draggable-list').classList.add('highlight-border');
          currentInstructionIndex = -2;
          return;
        }
      }
      else if (currentAssistanceState == "addWaypoint")
      {
        currentInstructionIndex++;
        switch (currentInstructionIndex) {
          case 0:
            document.getElementById('col-2').classList.add('highlight-border');
          case 1:
            removeHighlightFromButton("add-waypoint");
            //document.getElementById('col-2').classList.add('highlight-border');
            break;
          case 2:
            document.getElementById('col-2').classList.remove('highlight-border');
            addHighlightToButton("add-waypoint");
            break;
          case 3:
            removeHighlightFromButton("add-waypoint");
            highlightLastWaypointHandle();
          default:
            break;
          }
        updateTextBubble(addingInstructions[currentInstructionIndex]);
        if (currentInstructionIndex == 3)
        {
          currentInstructionIndex = -2;
          return;
        }
      }
    }
    else
    {
      /*const button = document.getElementById(currentAssistanceState);
      button.style.display = 'block';
      button.style.height = '';
      button.style.padding = '';
      button.style.margin = '';
      currentAssistanceState = "none";*/
      stopSpeech();
      clearAllGuides();
      var textBubbleContent = document.getElementById('text-bubble-content');
      textBubbleContent.textContent = "Hello, I'm here to help you!";
      document.getElementById("instruction-container").style.display = "none";
      document.getElementById('button-container').innerHTML =
      `<button id="createProgram" onclick="handleButtonClick('createProgram')">Help me with creating a program</button>
       <button id="editProgram" onclick="handleButtonClick('editProgram')">Help me with editing a program</button>
       <button id="runProgram" onclick="handleButtonClick('runProgram')">Help me with running a program</button>`;
      //initializeAssistant();
    }
  }

    /*if (currentInstructionIndex < instructions.length) {
      // Set the next instruction
      //document.getElementById("instruction-text").textContent = programmingInstructions[currentInstructionIndex];
      updateTextBubble(programmingInstructions[currentInstructionIndex]);
    } else {
      // All instructions completed
      // Show the button container again
      document.getElementById("button-container").style.display = "block";

      // Hide the instruction container
      document.getElementById("instruction-container").style.display = "none";
    }*/

function highlightLastWaypointHandle() {
  // Get the last drag handle and add the highlight class
  var lastWaypoint = document.getElementById('draggable-list').lastElementChild;
  if (lastWaypoint)
  {
    var lastDragHandle = lastWaypoint.querySelector('.drag-handle');
    lastDragHandle.classList.add('highlight-handle');
  }
}

function removeHighlightLastWaypointHandle() {
  // Get the last drag handle and add the highlight class
  var lastWaypoint = document.getElementById('draggable-list').lastElementChild;
  if (lastWaypoint)
  {
    var lastDragHandle = lastWaypoint.querySelector('.drag-handle');
    lastDragHandle.classList.remove('highlight-handle');
  }
}
  

function addHighlightToButton(buttonId) {
  const button = document.getElementById(buttonId);
  button.classList.add("highlight");
}


function removeHighlightFromButton(buttonId) {
  const button = document.getElementById(buttonId);
  button.classList.remove("highlight");
}


document.addEventListener('DOMContentLoaded', function() 
{
  var draggableList = document.getElementById('draggable-list');
  var addWaypointBtn = document.getElementById('add-waypoint');
  var runProgramBtn = document.getElementById('run-program');
  var clearProgramBtn = document.getElementById('clear-program');
  //var refreshBtn = document.getElementById('refresh');
  var contextMenu = document.getElementById('context-menu');
  var deleteWaypointOption = document.getElementById('delete-waypoint');
  var modifyWaypointOption = document.getElementById('modify-waypoint');
  var runWaypointOption = document.getElementById('exec-waypoint');

  var nextIndex = 1; // Initial index for waypoints

  addWaypointBtn.addEventListener('click', function() 
  {
    if (currentAssistanceState == "createProgram" && currentInstructionIndex == 1)
    {
      currentInstructionIndex = 2;
    }
    else if (currentAssistanceState == "addWaypoint" && currentInstructionIndex == 1)
    {
      currentInstructionIndex = 2;
    }
    createWaypoint();
  });

  runProgramBtn.addEventListener('click', function() 
  {
    runProgram();
  });


  function clearDraggableList() {
    var draggableList = document.getElementById('draggable-list');
      while (draggableList.firstChild) {
        draggableList.removeChild(draggableList.firstChild);
    }
  }

  clearProgramBtn.addEventListener('click', function() 
  {
    var clear_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 9,
        optional_index_1: 0,
        optional_index_2: 0
      }
    });
    clear_goal.on('feedback', function(feedback) {
      console.log('Feedback: ' + feedback.status);
    });
    clear_goal.on('result', function(result) {
      console.log('Final Result: ' + result.conclusion);
    });

    // Send the goal to the action server.
    clear_goal.send();

    clearDraggableList();
  });

  /*refreshBtn.addEventListener('click', function() 
  {
    updateIndices();
  });*/

  // Create a new waypoint item in the draggable list
  function createWaypoint() 
  {
    var add_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 2,
        optional_index_1: 0,
        optional_index_2: 0
      }
    });
    add_goal.on('feedback', function(feedback) {
      console.log('Feedback: ' + feedback.status);
    });
    add_goal.on('result', function(result) {
      console.log('Final Result: ' + result.conclusion);
    });

    // Send the goal to the action server.
    add_goal.send();

    var waypointItem = document.createElement('li');
    waypointItem.innerHTML = 'Waypoint ' + nextIndex;
      
    var dragHandle = document.createElement('span');
    dragHandle.className = 'drag-handle';
    //dragHandle.innerHTML = '<i class="uil uil-draggabledots"></i>';
    waypointItem.appendChild(dragHandle);
      
    waypointItem.draggable = true;
    addEventsDragAndDrop(waypointItem);
    waypointItem.addEventListener('contextmenu', handleContextMenu);
    waypointItem.addEventListener('click', handleItemClick);
      
    draggableList.appendChild(waypointItem);
    nextIndex++;  
  }

  function runProgram()
  {
    if (currentAssistanceState == "createProgram" || currentAssistanceState == "runProgram")
    {
      initializeAssistant();
    }

    var run_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 6,
        optional_index_1: 0,
        optional_index_2: 0
      }
    });
    run_goal.on('feedback', function(feedback) {
      console.log('Feedback: ' + feedback.status);
    });
    run_goal.on('result', function(result) {
      console.log('Final Result: ' + result.conclusion);
    });

    // Send the goal to the action server.
    run_goal.send();
  }

  function handleItemClick(event) {
    var item = event.target.closest('li');
    const draggableList = document.getElementById('draggable-list');
    const index = Array.from(draggableList.children).indexOf(item);

    if (item) 
    {
      var isHighlighted = item.classList.toggle('highlighted');

      // Check if other items are highlighted and remove their highlight
      Array.from(draggableList.children).forEach(function (child) 
      {
        if (child !== item && child.classList.contains('highlighted')) 
        {
          var childIndex = Array.from(draggableList.children).indexOf(child);
          child.classList.remove('highlighted');
          var deselectMessage = new ROSLIB.Message({
            num : ((childIndex + 1) * -1),
            swap_id1 : 0,
            swap_id2: 0
          });

          markerTopic.publish(deselectMessage);
          
          // Use the childIndex as needed
          //console.log('Index of highlighted item:', childIndex);
          /*var deselect_goal = new ROSLIB.Goal({
            actionClient: interfaceClient,
            goalMessage: {
              request: 7,
              optional_index_1: ((childIndex + 1) * -1),
              optional_index_2: 0
            }
          });
          deselect_goal.on('feedback', function (feedback) {
            console.log('Feedback: ' + feedback.status);
          });
          deselect_goal.on('result', function (result) {
            console.log('Final Result: ' + result.conclusion);
          });

          // Send the goal to the action server.
          deselect_goal.send();*/
        }
      });
    

      if (isHighlighted) {
        var select_goal = new ROSLIB.Goal({
          actionClient: interfaceClient,
          goalMessage: {
            request: 7,
            optional_index_1: ((index + 1) * -1),
            optional_index_2: 1
          }
        });
      } else {
        var select_goal = new ROSLIB.Goal({
          actionClient: interfaceClient,
          goalMessage: {
            request: 7,
            optional_index_1: ((index + 1) * -1),
            optional_index_2: 0
          }
        });
      }

      select_goal.on('feedback', function (feedback) {
        console.log('Feedback: ' + feedback.status);
      });
      select_goal.on('result', function (result) {
        console.log('Final Result: ' + result.conclusion);
      });

      // Send the goal to the action server.
      select_goal.send();
    }
  }




function addEventsDragAndDrop(el) {
  el.addEventListener('dragstart', dragStart, false);
  el.addEventListener('dragenter', dragEnter, false);
  el.addEventListener('dragover', dragOver, false);
  el.addEventListener('dragleave', dragLeave, false);
  el.addEventListener('drop', dragDrop, false);
  el.addEventListener('dragend', dragEnd, false);
}

// Global variables to store the initial and new index
var initialIndex;
var newIndex;

function dragStart(e) {
  this.style.opacity = '0.4';
  dragSrcEl = this;
  initialIndex = Array.from(draggableList.children).indexOf(dragSrcEl);
  e.dataTransfer.effectAllowed = 'move';
  e.dataTransfer.setData('text/html', this.innerHTML);
};

function dragEnter(e) {
  this.classList.add('over');
}

function dragLeave(e) {
  e.stopPropagation();
  this.classList.remove('over');
}

function dragOver(e) {
  e.preventDefault();
  e.dataTransfer.dropEffect = 'move';
  return false;
}

function dragDrop(e) {
  if (dragSrcEl != this) {
    dragSrcEl.innerHTML = this.innerHTML;
    this.innerHTML = e.dataTransfer.getData('text/html');
    newIndex = Array.from(draggableList.children).indexOf(this);
    
    // Call a function or perform actions with initialIndex and newIndex
    // For example:
    //console.log('Item dragged from index', initialIndex, 'to index', newIndex);

    var move_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 4,
        optional_index_1: initialIndex,
        optional_index_2: newIndex
      }
    });

    move_goal.on('feedback', function(feedback) {
      console.log('Feedback: ' + feedback.status);
    });
    move_goal.on('result', function(result) {
      console.log('Final Result: ' + result.conclusion);
    });

    // Send the goal to the action server.
    move_goal.send();
  }
  return false;
}

function dragEnd(e) {
  var listItens = document.querySelectorAll('.draggable');
  [].forEach.call(listItens, function(item) {
    item.classList.remove('over');
  });
  this.style.opacity = '1';
  updateIndices();

  if (currentAssistanceState == "addWaypoint")
  {
    initializeAssistant();
  }
}

  // Update the indices of the waypoints
  function updateIndices() 
  {
    var items = draggableList.getElementsByTagName('li');
    for (var i = 0; i < items.length; i++) {
      items[i].innerHTML = 'Waypoint ' + (i + 1);
      var dragHandle = document.createElement('span');
      dragHandle.className = 'drag-handle';
      //dragHandle.innerHTML = '<i class="uil uil-draggabledots"></i>';
      items[i].appendChild(dragHandle);
    }
    nextIndex = items.length + 1;
  }

    // Context menu event handler
  function handleContextMenu(event) {
    // Delete previous click event handlers
    deleteWaypointOption.removeEventListener('click', handleDeleteWaypoint);
    modifyWaypointOption.removeEventListener('click', handleModifyWaypoint);
    runWaypointOption.removeEventListener('click', handleRunWaypoint);

    event.preventDefault();
    var targetItem = event.target.closest('li');
    
    // Show the context menu at the click position
    contextMenu.style.display = 'block';
    contextMenu.style.left = event.pageX + 'px';
    contextMenu.style.top = event.pageY + 'px';
    
    // Delete Waypoint option click event handler
    function handleDeleteWaypoint() {
      if (currentAssistanceState == "deleteWaypoint")
      {
        initializeAssistant();
      }

      if (targetItem) {
        const listItem = event.target.closest('li');
        const draggableList = document.getElementById('draggable-list');
        const index = Array.from(draggableList.children).indexOf(listItem);
        targetItem.remove();
        if (index >= 0)
        {
          /*markerClient.removeMarker("sphere" + (index + 1));
          markerClient.removeMarker("text" + (index + 1));*/
          var del_goal = new ROSLIB.Goal({
            actionClient : interfaceClient,
            goalMessage : {
              request : 3,
              optional_index_1: index,
              optional_index_2: 0
            }
          });
          del_goal.on('feedback', function(feedback) {
            console.log('Feedback: ' + feedback.status);
          });
          del_goal.on('result', function(result) {
            console.log('Final Result: ' + result.conclusion);
          });

          // Send the goal to the action server.
          del_goal.send();
        }
        updateIndices();
      }
      contextMenu.style.display = 'none';
    }
    
    // Modify Waypoint option click event handler
    function handleModifyWaypoint() {
      if (currentAssistanceState == "replaceWaypoint")
      {
        initializeAssistant();
      }

      if (targetItem) {
        const listItem = event.target.closest('li');
        const draggableList = document.getElementById('draggable-list');
        const index = Array.from(draggableList.children).indexOf(listItem);
        if (index >= 0)
        {
          var rep_goal = new ROSLIB.Goal({
            actionClient : interfaceClient,
            goalMessage : {
              request : 5,
              optional_index_1: index,
              optional_index_2: 0
            }
          });
          rep_goal.on('feedback', function(feedback) {
            console.log('Feedback: ' + feedback.status);
          });
          rep_goal.on('result', function(result) {
            console.log('Final Result: ' + result.conclusion);
          });

          // Send the goal to the action server.
          rep_goal.send();
        }
      }
      contextMenu.style.display = 'none';
    }
    

    
    // Execute Waypoint option click event handler
    function handleRunWaypoint() {
      if (currentAssistanceState == "runToWaypoint")
      {
        initializeAssistant();
      }

      if (targetItem) {
        const listItem = event.target.closest('li');
        const draggableList = document.getElementById('draggable-list');
        const index = Array.from(draggableList.children).indexOf(listItem);
        if (index >= 0)
        {
          console.log('Sending run to goal');
          var run_to_goal = new ROSLIB.Goal({
            actionClient : interfaceClient,
            goalMessage : {
              request : 8,
              optional_index_1: index,
              optional_index_2: 0
            }
          });
          /*run_to_goal.on('feedback', function(feedback) {
            console.log('Feedback: ' + feedback.status);
          });
          run_to_goal.on('result', function(result) {
            console.log('Final Result: ' + result.conclusion);
          });*/

          // Send the goal to the action server.
          run_to_goal.send();
          runWaypointOption.removeEventListener('click', handleRunWaypoint);
        }
      }
      contextMenu.style.display = 'none';
      }

      // Delete previous click event handlers
  deleteWaypointOption.removeEventListener('click', handleDeleteWaypoint);
  modifyWaypointOption.removeEventListener('click', handleModifyWaypoint);
  runWaypointOption.removeEventListener('click', handleRunWaypoint);

  // Add new click event handlers
  deleteWaypointOption.addEventListener('click', handleDeleteWaypoint);
  modifyWaypointOption.addEventListener('click', handleModifyWaypoint);
  runWaypointOption.addEventListener('click', handleRunWaypoint);

    // Hide the context menu when clicking outside of it
    document.addEventListener('click', function() {
      contextMenu.style.display = 'none';
    }, { once: true });
  }
});

const beforeUnloadListener = (event) => {
    setTimeout(() => alert('hi!'));
    event.preventDefault();
    return event.returnValue = "Are you sure you want to exit?";
};

window.onbeforeunload = function (event) {
  // Deactivate teaching mode on the Kinova arm
  var stop_teach_goal = new ROSLIB.Goal({
    actionClient: interfaceClient,
    goalMessage: {
      request: 1,
      optional_index_1: 0,
      optional_index_2: 0
    }
  });
  stop_teach_goal.on('feedback', function (feedback) {
    console.log('Feedback: ' + feedback.status);
  });
  stop_teach_goal.on('result', function (result) {
    console.log('Final Result: ' + result.conclusion);
  });

  // Send the goal to the action server.
  stop_teach_goal.send();

  var clear_goal = new ROSLIB.Goal({
      actionClient : interfaceClient,
      goalMessage : {
        request : 9,
        optional_index_1: 0,
        optional_index_2: 0
      }
    });
  clear_goal.on('feedback', function(feedback) {
    console.log('Feedback: ' + feedback.status);
  });
  clear_goal.on('result', function(result) {
    console.log('Final Result: ' + result.conclusion);
  });

  // Send the goal to the action server.
  clear_goal.send();

  var clearMarkersMessage = new ROSLIB.Message({
            num : 0,
            swap_id1 : 0,
            swap_id2: 0
          });

  markerTopic.publish(clearMarkersMessage);

  // Show a popup message
  //var confirmationMessage = 'Remember to move the robot to the home position!';
  //event.returnValue = confirmationMessage; // Required for some browsers

  // Display the popup
  /*var popup = window.open('', '_blank', 'width=400,height=200');
  popup.document.write('<html><head><title>Reminder</title></head><body>');
  popup.document.write('<h1>Important Reminder</h1>');
  popup.document.write('<p>' + confirmationMessage + '</p>');
  popup.document.write('</body></html>');*/
};


function speak(text) {
  const utterance = new SpeechSynthesisUtterance(text);

  // Get the available voices
  const voices = speechSynthesis.getVoices();
  utterance.voice = voices[102];
  //utterance.voiceURI = 'native';
  utterance.volume = 1; // 0 to 1
  utterance.rate = 0.6; // 0.1 to 10
  utterance.pitch = 0.25; //0 to 2
  //utterance.lang = 'en-US';

  // Speak the text
  speechSynthesis.speak(utterance);
}


// Function to update the text bubble content
function updateTextBubble(content) {
  var textBubbleContent = document.getElementById('text-bubble-content');
  textBubbleContent.textContent = content;

  speak(content);
  // Call the function to log the voices array
  //logAvailableVoices();
}

// Log the available voices to the console
function logAvailableVoices() {
  if ('speechSynthesis' in window) {
    const synth = window.speechSynthesis;
    const voices = synth.getVoices();
    console.log(voices);
  } else {
    console.log('Speech synthesis is not supported.');
  }
}

function showTextBubble() {
  var textBubble = document.getElementById('textBubble');
  textBubble.style.display = 'block';
  textBubble.style.backgroundColor = 'yellow'
  //textBubble.style.color = 'black';
  textBubble.textContent = 'Remember to set any objects back to their original positions before running the program!';
}

function hideTextBubble() {
  var textBubble = document.getElementById('textBubble');
  textBubble.style.display = 'none';
}

function showTextBubble2() {
  var textBubble = document.getElementById('textBubble');
  textBubble.style.display = 'block';
  textBubble.style.backgroundColor = '#f08080';
  //textBubble.style.color = 'black';
  textBubble.textContent = 'Warning: You are about to erase your program!';
}

function hideTextBubble2() {
  var textBubble = document.getElementById('textBubble');
  textBubble.style.display = 'none';
}


