#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <prog_support_backend/InterfaceRequestAction.h>
#include <marker_package/Waypoint.h>

#include <DoublyLinkedList.h>

// Include any additional files requored for the robot you are using

typedef actionlib::SimpleActionServer<prog_support_backend::InterfaceRequestAction> Server;
enum  Requests {TEACH=0, STOP_TEACH=1, ADD_WAYPOINT=2, DEL_WAYPOINT=3, MOVE_WAYPOINT=4, REP_WAYPOINT=5, EXEC=6, SELECT=7, EXEC_UNTIL=8, CLEAR_PROG=9, GO_TO=10, ADD_LOOP=11, DEL_LOOP=12};  // Types of requests from action client

// Robot-specific values
const double GRIPPER_OPEN_STATE = 0.00873365; // Replace this value with the value representing your robot's open gripper state
const int NUM_JOINTS = 7; // Replace this value with the number of joints your robot has

// Replace double with the data type for your robot's joint values
DoublyLinkedList<std::vector<double>> userProgram;

ros::Publisher waypoint_pub;
ros::Publisher exec_pub;


// activateTeachMode
// This function should activate your robot's kinesthetic teaching functionality.
// Input: ROS node handle
// Output: Boolean value indicating whether the robot's kinesthetic teaching functionality was successfully activated
bool activateTeachMode(ros::NodeHandle &n)
{
  /* Insert code here */
  return false;
}


// deactivateTeachMode
// This function should deactivate your robot's kinesthetic teaching functionality.
// Input: ROS node handle
// Output: Boolean value indicating whether the robot's kinesthetic teaching functionality was successfully deactivated
bool deactivateTeachMode(ros::NodeHandle &n)
{
  /* Insert code here */
  return false;
}

// getCurrentJointStates
// This function will populate a vector that you provide with the robot's current joint states and its gripper state
// Input: ROS node handle, vector to hold robot's joint states and gripper state (the first n values will contain the robot's joint states, and the (n + 1)th value will contain the robot's gripper state); replace double with the data type for your robot's joint values)
// Output: Boolean value indicating whether the vector you provided was successfully populated with the robot's current joint states and its gripper state
bool getCurrentJointStates(ros::NodeHandle &n, std::vector<double> &waypoint)
{
  /* Insert code here */
  return true;
}

// addWaypoint
// This function adds a new waypoint into the user's program and publishes a marker representing the waypoint for the frontend interface to display
// Input: ROS node handle
// Output: None
void addWaypoint(ros::NodeHandle &n)
{
  std::vector<double> waypoint;
  getWaypoint(n, waypoint);
  userProgram.insert(waypoint);
  
  marker_package::Waypoint msg;
  msg.num = userProgram.size();
  waypoint_pub.publish(msg);
}

// replaceWaypoint
// This function replaces the values stored at a waypoint at a specified index with the values corresponding to the robot's current joint states and gripper state.
// Input: ROS node handle, integer specifying the index of the waypoint being replaced
// Output: Boolean value indicating whether the values at the specified waypoint were replaced successfully
bool replaceWaypoint(ros::NodeHandle &n, int index)
{
  std::vector<double> waypoint;
  getWaypoint(n, waypoint);
  marker_package::Waypoint msg;
  msg.num = index + 1;
  waypoint_pub.publish(msg);
  return userProgram.replace(index, waypoint);
}

// moveToWaypoint
// This function will move the robot to the provided set of joint states
// Input: ROS node handle, vector containing a set of robot joint states (replace double with the data type for your robot's joint values)
// Output: Boolean value indicating whether the robot successfully moved to the specified configuration
bool moveToWaypoint(ros::NodeHandle &n, std::vector<double> pt)
{
  /* Insert code here */
  return false;
}

// sendGripperCommand
// This function will move the robot's gripper to the specified value
// Input: ROS node handle, double value containing the desired state for the robot's gripper (replace double with the data type for your robot's gripper value)
// Output: Boolean value indicating whether the gripper successfully moved to the specified configuration
bool sendGripperCommand(ros::NodeHandle n, double value)
{
  /* Insert code here */
  return false;
}

// executeWaypoint
// This function will move the robot to the specified waypoint
// Input: Waypoint to be executed (represented as a vector of double values representing the robot's joint states and gripper state, respectively; replace double with the data type your robot uses), ROS node handle
// Output: Booelan value indicating whether the robot successfully moved to the specified waypoint
bool executeWaypoint(std::vector<double> waypoint, ros::NodeHandle &n)
{
  bool status = true;
  status &= clearFaults(n);
  status &= moveToWaypoint(n, waypoint);
  status &= sendGripperCommand(n, waypoint[NUM_JOINTS]);

  std::cout << "Executed the waypoint" << std::endl;
  return status;
}

/*** Action server ***/
// execute
// This function handles the requests coming from the frontend interface.
// Input: Goal pointer, ROS node handle
// Output: None
void execute(const prog_support_backend::InterfaceRequestGoalConstPtr& goal, Server* as, ros::NodeHandle &node_handle)
{
  prog_support_backend::InterfaceRequestResult result;
  bool status;

  if (goal->request == TEACH)
  {
    status = activateTeachMode(node_handle);
    if (status)
    {
      result.conclusion = "Set robot to teach mode";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to set robot to teach mode";
      as->setAborted(result);
    }    
  }
  else if (goal->request == STOP_TEACH)
  {
    status = deactivateTeachMode(node_handle);
    if (status)
    {
      result.conclusion = "Deactivated teach mode";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to deactivate teach mode";
      as->setAborted(result);
    }    
  }
  else if (goal->request == ADD_WAYPOINT)
  {
    addWaypoint(node_handle);
    result.conclusion = "Added waypoint into program";
    as->setSucceeded(result);
  }
  else if (goal->request == DEL_WAYPOINT)
  {
    status = userProgram.removeAt(goal->optional_index_1);
    if (status)
    {
      result.conclusion = "Removed waypoint " + std::to_string(goal->optional_index_1) + " from program";
      as->setSucceeded(result);
      marker_package::Waypoint msg;
      msg.num = 0;
      msg.swap_id1 = 0;
      msg.swap_id2 = -1 * (goal->optional_index_1 + 1);
      msg.extra_num = 0;
      waypoint_pub.publish(msg);
    }
    else
    {
      result.conclusion = "Failed to remove waypoint " + std::to_string(goal->optional_index_1) + " from program";
      as->setAborted(result);
    }    
  }
  else if (goal->request == MOVE_WAYPOINT)
  {
    userProgram.move(goal->optional_index_1, goal->optional_index_2);
    result.conclusion = "Moved waypoint from index " + std::to_string(goal->optional_index_1) + " to " + std::to_string(goal->optional_index_2);
    as->setSucceeded(result);
    marker_package::Waypoint msg;
    msg.num = 0;
    msg.swap_id1 = goal->optional_index_1 + 1;
    msg.swap_id2 = goal->optional_index_2 + 1;
    msg.extra_num = 0;
    waypoint_pub.publish(msg);
  }
  else if (goal->request == REP_WAYPOINT)
  {
    bool status = replaceWaypoint(node_handle, goal->optional_index_1);
    if (status)
    {
      result.conclusion = "Replaced waypoint at index " + std::to_string(goal->optional_index_1) + " with current robot position";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to replace waypoint at index " + std::to_string(goal->optional_index_1) + " with current robot position";
      as->setAborted(result);
    }  
  }
  else if (goal->request == EXEC)
  {
    int i = 0;
    sendGripperCommand(node_handle, GRIPPER_OPEN_STATE);

    if (userProgram.getHead() != nullptr)
    { 
      moveToWaypoint(node_handle, userProgram.getHeadValue()); // Account for not executing first waypoint sometimes; remove this line if not necessary for your robot

      Node<std::vector<double>>* currentNode = userProgram.getHead();
      bool status;

      while (currentNode != nullptr) 
      {
          if (currentNode->isInLoop) 
          {
              int loopIterations = currentNode->loop->loopCount;
              int loopStartIndex = currentNode->loop->startIndex;
              int loopEndIndex = currentNode->loop->endIndex;

              // Execute the loop content.
              Node<std::vector<double>>* loopStartNode = userProgram.getNodeAtIndex(loopStartIndex);
              Node<std::vector<double>>* loopEndNode = userProgram.getNodeAtIndex(loopEndIndex);

              if (loopStartNode == nullptr || loopEndNode == nullptr) 
              {
                  // Invalid loop indices. Abort loop execution.
                  break;
              }

              for (int i = 0; i < loopIterations; ++i) 
              {
                currentNode = loopStartNode;
                do
                {
                    status = executeWaypoint(currentNode->value, node_handle);
                    currentNode = currentNode->next;
                } while (currentNode != loopEndNode->next);
              }
          } 
          else 
          {
              // Execute the current waypoint.
              status = executeWaypoint(currentNode->value, node_handle);
              currentNode = currentNode->next;
          }
          i++;
      }

      if (status)
      {
        result.conclusion = "Executed user program";
        as->setSucceeded(result);
      }
      else
      {
        result.conclusion = "Failed to execute user program";
        as->setAborted(result);
      }  

      status = activateTeachMode(node_handle);
      if (!status)
      {
        std::cout << "Failed to activate teach mode after program execution" << std::endl;
      }
    }
    else
    {
      result.conclusion = "Tried to execute empty user program";
      as->setAborted(result);
    }
  }
  else if (goal->request == EXEC_UNTIL)
  {
    sendGripperCommand(node_handle, GRIPPER_OPEN_STATE);
    moveToWaypoint(node_handle, userProgram.getHeadValue()); // Account for not executing first waypoint sometimes
    std::cout << "Moved to initial position" << std::endl;

    bool status = userProgram.iterate_until(executeWaypoint, goal->optional_index_1, node_handle);
    if (status)
    {
      result.conclusion = "Executed user program until specified point";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to execute user program until specified point";
      as->setSucceeded(result);
    }  

    status = activateTeachMode(node_handle);
    if (!status)
    {
      std::cout << "Failed to activate teach mode after program execution" << std::endl;
    }
  }
  else if (goal->request == SELECT)
  {
    marker_package::Waypoint msg;
    msg.num = goal->optional_index_1;
    msg.swap_id1 = goal->optional_index_2;
    msg.extra_num = 0;
    waypoint_pub.publish(msg);
    result.conclusion = "Selected waypoint";
    as->setSucceeded(result);
  }
  else if (goal->request == CLEAR_PROG)
  {
    userProgram.eraseAllNodes();
    result.conclusion = "Starting new program";
    as->setSucceeded(result);
  }
  else if (goal->request == GO_TO)
  {
    int waypointNum = int(goal->optional_index_1);
    clearFaults(node_handle);

    if (waypointNum== 0)
    {
      sendGripperCommand(node_handle, GRIPPER_OPEN_STATE);
      moveToWaypoint(node_handle, userProgram.getHeadValue()); // Account for not executing first waypoint sometimes
    }

    moveToWaypoint(node_handle, userProgram.getValueAtIndex(waypointNum));
    result.conclusion = "Robot is moving to waypoint " + waypointNum;
    as->setSucceeded(result);
  }
  else if (goal->request == ADD_LOOP)
  {
    userProgram.insertLoop(goal->optional_index_2, goal->optional_index_3, goal->optional_index_1);
    result.conclusion = "Added a loop of iterations " + std::to_string(goal->optional_index_1) + " starting at " + std::to_string(goal->optional_index_2) + " and ending at " + std::to_string(goal->optional_index_3);
    as->setSucceeded(result);
  }
  else if (goal->request == DEL_LOOP)
  {
    userProgram.deleteLoop(goal->optional_index_1, goal->optional_index_2);
    result.conclusion = "Deleted loop starting at " + std::to_string(goal->optional_index_1) + " and ending at " + std::to_string(goal->optional_index_2);
    as->setSucceeded(result);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "backend");
  ros::NodeHandle node_handle;

  /* Insert any necessary robot setup/initialization code here */

  ros::AsyncSpinner spinner(2); 
  spinner.start();

   // Subscribe to the Action Topic
  ros::Subscriber sub = node_handle.subscribe("/action_topic", 1000, notification_callback);
  waypoint_pub = node_handle.advertise<marker_package::Waypoint>("/waypoint", 1000);

  Server server(node_handle, "interface_controller", boost::bind(&execute, _1, &server, boost::ref(node_handle)), false);
  server.start();

  ros::waitForShutdown();
  
  return 0;
}
