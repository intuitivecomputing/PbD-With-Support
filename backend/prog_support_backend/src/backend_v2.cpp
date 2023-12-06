#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <prog_support_backend/InterfaceRequestAction.h>
#include <marker_package/Waypoint.h>

#include <DoublyLinkedList_v2.h>

typedef actionlib::SimpleActionServer<prog_support_backend::InterfaceRequestAction> Server;
enum  Requests {TEACH=0, STOP_TEACH=1, ADD_WAYPOINT=2, DEL_WAYPOINT=3, MOVE_WAYPOINT=4, REP_WAYPOINT=5, EXEC=6, SELECT=7, EXEC_UNTIL=8, CLEAR_PROG=9, GO_TO=10, ADD_LOOP=11, DEL_LOOP=12};  // Types of requests from action client

#include <kortex_driver/SetAdmittance.h>
#include <kortex_driver/GetMeasuredJointAngles.h>
#include <kortex_driver/JointAngle.h>
#include <kortex_driver/JointAngles.h>
#include <kortex_driver/GetMeasuredGripperMovement.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <kortex_driver/Waypoint.h>
#include <kortex_driver/WaypointList.h>
#include <kortex_driver/AngularWaypoint.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/JointSpeedSoftLimits.h>
#include <kortex_driver/JointAccelerationSoftLimits.h>
#include <kortex_driver/SetJointSpeedSoftLimits.h>
#include <kortex_driver/SetJointAccelerationSoftLimits.h>
#include <kortex_driver/Base_ClearFaults.h>

enum AdmittanceModes {JOINT=2, DISABLED=4};
enum ControlModes {ANGULAR_TRAJECTORY=4};
const double GRIPPER_MAX_ABSOLUTE_POS = 0.8;
const double GRIPPER_MIN_ABSOLUTE_POS = 0.0;
const int NUM_JOINTS = 7;

//#include <moveit/move_group_interface/move_group_interface.h>
//
// Kortex References
// Services source code: https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/src/generated/robot/base_services.cpp
// Gripper enum: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/enums/Base/GripperMode.md
/*roslaunch kortex_driver icl_kortex_driver.launch
  roslaunch prog_support_backend web_communication.launch
  
*/

DoublyLinkedList<std::vector<double>> userProgram;

ros::Publisher waypoint_pub;
ros::Publisher exec_pub;

std::atomic<int> last_action_notification_event{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification");
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    ros::spinOnce();
  }
  return true;
}

bool activateTeachMode(ros::NodeHandle &n)
{
  ros::ServiceClient service_client_set_admittance = n.serviceClient<kortex_driver::SetAdmittance>("/base/set_admittance");
  kortex_driver::SetAdmittance service_set_admittance;
  service_set_admittance.request.input.admittance_mode = JOINT;
  // Send the command
  if (service_client_set_admittance.call(service_set_admittance))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool deactivateTeachMode(ros::NodeHandle &n)
{
  ros::ServiceClient service_client_set_admittance = n.serviceClient<kortex_driver::SetAdmittance>("/base/set_admittance");
  kortex_driver::SetAdmittance service_set_admittance;
  service_set_admittance.request.input.admittance_mode = DISABLED;
  // Send the command
  if (service_client_set_admittance.call(service_set_admittance))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool clearFaults(ros::NodeHandle &n)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults))
  {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit

  //std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ros::Duration(0.5).sleep(); 
  return true;
}

bool getWaypoint(ros::NodeHandle &n, std::vector<double> &waypoint)
{
  ros::ServiceClient service_client_get_measured_joint_angles = n.serviceClient<kortex_driver::GetMeasuredJointAngles>("/base/get_measured_joint_angles");
  kortex_driver::GetMeasuredJointAngles service_get_measured_joint_angles;
  if (!service_client_get_measured_joint_angles.call(service_get_measured_joint_angles))
  {
    std::string error_string = "Failed to call GetMeasuredJointAngles";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  kortex_driver::JointAngles currentJointValues = service_get_measured_joint_angles.response.output;
  for (unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    waypoint.push_back(currentJointValues.joint_angles[i].value);
  }

  ros::ServiceClient service_client_get_measured_gripper_movement = n.serviceClient<kortex_driver::GetMeasuredGripperMovement>("/base/get_measured_gripper_movement");
  kortex_driver::GetMeasuredGripperMovement service_get_measured_gripper_movement;
  service_get_measured_gripper_movement.request.input.mode = 3; // 3 is position control
  if (!service_client_get_measured_gripper_movement.call(service_get_measured_gripper_movement))
  {
    std::string error_string = "Failed to call GetMeasuredGripperMovement";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  waypoint.push_back(service_get_measured_gripper_movement.response.output.finger[0].value);
  return true;
}

//void addWaypoint(moveit::planning_interface::MoveGroupInterface &armGroup, moveit::planning_interface::MoveGroupInterface &gripperGroup)
void addWaypoint(ros::NodeHandle &n)
{
  std::vector<double> waypoint;
  getWaypoint(n, waypoint);
  userProgram.insert(waypoint);
  
  marker_package::Waypoint msg;
  msg.num = userProgram.size();
  waypoint_pub.publish(msg);
}

//bool replaceWaypoint(moveit::planning_interface::MoveGroupInterface &armGroup, moveit::planning_interface::MoveGroupInterface &gripperGroup, int index)
bool replaceWaypoint(ros::NodeHandle &n, int index)
{
  std::vector<double> waypoint;
  /*waypoint = armGroup.getCurrentJointValues();
  waypoint.push_back(gripperGroup.getCurrentJointValues()[0]);*/
  getWaypoint(n, waypoint);
  marker_package::Waypoint msg;
  msg.num = index + 1;
  waypoint_pub.publish(msg);
  return userProgram.replace(index, waypoint);
}

// https://raw.githubusercontent.com/Kinovarobotics/ros_kortex/noetic-devel/kortex_examples/src/full_arm/example_full_arm_movement.cpp
bool moveToWaypoint(ros::NodeHandle &n, std::vector<double> pt)
{
  last_action_notification_event = 0;
  ros::ServiceClient service_client_execute_waypoints_trajectory = n.serviceClient<kortex_driver::ExecuteWaypointTrajectory>("/base/execute_waypoint_trajectory");
  kortex_driver::ExecuteWaypointTrajectory service_execute_waypoints_trajectory;

  ros::ServiceClient service_client_validate_waypoint_list = n.serviceClient<kortex_driver::ValidateWaypointList>("/base/validate_waypoint_list");
  kortex_driver::ValidateWaypointList service_validate_waypoint_list;

  kortex_driver::WaypointList trajectory;
  kortex_driver::Waypoint waypoint;
  kortex_driver::AngularWaypoint angularWaypoint;

  for (unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    angularWaypoint.angles.push_back(pt[i]);
  }

  int angularDuration = 0;
  angularWaypoint.duration = angularDuration;

  waypoint.oneof_type_of_waypoint.angular_waypoint.push_back(angularWaypoint);
  trajectory.duration = 0;
  trajectory.use_optimal_blending = false;
  trajectory.waypoints.push_back(waypoint);

  service_validate_waypoint_list.request.input = trajectory;
  if (!service_client_validate_waypoint_list.call(service_validate_waypoint_list))
  {
    std::string error_string = "Failed to call ValidateWaypointList";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  int error_number = service_validate_waypoint_list.response.output.trajectory_error_report.trajectory_error_elements.size();
  static const int MAX_ANGULAR_DURATION = 600;

  while (error_number >= 1 && angularDuration < MAX_ANGULAR_DURATION)
  {
    angularDuration++;
    trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angularDuration;

    service_validate_waypoint_list.request.input = trajectory;
    if (!service_client_validate_waypoint_list.call(service_validate_waypoint_list))
    {
      std::string error_string = "Failed to call ValidateWaypointList";
      ROS_ERROR("%s", error_string.c_str());
      return false;
    }
    error_number = service_validate_waypoint_list.response.output.trajectory_error_report.trajectory_error_elements.size();
  }

  if (angularDuration >= MAX_ANGULAR_DURATION)
  {
    // It should be possible to reach position within 30s
    // WaypointList is invalid (other error than angularWaypoint duration)
    std::string error_string = "WaypointList is invalid";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  service_execute_waypoints_trajectory.request.input = trajectory;

  if (service_client_execute_waypoints_trajectory.call(service_execute_waypoints_trajectory))
  {
    ROS_INFO("The joint angles were sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  wait_for_action_end_or_abort();
}

bool sendGripperCommand(ros::NodeHandle n, double value)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/base/send_gripper_command");
  kortex_driver::SendGripperCommand service_send_gripper_command;

  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  std::cout << "Moving gripper to position " << value << std::endl;
  service_send_gripper_command.request.input.gripper.finger.push_back(finger);
  service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

  if (service_client_send_gripper_command.call(service_send_gripper_command))  
  {
    ROS_INFO("The gripper command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendGripperCommand";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  ros::Duration(0, 500000000).sleep();
  return true;
}

// https://github.com/Kinovarobotics/ros_kortex/blob/373cacd4e3f7bba6dddaeb3451f31dbcaa64beb8/kortex_examples/src/move_it/example_move_it_trajectories.py#L103
//bool executeWaypoint(std::vector<double> waypoint, moveit::planning_interface::MoveGroupInterface &armGroup, moveit::planning_interface::MoveGroupInterface &gripperGroup)
bool executeWaypoint(std::vector<double> waypoint, ros::NodeHandle &n)
{
  bool status = true;
  status &= clearFaults(n);
  status &= moveToWaypoint(n, waypoint);
  status &= sendGripperCommand(n, waypoint[7]);

  std::cout << "Executed the waypoint" << std::endl;
  return status;
  /*armGroup.setGoalJointTolerance(0.01);
  std::vector<double> jointTarget = std::vector<double>(waypoint.begin(), waypoint.end()); // Copy all values but the last, which contains gripper position
  armGroup.setJointValueTarget(jointTarget);
  bool success = (armGroup.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //double gripperVal = waypoint[7] * (GRIPPER_MAX_ABSOLUTE_POS - GRIPPER_MIN_ABSOLUTE_POS) + GRIPPER_MIN_ABSOLUTE_POS;
  gripperGroup.setJointValueTarget(gripperGroup.getJointNames()[0], waypoint[7]);
  success &= (gripperGroup.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;*/
}


/*** Action server ***/
void execute(const prog_support_backend::InterfaceRequestGoalConstPtr& goal, Server* as, ros::NodeHandle &node_handle)
{
  prog_support_backend::InterfaceRequestResult result;
  bool status;
  //moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm", "/my_gen3/robot_description");
  //moveit::planning_interface::MoveGroupInterface armMoveGroup(move_group_options);
  //moveit::planning_interface::MoveGroupInterface armMoveGroup("arm");  
  //armMoveGroup.ROBOT_DESCRIPTION = "/my_gen3/robot_description";
  //moveit::planning_interface::MoveGroupInterface gripperMoveGroup("gripper");   

  if (goal->request == TEACH)
  {
    ROS_INFO("Activating teach mode");
    status = activateTeachMode(node_handle);
    if (status)
    {
      result.conclusion = "Set robot to joint mode";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to set robot to joint mode";
      as->setAborted(result);
    }    
  }
  else if (goal->request == STOP_TEACH)
  {
    status = deactivateTeachMode(node_handle);
    if (status)
    {
      result.conclusion = "Deactivated admittance";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to deactivate admittance";
      as->setAborted(result);
    }    
  }
  else if (goal->request == ADD_WAYPOINT)
  {
    //addWaypoint(armMoveGroup, gripperMoveGroup);
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
    //bool status = replaceWaypoint(armMoveGroup, gripperMoveGroup, goal->optional_index_1);
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
    clearFaults(node_handle);
    sendGripperCommand(node_handle, 0.00873365);

    if (userProgram.getHead() != nullptr)
    { 
      moveToWaypoint(node_handle, userProgram.getHeadValue()); // Account for not executing first waypoint sometimes
      std::cout << "Moved to initial position" << std::endl;
      //bool status = userProgram.iterate(executeWaypoint, armMoveGroup, gripperMoveGroup);

      std::cout << "Executing program of size " << userProgram.size() << std::endl;
      Node<std::vector<double>>* currentNode = userProgram.getHead();
       bool status;

      while (currentNode != nullptr) 
      {
          if (currentNode->isInLoop) 
          {
              int loopIterations = currentNode->loop->loopCount;
              int loopStartIndex = currentNode->loop->startIndex;
              int loopEndIndex = currentNode->loop->endIndex;
              std::cout << "Executing loop of iterations " << loopIterations << " with start index " << loopStartIndex << " and end index " << loopEndIndex << std::endl;

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
                std::cout << "Starting loop iteration number " << i  << std::endl;
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
              std::cout << "Executing waypoint that is not in a loop " << std::endl;
              // Execute the current waypoint.
              status = executeWaypoint(currentNode->value, node_handle);
              currentNode = currentNode->next;
          }
          //exec_pub.publish(i);
          i++;
      }

      //bool status = userProgram.iterate(executeWaypoint, node_handle);
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
    clearFaults(node_handle);
    sendGripperCommand(node_handle, 0.00873365);
    moveToWaypoint(node_handle, userProgram.getHeadValue()); // Account for not executing first waypoint sometimes
    std::cout << "Moved to initial position" << std::endl;

    /*DoublyLinkedList<std::vector<double>> subProgram = userProgram.copyUpToIndex(goal->optional_index_1);
    std::cout << "Executing program of size " << userProgram.size() << std::endl;
    bool status = subProgram.iterate(executeWaypoint, node_handle);
    if (status)
    {
      result.conclusion = "Executed user program until specified point";
      as->setSucceeded(result);
    }
    else
    {
      result.conclusion = "Failed to execute user program until specified point";
      as->setAborted(result);
    }  
    status = activateTeachMode(node_handle);
    if (!status)
    {
      std::cout << "Failed to activate teach mode after program execution" << std::endl;
    }*/


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
      //as->setAborted(result);
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
    std::cout << "GO_TO request with index " << waypointNum << std::endl;
    if (waypointNum== 0)
    {
      sendGripperCommand(node_handle, 0.00873365);
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
    std::cout << "Added a loop of iterations " << std::to_string(goal->optional_index_1) << " starting at " << std::to_string(goal->optional_index_2) << " and ending at " << std::to_string(goal->optional_index_3) << std::endl;
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

  ros::ServiceClient service_client_set_joint_speed_soft_limits = node_handle.serviceClient<kortex_driver::SetJointSpeedSoftLimits>("/control_config/set_joint_speed_soft_limits");
  kortex_driver::SetJointSpeedSoftLimits service_send_joint_speed_soft_limits;
  service_send_joint_speed_soft_limits.request.input.control_mode = ANGULAR_TRAJECTORY;
  service_send_joint_speed_soft_limits.request.input.joint_speed_soft_limits = {20, 20, 20, 20, 17, 17, 17};
  if (!service_client_set_joint_speed_soft_limits.call(service_send_joint_speed_soft_limits))
  {
    std::string error_string = "Failed to call SetJointSpeedSoftLimits";
    ROS_ERROR("%s", error_string.c_str());
  }
  std::cout << "Set speed limits" << std::endl;

  ros::ServiceClient service_client_set_joint_acceleration_soft_limits = node_handle.serviceClient<kortex_driver::SetJointAccelerationSoftLimits>("/control_config/set_joint_acceleration_soft_limits");
  kortex_driver::SetJointAccelerationSoftLimits service_send_joint_acceleration_soft_limits;
  service_send_joint_acceleration_soft_limits.request.input.control_mode = ANGULAR_TRAJECTORY;
  service_send_joint_acceleration_soft_limits.request.input.joint_acceleration_soft_limits = {75, 75, 75, 75, 143, 143, 143};
  if (!service_client_set_joint_acceleration_soft_limits.call(service_send_joint_acceleration_soft_limits))
  {
    std::string error_string = "Failed to call SetJointAccelerationSoftLimits";
    ROS_ERROR("%s", error_string.c_str());
  }

    std::cout << "Set acceleration limits" << std::endl;

  ros::AsyncSpinner spinner(2); 
  spinner.start();

   // Subscribe to the Action Topic
  ros::Subscriber sub = node_handle.subscribe("/action_topic", 1000, notification_callback);
  waypoint_pub = node_handle.advertise<marker_package::Waypoint>("/waypoint", 1000);
  //exec_pub = node_handle.advertise<uint8>("/waypoint", 1000);

  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = node_handle.serviceClient<kortex_driver::OnNotificationActionTopic>("/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
  }

  Server server(node_handle, "interface_controller", boost::bind(&execute, _1, &server, boost::ref(node_handle)), false);
  server.start();

  ros::waitForShutdown();
  
  return 0;
}
