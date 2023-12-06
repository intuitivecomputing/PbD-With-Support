#!/usr/bin/env python3


# Run the code: rosrun marker_package marker_publisher.py

import rospy
from visualization_msgs.msg import Marker
from marker_package.msg import Waypoint # import the new message type
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import sys

# Initialize the ROS publisher and the tf2 listener outside the callback
pub = None
tf_buffer = None
tf_listener = None
markers = {}  # dictionary to store the markers
marker_num = []
def marker_exist(id1, id2):
    print("Markers exist, swapping now")
    # Check if the markers exist in the marker_num
    return id1 in marker_num and id2 in marker_num

def update_markers(deleted_marker_id):
    global markers
    print(deleted_marker_id)
    print(marker_num)
    if deleted_marker_id == max(marker_num):
        print("delete last id")
        markers[deleted_marker_id][0].action = Marker.DELETE
        markers[deleted_marker_id][1].action = Marker.DELETE
        pub.publish(markers[deleted_marker_id][0])
        pub.publish(markers[deleted_marker_id][1])
        # Remove the marker from the dictionary
        del markers[deleted_marker_id]

        # Remove the marker id from the list
        marker_num.remove(deleted_marker_id)
        print(marker_num)

    
    else:
        print("delete middle id")
        # Delete from the markers dict
        del markers[deleted_marker_id]
        marker_num.remove(deleted_marker_id)    
        # Iterate over markers with an ID greater than the deleted marker's ID
        for id in range(deleted_marker_id + 1, max(marker_num) + 1):

            # Delete old markers from visualization
            markers[id][0].action = Marker.DELETE
            markers[id][1].action = Marker.DELETE
            pub.publish(markers[id][0])
            pub.publish(markers[id][1])
            
            # Update the markers' IDs in the publisher
            markers[id][0].id -= 1
            markers[id][1].id -= 1

            # Update the text marker's text property
            markers[id][1].text = str(int(markers[id][1].text) - 1)

            # Set marker actions back to ADD
            markers[id][0].action = Marker.ADD
            markers[id][1].action = Marker.ADD

            # Publish the updated marker
            pub.publish(markers[id][0])
            pub.publish(markers[id][1])

            # Update the dictionary keys
            markers[id - 1] = markers.pop(id)

        # Update the marker_num list
        for i in range(len(marker_num)):
            if marker_num[i] > deleted_marker_id:
                marker_num[i] -= 1

        print(marker_num)

def clear_all():
    global markers, marker_num, pub

    # Iterate through all markers
    for id in marker_num:
        print(id)

        # Delete each marker from visualization
        markers[id][0].action = Marker.DELETEALL
        markers[id][1].action = Marker.DELETEALL
        pub.publish(markers[id][0])
        rospy.sleep(0.01)
        pub.publish(markers[id][1])
        rospy.sleep(0.01)

    # Clear the dictionary and list
    markers.clear()
    marker_num.clear()
    print("Cleared all markers")



def waypoint_cb(msg):
    global pub, tf_buffer, tf_listener, markers, marker_num

    if msg.extra_num == 0:
        #Add markers
        if msg.num != 0:
            if msg.num > 0:
                print(msg.num)
                # Create a marker
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = Marker.SPHERE
                marker.ns = "sphere"
                marker.id = msg.num
                marker.action = Marker.ADD
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                # Create text marker
                text_marker = Marker()
                text_marker.header.frame_id = "/map"
                text_marker.ns = "text"
                text_marker.id = msg.num
                # Set the text marker's text to "Waypoint: " followed by the waypoint number
                text_marker.text = str(msg.num)
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.x = 0.1
                text_marker.scale.y = 0.1
                text_marker.scale.z = 0.1
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.lifetime = rospy.Duration()

                # Check if the transform is available
                if tf_buffer.can_transform('base_link', 'tool_frame', rospy.Time(0)):
                    # Get the transform
                    transform = tf_buffer.lookup_transform('base_link', 'tool_frame', rospy.Time(0))

                    # Create a PointStamped message to represent the end-effector's position
                    point_stamped = PointStamped()
                    point_stamped.header.frame_id = 'tool_frame'
                    point_stamped.header.stamp = rospy.Time.now()

                    # Transform the point to the 'shoulder_link' frame
                    point_transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

                    # Set the marker's position to the end-effector's position
                    marker.pose.position.x = point_transformed.point.x
                    marker.pose.position.y = point_transformed.point.y
                    marker.pose.position.z = point_transformed.point.z

                    # Set the text marker's position to the same as the sphere's
                    text_marker.pose.position.x = point_transformed.point.x + 0.05
                    text_marker.pose.position.y = point_transformed.point.y + 0.05
                    text_marker.pose.position.z = point_transformed.point.z + 0.05

                # Publish the markers
                pub.publish(marker)
                rospy.sleep(0.01)
                pub.publish(text_marker)
                rospy.sleep(0.01)


                # Add or update the markers in the dictionary
                markers[msg.num] = (marker, text_marker)
                marker_num.append(msg.num)
            else: # msg num is -id swapid1 is bool indicating highlight or not
                change_marker_color((msg.num * -1), msg.swap_id1)

        # Deleting Markers        
        elif msg.swap_id1 == 0 and msg.swap_id2 < 0:
            print('start delete')
            deleted_marker_id = -msg.swap_id2

            # Check if marker exists before deleting
            if deleted_marker_id in markers:
                # Update the remaining markers
                update_markers(deleted_marker_id)

        #Clear all markers
        elif msg.num == 0 and msg.swap_id1 == 0 and msg.swap_id2 == 0:
            print("Clear all")
            clear_all()

        #Shift markers
        else:
            # Check if we need to shift markers and if they exist
            if msg.swap_id1 != 0 and msg.swap_id2 != 0 and marker_exist(msg.swap_id1, msg.swap_id2):
                # Call the shift_markers function
                print("Attempting to shift markers")
                shift_markers(msg.swap_id1, msg.swap_id2)
    else:
        # Change all marker color
        if msg.extra_num == -1 and msg.swap_id1 == 0 and msg.swap_id2 == 0:
            for id, marker_tuple in list(markers.items()):
                marker, text_marker = marker_tuple        # Change the marker's color to yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0        # Change the text marker's color to yellow as well, if desired
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0        # Publish the updated markers
                pub.publish(marker)
                rospy.sleep(0.01)
                pub.publish(text_marker)
                rospy.sleep(0.01)        
                #print("Changed all markers to yellow")


def change_marker_color(id, highlight):
    if highlight:
        print("Highlighting marker {}".format(id))
        markers[id][0].color.g = 0.0
        markers[id][0].color.b = 1.0
        markers[id][1].color.g = 0.0
        markers[id][1].color.b = 1.0
        # Publish the updated markers
        pub.publish(markers[id][0])
        pub.publish(markers[id][1])
    else:
        print("Unhighlighting marker {}".format(id))
        markers[id][0].color.g = 1.0
        markers[id][0].color.b = 0.0
        markers[id][1].color.g = 1.0
        markers[id][1].color.b = 0.0
        # Publish the updated markers
        pub.publish(markers[id][0])
        pub.publish(markers[id][1])

# def swap_markers(id1, id2):
#     global markers, marker_num

#     # Check if the markers exist
#     if marker_exist(id1, id2):
#         print("Before swap: Marker {} position: {}".format(id1, markers[id1][1].pose.position))
#         print("Before swap: Marker {} position: {}".format(id2, markers[id2][1].pose.position))
#         # Swap the positions
#         markers[id1][0].pose.position, markers[id2][0].pose.position = markers[id2][0].pose.position, markers[id1][0].pose.position
#         markers[id1][1].pose.position, markers[id2][1].pose.position = markers[id2][1].pose.position, markers[id1][1].pose.position
#         print("After swap: Marker {} position: {}".format(id1, markers[id1][1].pose.position))
#         print("After swap: Marker {} position: {}".format(id2, markers[id2][1].pose.position))


#         # Swap the text
#         # markers[id1][1].text, markers[id2][1].text = markers[id2][1].text, markers[id1][1].text

#         # Publish the updated markers
#         pub.publish(markers[id1][0])
#         pub.publish(markers[id1][1])
#         pub.publish(markers[id2][0])
#         pub.publish(markers[id2][1])

#         # Update marker_num
#         idx1 = marker_num.index(id1)
#         idx2 = marker_num.index(id2)
#         marker_num[idx1], marker_num[idx2] = marker_num[idx2], marker_num[idx1]
#         print("Swap finished")

def shift_markers(id1, id2):
    global markers, marker_num

    if marker_exist(id1, id2):
        if id1 > id2:
            # Save marker to be shifted
            shifted_marker = markers.pop(id1)

            # Shift other markers
            for id in range(id1, id2, -1):
                # Update the markers' IDs in the publisher
                markers[id-1][0].id += 1
                markers[id-1][1].id += 1

                # Update the text marker's text property
                markers[id-1][1].text = str(int(markers[id-1][1].text) + 1)

                # Publish the updated marker
                pub.publish(markers[id-1][0])
                rospy.sleep(0.01)
                pub.publish(markers[id-1][1])
                rospy.sleep(0.01)

                # Update the dictionary keys
                markers[id] = markers.pop(id-1)

            # Place shifted marker
            shifted_marker[0].id = id2
            shifted_marker[1].id = id2
            shifted_marker[1].text = str(id2)
            markers[id2] = shifted_marker
            pub.publish(shifted_marker[0])
            rospy.sleep(0.01)
            pub.publish(shifted_marker[1])
            rospy.sleep(0.01)

        elif id1 < id2:
            # Save marker to be shifted
            shifted_marker = markers.pop(id1)

            # Shift other markers
            for id in range(id1, id2):
                # Update the markers' IDs in the publisher
                markers[id+1][0].id -= 1
                markers[id+1][1].id -= 1

                # Update the text marker's text property
                markers[id+1][1].text = str(int(markers[id+1][1].text) - 1)

                # Publish the updated marker
                pub.publish(markers[id+1][0])
                rospy.sleep(0.01)
                pub.publish(markers[id+1][1])
                rospy.sleep(0.01)

                # Update the dictionary keys
                markers[id] = markers.pop(id+1)

            # Place shifted marker
            shifted_marker[0].id = id2
            shifted_marker[1].id = id2
            shifted_marker[1].text = str(id2)
            markers[id2] = shifted_marker
            pub.publish(shifted_marker[0])
            rospy.sleep(0.01)
            pub.publish(shifted_marker[1])
            rospy.sleep(0.01)
        # Update marker_num
        marker_num.sort()
        print("Shift finished")

def marker_publisher():
    global pub, tf_buffer, tf_listener

    rospy.init_node('marker_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    waypoint_sub = rospy.Subscriber('waypoint', Waypoint, waypoint_cb)  # new subscriber
    rospy.spin()

if __name__ == '__main__':
    try:
        marker_publisher()
    except rospy.ROSInterruptException:
        pass









# import rospy
# from visualization_msgs.msg import Marker

# def marker_publisher():
#     rospy.init_node('marker_publisher')
#     pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

#     marker = Marker()
#     marker.header.frame_id = "/map"
#     marker.type = marker.SPHERE
#     marker.action = marker.ADD
#     marker.scale.x = 0.3 
#     marker.scale.y = 0.3
#     marker.scale.z = 0.3
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.pose.orientation.w = 1.0
#     marker.pose.position.x = 3
#     marker.pose.position.y = 0
#     marker.pose.position.z = 0

#     rate = rospy.Rate(120) # 10 Hz
#     while not rospy.is_shutdown():
#         pub.publish(marker)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         marker_publisher()
#     except rospy.ROSInterruptException:
#         pass
