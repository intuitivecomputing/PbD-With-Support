#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from marker_package.msg import Waypoint  # Import the new message type
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# Initialize the ROS publisher and the tf2 listener outside the callback
pub = None
tf_buffer = None
tf_listener = None
markers = {}  # Dictionary to store the markers
marker_num = []

# Rest of your functions (marker_exist, update_markers, etc.) go here...
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
        pub.publish(markers[id][1])

    # Clear the dictionary and list
    markers.clear()
    marker_num.clear()
    print("Cleared all markers")

def waypoint_cb(msg):
    global pub, tf_buffer, tf_listener, markers, marker_num

    # Retrieve frame names from parameters
    base_frame = rospy.get_param('~base_frame', 'default_base_frame')
    tool_frame = rospy.get_param('~tool_frame', 'default_tool_frame')

    if msg.extra_num == 0:
        # Add markers
        if msg.num > 0:
            print("Adding marker", msg.num)
            # (Your existing code for adding markers...)

        elif msg.num < 0:  # msg.num is -id; swapid1 is bool indicating highlight or not
            change_marker_color((-msg.num), msg.swap_id1)

        # Deleting Markers
        elif msg.swap_id1 == 0 and msg.swap_id2 < 0:
            print('Deleting marker')
            deleted_marker_id = -msg.swap_id2
            if deleted_marker_id in markers:
                update_markers(deleted_marker_id)

        # Clear all markers
        elif msg.num == 0 and msg.swap_id1 == 0 and msg.swap_id2 == 0:
            print("Clearing all markers")
            clear_all()

        # Shift markers
        else:
            if msg.swap_id1 != 0 and msg.swap_id2 != 0 and marker_exist(msg.swap_id1, msg.swap_id2):
                print("Shifting markers")
                shift_markers(msg.swap_id1, msg.swap_id2)

    else:
        # Change all marker colors
        if msg.extra_num == -1 and msg.swap_id1 == 0 and msg.swap_id2 == 0:
            print("Changing color of all markers")
            for id, marker_tuple in markers.items():
                marker, text_marker = marker_tuple
                # Change the marker's color
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                # Change the text marker's color
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                # Publish the updated markers
                pub.publish(marker)
                pub.publish(text_marker)

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
                pub.publish(markers[id-1][1])

                # Update the dictionary keys
                markers[id] = markers.pop(id-1)

            # Place shifted marker
            shifted_marker[0].id = id2
            shifted_marker[1].id = id2
            shifted_marker[1].text = str(id2)
            markers[id2] = shifted_marker
            pub.publish(shifted_marker[0])
            pub.publish(shifted_marker[1])

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
                pub.publish(markers[id+1][1])

                # Update the dictionary keys
                markers[id] = markers.pop(id+1)

            # Place shifted marker
            shifted_marker[0].id = id2
            shifted_marker[1].id = id2
            shifted_marker[1].text = str(id2)
            markers[id2] = shifted_marker
            pub.publish(shifted_marker[0])
            pub.publish(shifted_marker[1])
            
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