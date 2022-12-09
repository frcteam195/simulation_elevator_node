#!/usr/bin/env python3

import tf
import tf2_ros
import geometry_msgs.msg
import rospy


REVOLUTIONS_PER_METER = 6000.0

def get_carriage_transforms(elevator_position_revolutions: float):
    """
    Returns the carriage transform as a single element in a stamped transform list.
    """

    carriage_transform = geometry_msgs.msg.TransformStamped()
    
    carriage_transform.header.stamp = rospy.Time.now()
    carriage_transform.header.frame_id = "base_link"

    carriage_transform.child_frame_id = "carriage_link"

    carriage_transform.transform.translation.x = float(0.5)
    carriage_transform.transform.translation.y = float(0.0)
    carriage_transform.transform.translation.z = float(elevator_position_revolutions / REVOLUTIONS_PER_METER)

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    carriage_transform.transform.rotation.x = quat[0]
    carriage_transform.transform.rotation.y = quat[1]
    carriage_transform.transform.rotation.z = quat[2]
    carriage_transform.transform.rotation.w = quat[3]

    return [carriage_transform]
