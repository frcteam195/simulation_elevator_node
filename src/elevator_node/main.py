#!/usr/bin/env python3

import tf2_ros
import rospy
from threading import Thread

from elevator_node.carriage_link import *
from ck_utilities_py_node.motor import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode


def ros_func():
    global hmi_updates
    global robot_status

    elevatorMotor = Motor(9, MotorType.TalonFX)
    elevatorMotor.set_defaults()
    elevatorMotor.set_neutral_mode(NeutralMode.Brake)
    elevatorMotor.set_forward_soft_limit(18000.0)
    elevatorMotor.set_reverse_soft_limit(0.0)
    elevatorMotor.apply()

    clawMotor = Motor(10, MotorType.TalonFX)
    clawMotor.set_defaults()
    clawMotor.apply()

    carriage_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if hmi_updates.get() is None:
            continue

        if robot_status.get_mode() == RobotMode.TELEOP:
            elevatorMotor.set(ControlMode.PERCENT_OUTPUT, hmi_updates.get().elevator_vertical, 0.0)

            if (hmi_updates.get().claw_open):
                clawMotor.set(ControlMode.MOTION_MAGIC, 0.0, 0.0)
            else:
                clawMotor.set(ControlMode.MOTION_MAGIC, 500.0, 0.0)

        carriage_broadcaster.sendTransform(get_carriage_transforms(elevatorMotor.get_sensor_position()))

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    main_thread = Thread(target=ros_func)
    main_thread.start()

    rospy.spin()

    main_thread.join(5)
