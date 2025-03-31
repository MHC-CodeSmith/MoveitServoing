#!/usr/bin/env python3

import sys
import time
import argparse

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, HAND_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import SE3Pose, Quat


def spot_gripper_pose_to_ros_pose(spot_pose: SE3Pose) -> PoseStamped:
    """Convert Spot's SE3Pose (relative to BODY_FRAME) into a ROS PoseStamped message."""
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "base_link"  
    # ^ "base_link" depends on your Gazebo/robot model. Could be "world" or "odom" in your setup.

    pose_msg.pose.position.x = spot_pose.x
    pose_msg.pose.position.y = spot_pose.y
    pose_msg.pose.position.z = spot_pose.z

    pose_msg.pose.orientation.w = spot_pose.rot.w
    pose_msg.pose.orientation.x = spot_pose.rot.x
    pose_msg.pose.orientation.y = spot_pose.rot.y
    pose_msg.pose.orientation.z = spot_pose.rot.z

    return pose_msg


def retrieve_spot_hand_pose(robot_state_client) -> SE3Pose:
    """
    Retrieve the gripper (hand) pose from Spot with respect to the BODY_FRAME.
    Returns an SE3Pose for the hand in the BODY frame.
    """
    # Grab the latest state from the robot.
    robot_state = robot_state_client.get_robot_state()

    # We use get_a_tform_b() to get BODY_FRAME -> HAND_FRAME transform
    body_T_hand = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                BODY_FRAME_NAME, HAND_FRAME_NAME)
    # body_T_hand is already an SE3Pose
    return body_T_hand


def main():
    """
    1) Connect to Spot and retrieve the gripper transform in BODY frame.
    2) Convert to a ROS pose.
    3) Publish that pose so your Gazebo arm can move to it (or for RViz visualization).
    """
    rospy.init_node('spot_gripper_to_gazebo_node')
    pub = rospy.Publisher('/joint_states', PoseStamped, queue_size=10)
    # ^ Replace '/gazebo_arm_pose_cmd' with whatever topic your Gazebo arm controller listens on.

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options, unknown = parser.parse_known_args(sys.argv[1:])

    # Create Spot SDK object and robot connection
    bosdyn.client.util.setup_logging(False)
    sdk = bosdyn.client.create_standard_sdk('SpotGripperPoseToGazebo')
    robot = sdk.create_robot(options.hostname)

    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # Make sure the robot has an arm
    if not robot.has_arm():
        rospy.logerr("Robot does not have an arm. Exiting.")
        return 1

    # Create the RobotState client
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # You may want to acquire a lease if you also plan to move the real arm,
    # but for simply reading state, it is not strictly required:
    # lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    # with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
    #     # etc...

    rate = rospy.Rate(1.0)  # Publish at 1 Hz, for example

    rospy.loginfo("spot_gripper_to_gazebo_node is running. Publishing Spot gripper pose to Gazebo...")

    while not rospy.is_shutdown():
        try:
            # 1. Get the latest hand pose from Spot
            hand_in_body = retrieve_spot_hand_pose(robot_state_client)

            # 2. Convert Spot's SE3Pose to a standard ROS Pose
            hand_pose_ros = spot_gripper_pose_to_ros_pose(hand_in_body)

            # 3. Publish that PoseStamped so your Gazebo node (or your custom node) can
            #    command the simulated arm or just display in RViz.
            pub.publish(hand_pose_ros)

            rospy.loginfo_once(
                f"Publishing Spot gripper pose in BODY frame. (x: {hand_in_body.x:.3f}, "
                f"y: {hand_in_body.y:.3f}, z: {hand_in_body.z:.3f})"
            )

        except Exception as e:
            rospy.logerr(f"Error retrieving Spot hand pose: {e}")

        rate.sleep()

    return 0


if __name__ == '__main__':
    sys.exit(main())