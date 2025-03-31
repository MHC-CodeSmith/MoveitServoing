#!/usr/bin/env python3

import sys
import time
import argparse
import os

import rospy
from sensor_msgs.msg import JointState

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.math_helpers import SE3Pose, Quat

# List of Spot arm joint names in the order you'd like to publish them.
# They must match how you want them labeled in your ROS environment or URDF.
ARM_JOINTS = [
    "arm0.sh0",  # Shoulder_0
    "arm0.sh1",  # Shoulder_1
    "arm0.el0",  # Elbow_0
    "arm0.el1",  # Elbow_1
    "arm0.wr0",  # Wrist_0
    "arm0.wr1",  # Wrist_1
    "arm0.f1x",  # Gripper finger position
]

def main():
    """
    Continuously publishes the current Spot arm joint positions (and velocities) as a ROS JointState.
    """

    # Initialize the ROS node
    rospy.init_node('spot_arm_joint_publisher', anonymous=True)

    # Create a publisher for the JointState
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Parse command-line arguments for Spot connection info
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(rospy.myargv()[1:])

    # Optional: environment-based auth, or set them directly
    # e.g., os.environ["BOSDYN_CLIENT_USERNAME"] = "admin"
    #       os.environ["BOSDYN_CLIENT_PASSWORD"] = "spotadmin2017"

    # Create SDK and connect to the robot
    bosdyn.client.util.setup_logging(options.verbose)
    sdk = bosdyn.client.create_standard_sdk("SpotArmJointPublisher")
    robot = sdk.create_robot(options.hostname)

    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # We only need RobotStateClient to read joint states. No lease required if we're not commanding the arm.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

    # Check the robot actually has an arm
    if not robot.has_arm():
        rospy.logerr("This Spot robot does not have an arm. Exiting.")
        return 1

    rospy.loginfo("Connected to Spot. Publishing arm joint states...")

    # We will publish at 10 Hz (adjust as desired)
    publish_rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Prepare the JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        # Retrieve the latest RobotState
        try:
            robot_state = robot_state_client.get_robot_state()
        except Exception as e:
            rospy.logerr(f"Failed to get robot state: {e}")
            publish_rate.sleep()
            continue

        # Extract arm joint states
        # (You could store them in a dictionary keyed by name for easy lookup.)
        arm_joint_positions = {}
        arm_joint_velocities = {}

        for link in robot_state.kinematic_state.joint_states:
            if link.name in ARM_JOINTS:
                arm_joint_positions[link.name] = link.position.value
                arm_joint_velocities[link.name] = link.velocity.value

        # Fill the JointState message in a consistent order
        # (the same order as ARM_JOINTS above)
        for joint_name in ARM_JOINTS:
            # Some joints might not be in the returned list if the arm is stowed or certain states
            # are missing. So we fall back to 0.0 if not found.
            
            
            joint_state_msg.name.append(joint_name.replace("0.", "_"))
            joint_state_msg.position.append(arm_joint_positions.get(joint_name, 0.0))
            joint_state_msg.velocity.append(arm_joint_velocities.get(joint_name, 0.0))
            # If you want to fill effort, you could do: joint_state_msg.effort.append(0.0)

        # Publish the message
        pub.publish(joint_state_msg)

        publish_rate.sleep()

    rospy.loginfo("Shutting down Spot arm joint publisher.")
    return 0


if __name__ == '__main__':
    sys.exit(main())