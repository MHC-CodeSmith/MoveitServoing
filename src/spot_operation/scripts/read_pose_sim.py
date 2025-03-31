#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def get_moveit_end_effector_pose():
    rospy.init_node('moveit_finger_pose_node')

    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    group.set_end_effector_link("arm_link_fngr")  # <- força o link final como o dedo

    rospy.loginfo(f"Reference frame: {group.get_pose_reference_frame()}")
    rospy.loginfo(f"End-effector link: {group.get_end_effector_link()}")

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        pose = group.get_current_pose().pose
        rospy.loginfo(f"Current end-effector pose:\n{pose}")
        rate.sleep()

if __name__ == '__main__':
    get_moveit_end_effector_pose()
