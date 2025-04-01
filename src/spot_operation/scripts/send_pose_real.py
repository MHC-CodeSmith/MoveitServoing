#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import SE3Pose, Quat

def get_simulated_end_effector_pose():
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_end_effector_link("arm_link_fngr")
    rospy.sleep(1.0)
    pose = group.get_current_pose().pose
    return pose

def move_real_spot_to_sim_pose(spot_hostname, sim_pose):
    # Conecta ao Spot
    sdk = bosdyn.client.create_standard_sdk("MoveSimPoseToRealSpot")
    robot = sdk.create_robot(spot_hostname)
    robot.authenticate("admin", "spotadmin2017")
    robot.time_sync.wait_for_sync()

    # Inicializa os clientes
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
    
        # Ativa o robô
        robot.assert_arm_ready_for_command()

        # Lê transformação odom_T_body (gravity-aligned body)
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                        ODOM_FRAME_NAME, "body")

        # Constrói pose do MoveIt em relação ao body (equivalente ao flat_body_T_hand)
        flat_body_T_hand = SE3Pose(
            x=sim_pose.position.x,
            y=sim_pose.position.y,
            z=sim_pose.position.z,
            rot=Quat(w=sim_pose.orientation.w,
                    x=sim_pose.orientation.x,
                    y=sim_pose.orientation.y,
                    z=sim_pose.orientation.z)
        )

        # Converte para odometria (odom_T_hand)
        odom_T_hand = odom_T_flat_body * flat_body_T_hand

        # Cria o comando
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z,
            odom_T_hand.rot.w, odom_T_hand.rot.x, odom_T_hand.rot.y, odom_T_hand.rot.z,
            ODOM_FRAME_NAME, 2.0)

        # Manda o comando
        command_client.robot_command(arm_command)
        rospy.loginfo("Comando enviado para o Spot real.")

def main():
    rospy.init_node("send_moveit_pose_to_spot_real")

    rospy.loginfo("Lendo pose do MoveIt (simulação)...")
    sim_pose = get_simulated_end_effector_pose()

    spot_hostname = rospy.get_param("~spot_hostname", "192.168.80.3")
    move_real_spot_to_sim_pose(spot_hostname, sim_pose)

    rospy.loginfo("Finalizado.")
    rospy.signal_shutdown("Feito")

if __name__ == "__main__":
    main()

# import rospy
# import moveit_commander
# from geometry_msgs.msg import PoseStamped

# import bosdyn.client
# import bosdyn.client.util
# from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
# from bosdyn.client.robot_state import RobotStateClient
# from bosdyn.client.math_helpers import SE3Pose, Quat

# def get_simulated_end_effector_pose():
#     moveit_commander.roscpp_initialize([])
#     group = moveit_commander.MoveGroupCommander("manipulator")
#     group.set_end_effector_link("arm_link_fngr")
#     rospy.sleep(1.0)
#     pose = group.get_current_pose().pose
#     return pose

# def move_real_spot_to_pose(spot_hostname, target_pose):
#     # Conecta ao Spot
#     sdk = bosdyn.client.create_standard_sdk("MoveToPose")
#     robot = sdk.create_robot(spot_hostname)
#     robot.authenticate("admin", "spotadmin2017")
#     robot.time_sync.wait_for_sync()

#     # Certifique-se de que o robô esteja ativado
#     robot.power_on(timeout_sec=20)
#     robot.assert_arm_ready_for_command()

#     # Inicializa os clientes
#     command_client = robot.ensure_client(RobotCommandClient.default_service_name)

#     # Converte a pose para SE3Pose
#     pose_spot = SE3Pose(
#         x=target_pose.position.x,
#         y=target_pose.position.y,
#         z=target_pose.position.z,
#         rot=Quat(
#             w=target_pose.orientation.w,
#             x=target_pose.orientation.x,
#             y=target_pose.orientation.y,
#             z=target_pose.orientation.z,
#         )
#     )

#     # Cria e envia o comando de movimento do braço
#     arm_command = RobotCommandBuilder.arm_pose_command(
#         root_frame_name="body",
#         hand_pose=pose_spot,
#         duration=3.0
#     )
#     command_client.robot_command(arm_command)

#     rospy.loginfo("Comando enviado para o braço do Spot.")

# def main():
#     rospy.init_node("send_sim_pose_to_spot_real")

#     # Lê pose atual do MoveIt (simulação)
#     rospy.loginfo("Obtendo pose do end-effector da simulação...")
#     sim_pose = get_simulated_end_effector_pose()

#     # Envia comando para o Spot real
#     spot_hostname = rospy.get_param("~spot_hostname", "192.168.80.3")
#     move_real_spot_to_pose(spot_hostname, sim_pose)

#     rospy.loginfo("Finalizado.")
#     rospy.signal_shutdown("Done")

# if __name__ == "__main__":
#     main()
