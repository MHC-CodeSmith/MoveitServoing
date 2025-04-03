#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import SE3Pose, Quat
import time

def get_simulated_end_effector_pose(group):
    """
    Lê a pose atual do end-effector da simulação usando MoveIt.
    """
    rospy.sleep(0.1)  # Pequena pausa para garantir atualização
    return group.get_current_pose().pose

def continuous_send_pose(spot_hostname, group, command_client, robot_state_client):
    """
    Em loop, lê a pose simulada, converte para o frame 'odom' usando a transformação do robô real,
    e envia o comando para o braço do Spot.
    """
    rate = rospy.Rate(2)  # Atualiza 2 Hz (ajuste conforme necessário)
    while not rospy.is_shutdown():
        # 1. Obtém a pose do end-effector na simulação.
        sim_pose = get_simulated_end_effector_pose(group)

        # 2. Lê a transformação atual do Spot: de ODOM para o corpo (gravity-aligned)
        robot_state = robot_state_client.get_robot_state()
        # Aqui usamos "body" como o frame base do Spot (que deve estar definido no URDF/SDK)
        odom_T_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                    ODOM_FRAME_NAME, "body")

        # 3. Constrói a pose do end-effector em relação ao corpo (como se estivesse definida no 'body')
        flat_body_T_hand = SE3Pose(
            x=sim_pose.position.x,
            y=sim_pose.position.y,
            z=sim_pose.position.z,
            rot=Quat(
                w=sim_pose.orientation.w,
                x=sim_pose.orientation.x,
                y=sim_pose.orientation.y,
                z=sim_pose.orientation.z
            )
        )

        # 4. Converte a pose para o frame global (odom)
        odom_T_hand = odom_T_body * flat_body_T_hand

        # 5. Cria o comando de movimento com uma duração curta (para atualização contínua)
        seconds = 0.5  # Duração do comando; ajuste conforme a resposta do braço real
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z,
            odom_T_hand.rot.w, odom_T_hand.rot.x, odom_T_hand.rot.y, odom_T_hand.rot.z,
            ODOM_FRAME_NAME, seconds
        )

        # 6. Envia o comando
        command_client.robot_command(arm_command)
        rospy.loginfo("Comando enviado: Pose alvo (odom) = [%.3f, %.3f, %.3f]", 
                      odom_T_hand.x, odom_T_hand.y, odom_T_hand.z)

        rate.sleep()

def main():
    rospy.init_node("continuous_moveit_pose_to_spot_real", anonymous=True)

    # Inicializa o moveit_commander e o grupo de manipulação da simulação
    moveit_commander.roscpp_initialize([])
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_end_effector_link("arm_link_fngr")
    rospy.loginfo("Simulação: End-effector (%s) em relação a: %s", 
                  group.get_end_effector_link(), group.get_pose_reference_frame())

    # Conecta ao Spot real
    spot_hostname = rospy.get_param("~spot_hostname", "192.168.80.3")
    sdk = bosdyn.client.create_standard_sdk("ContinuousPoseToSpot")
    robot = sdk.create_robot(spot_hostname)
    robot.authenticate("admin", "spotadmin2017")
    robot.time_sync.wait_for_sync()

    # Verifica se o robô possui braço e está pronto
    if not robot.has_arm():
        rospy.logerr("Spot não possui braço. Abortando.")
        return 1
    robot.power_on(timeout_sec=20)
    robot.assert_arm_ready_for_command()

    # Inicializa os clientes do Spot
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    rospy.loginfo("Conectado ao Spot real. Iniciando sincronização contínua...")

    # Usa o lease para manter o controle do robô
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):
        continuous_send_pose(spot_hostname, group, command_client, robot_state_client)

if __name__ == "__main__":
    main()
