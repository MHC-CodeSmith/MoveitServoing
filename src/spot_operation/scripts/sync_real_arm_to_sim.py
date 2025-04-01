#!/usr/bin/env python3
import rospy
import moveit_commander
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient

# Lista dos nomes das juntas do Spot (como vêm da SDK)
ARM_JOINTS = [
    "arm0.sh0",
    "arm0.sh1",
    "arm0.el0",
    "arm0.el1",
    "arm0.wr0",
    "arm0.wr1",
    # "arm0.f1x",
]

# Converte nomes da SDK do Spot para os nomes usados no MoveIt (baseado no SRDF/URDF)
def convert_joint_name(spot_name):
    return spot_name.replace("0.", "_")
def get_current_joint_positions(spot_hostname):
    rospy.loginfo("Conectando ao Spot em %s...", spot_hostname)

    sdk = bosdyn.client.create_standard_sdk("SyncArmToSim")
    robot = sdk.create_robot(spot_hostname)

    # Login automático direto
    robot.authenticate("admin", "spotadmin2017")  
    robot.time_sync.wait_for_sync()

    if not robot.has_arm():
        rospy.logerr("Este Spot não possui braço. Abortando.")
        return None

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_state = robot_state_client.get_robot_state()

    joint_positions = {}
    for joint in robot_state.kinematic_state.joint_states:
        if joint.name in ARM_JOINTS:
            converted = convert_joint_name(joint.name)
            joint_positions[converted] = joint.position.value

    rospy.loginfo("Juntas do Spot real lidas com sucesso.")
    return joint_positions

# Move o braço na simulação (Gazebo + MoveIt) para a posição lida
def move_sim_arm_to_joint_positions(joint_goal):
    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    rospy.sleep(1.0)  # Garante que o MoveIt inicializou

    rospy.loginfo("Movendo braço simulado para a posição atual do Spot real...")

    group.set_joint_value_target(joint_goal)

    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo("Movimento de sincronização concluído com sucesso.")
    else:
        rospy.logwarn("Falha ao executar o movimento de sincronização.")

# Função principal
def main():
    rospy.init_node("sync_real_arm_to_sim", anonymous=True)

    spot_hostname = rospy.get_param("~spot_hostname", "192.168.80.3")
    joint_goal = get_current_joint_positions(spot_hostname)

    if joint_goal:
        move_sim_arm_to_joint_positions(joint_goal)
    else:
        rospy.logerr("Não foi possível recuperar o estado do braço do Spot.")

    rospy.signal_shutdown("Script concluído.")

if __name__ == "__main__":
    main()
