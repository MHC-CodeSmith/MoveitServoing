#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("send_pose_goal_node", anonymous=True)

    moveit_commander.roscpp_initialize([])
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")  # nome do group no SRDF

    rospy.sleep(2)  # espera o MoveIt inicializar direito

    # Cria uma pose alvo arbitrária
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.header.stamp = rospy.Time.now()

    # POSIÇÃO ARBITRÁRIA (ajuste conforme necessário)
    target_pose.pose.position.x = 0.9
    target_pose.pose.position.y = 0.25
    target_pose.pose.position.z = 0.3

    # ORIENTAÇÃO em quaternion (w,x,y,z) – sem rotação aqui
    target_pose.pose.orientation.w = 1.0
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0

    # Define e executa o movimento
    rospy.loginfo("Enviando pose alvo para o MoveIt...")
    group.set_pose_target(target_pose)

    success = group.go(wait=True)

    # Limpa os targets após o movimento
    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo("Movimento concluído com sucesso!")
    else:
        rospy.logwarn("Falha ao executar o movimento!")

    rospy.signal_shutdown("Script finalizado")

if __name__ == '__main__':
    main()
