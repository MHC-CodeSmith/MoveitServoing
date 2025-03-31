#include <iostream>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

static const std::string LOGNAME = "cpp_interface_example";

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();
  std::string planning_frame, target_frame;

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
  ros::Publisher target_pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

  ros::param::get("~planning_frame", planning_frame);
  ros::param::param<std::string>("~target_frame", target_frame, "ee_target");
  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  // Make tf listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  Eigen::Vector3d lin_tol{ 0, 0, 0};
  double rot_tol = 0;

  // Get the current EE transform
  geometry_msgs::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  geometry_msgs::TransformStamped target_ee_tf;
  geometry_msgs::TransformStamped empty_tf;

  // Convert it to a Pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;


  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker.resetTargetPose();

  // Publish target pose
  target_pose.header.stamp = ros::Time::now();
  target_pose_pub.publish(target_pose);
  
  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread(
      [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.15 /* target pose timeout */); });

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    try{
      target_ee_tf = tfBuffer.lookupTransform(planning_frame, target_frame,
                            ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      target_ee_tf = empty_tf;
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    // tracker.getCommandFrameTransform(current_ee_tf);
    // Convert it to a Pose
    if (target_ee_tf != empty_tf){
      target_pose.pose.position.x = target_ee_tf.transform.translation.x;
      target_pose.pose.position.y = target_ee_tf.transform.translation.y;
      target_pose.pose.position.z = target_ee_tf.transform.translation.z;
      target_pose.pose.orientation = target_ee_tf.transform.rotation;
      target_pose.header.stamp = ros::Time::now();
      target_pose_pub.publish(target_pose);
    }
    
    loop_rate.sleep();
  }

  // Make sure the tracker is stopped and clean up
  tracker.stopMotion();
  move_to_pose_thread.join();

  return EXIT_SUCCESS;
}