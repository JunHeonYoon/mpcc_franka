#ifndef TTMPC_CONTROLLER_H
#define TTMPC_CONTROLLER_H

#pragma once
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <memory>

#include <thread>
#include <mutex>
#include <chrono>

#include <Eigen/Dense>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka/robot_state.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>

#include "suhan_benchmark.h"
#include "math_type_define.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include <fcntl.h>
#include <termios.h>

// ================ TTMPC ================
#include <nlohmann/json.hpp>
#include "MPC/mpc.h"
#include "Params/track.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
// ======================================


namespace ttmpc_franka 
{

class ttmpc_controller : public controller_interface::MultiInterfaceController<
								                franka_hw::FrankaModelInterface,
                                hardware_interface::VelocityJointInterface,
								                franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time);

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
  franka_hw::TriggerRate state_pub_trigger_{100}; 
	
  // initial state
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Matrix<double, 7, 1> qdot_init_;

  // current state
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qdot_;
  // Eigen::Matrix<double, 7, 1> torque_;

  // control value
  Eigen::Matrix<double, 7, 1> q_desired_;
  Eigen::Matrix<double, 7, 1> qdot_desired_;


  // Task space
	Eigen::Matrix<double, 3, 1> x_;
	Eigen::Matrix<double, 3, 1> x_init_;
	Eigen::Matrix<double, 3, 3> rotation_;
	Eigen::Matrix<double, 3, 3> rotation_init_;
	Eigen::Matrix<double, 3, 1> phi_;
	Eigen::Matrix<double, 6, 1> x_dot_;   // 6D (linear + angular)
	Eigen::Matrix<double, 6, 1> x_dot_desired_;   // 6D (linear + angular)
	Eigen::Matrix<double, 6, 1> x_dot_init_;   // 6D (linear + angular)
	Eigen::Matrix<double, 6, 1> x_error_;

  // dynamics
  // Eigen::Matrix<double, 7, 1> g_; // gravity matrix
  // Eigen::Matrix<double, 7, 7> m_; // mass matrix
  // Eigen::Matrix<double, 7, 7> m_inv_; // Inverse of mass matrix
  // Eigen::Matrix<double, 7, 1> c_; // coliolis matrix

  // For controller
	Eigen::Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Eigen::Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix

  // transformation matrix
  Eigen::Affine3d transform_init_;
  Eigen::Affine3d transform_;

  std::ofstream debug_file_;

  bool is_initialized_{false};

  const double hz_{1000.};
  ros::Time start_time_;
  ros::Time play_time_;
  ros::Time control_start_time_;
  SuhanBenchmark bench_timer_;

  enum CTRL_MODE{NONE, HOME, TTMPC, TTMPC_PICK, TTMPC_DROP, TTMPC_PLACE};
  CTRL_MODE control_mode_{NONE};
	bool is_mode_changed_ {false};

  std::mutex calculation_mutex_;
  std::mutex input_mutex_;

  bool quit_all_proc_{false};
  std::thread async_calculation_thread_;
  std::thread mode_change_thread_;
  std::thread state_pub_thread_;


  enum GRIPPER_MODE{OPEN, CLOSE, NEUTRAL};
  GRIPPER_MODE gripper_command_{OPEN};
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close_
  {"/franka_gripper/grasp", true};
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open_
  {"/franka_gripper/move", true};
  actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_ac_homing_
  {"/franka_gripper/homing", true};

  ros::Publisher control_mode_pub_;
  // ================ TTMPC ================
  franka_hw::TriggerRate ttmpc_trigger_;  
  std::thread async_ttmpc_thread_;
  std::mutex ttmpc_input_mutex_, ttmpc_output_mutex_;
  bool ttmpc_thread_enabled_{false};

  std::unique_ptr<ttmpc::MPC> ttmpc_;
  std::unique_ptr<ttmpc::SelCollNNmodel> selcolNN_;
  std::unique_ptr<ttmpc::EnvCollNNmodel> envcolNN_;

  ttmpc::PathToJson json_paths_;
  double Ts_ttmpc_;
  unsigned int time_idx_ttmpc_{0};

  bool is_reached_{false};

  bool is_ttmpc_solved_{false};
  Eigen::Vector3d obs_posi_;
  double obs_radi_ = sqrt(pow(0.16/2., 2) + pow(0.195/2., 2) + pow(0.195/2., 2))*100.;
  Eigen::Matrix<double, 7, 1> ttmpc_qdot_desired_;
  ttmpc::ArcLengthSpline spline_track_;
  double pred_sel_min_dist_{0.};
  double pred_env_min_dist_{0.};
  double mani_;
  double contour_error_{0.};

  ros::Publisher ttmpc_ref_path_pub_;
  ros::Publisher ttmpc_opt_traj_pub_;
  ros::Publisher ttmpc_ee_pose_pub_;
  ros::Publisher ttmpc_ee_speed_pub_;
  ros::Publisher ttmpc_mani_pub_;
  ros::Publisher ttmpc_selcol_pub_;
  ros::Publisher ttmpc_envcol_pub_;
  ros::Publisher ttmpc_Ec_pub_;

  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  std::ofstream joint_info_file_, s_info_file_, ee_vel_info_file_, 
                min_dist_info_file_, mani_info_file_, contour_error_info_file_, 
                ttmpc_ref_path_info_file_, ttmpc_opt_traj_info_file_, 
                ttmpc_comp_time_info_file_, obs_info_file_;

  void asyncTTMPCProc();
  void Rot2Quat(const Eigen::Matrix<double, 3, 3> &rot, geometry_msgs::Pose &pose);
  // ======================================

  void printState();
  void moveJointPosition(const Eigen::Matrix<double, 7, 1> & target_q, double duration);
  int kbhit(void);

  void setMode(const CTRL_MODE & mode);
  void getCurrentState();
  void setDesiredJointVel(const Eigen::Matrix<double, 7, 1> & desired_qdot);

  void modeChangeReaderProc();
  void asyncCalculationProc();
  void StatePubProc();

  Eigen::MatrixXd LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq);
  bool getTransform(const std::string& parent_frame, const std::string& child_frame, geometry_msgs::TransformStamped& transformStamped);
};

}  // namespace advanced_robotics_franka_controllers

#endif