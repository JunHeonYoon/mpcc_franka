#include "mpcc_franka/mpcc_controller.h"

namespace mpcc_franka
{
// ---------------------------default controller function-----------------------------------------
bool mpcc_controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "mpcc_controller: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("mpcc_controller: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "mpcc_controller: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("mpcc_controller: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "mpcc_controller: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("mpcc_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "mpcc_controller: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR_STREAM("mpcc_controller: Error getting velocity joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("mpcc_controller: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  mode_change_thread_ = std::thread(&mpcc_controller::modeChangeReaderProc, this);
  control_mode_pub_ = node_handle.advertise<std_msgs::Float64>("/franka/control_mode", 1);

  // gripper_ac_homing_.waitForServer();  
  // gripper_ac_homing_.sendGoal(franka_gripper::HomingGoal());
  // ================ MPCC ================
  mpcc_ref_path_pub_ = node_handle.advertise<nav_msgs::Path>("/mpcc/ref_path", 1);
  mpcc_opt_traj_pub_ = node_handle.advertise<nav_msgs::Path>("/mpcc/opt_traj", 1);
  mpcc_ee_pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("/mpcc/ee_pose", 1);
  mpcc_ee_speed_pub_ = node_handle.advertise<std_msgs::Float64>("/mpcc/ee_speed", 1);
  mpcc_mani_pub_ = node_handle.advertise<std_msgs::Float64>("/mpcc/manipulability", 1);
  mpcc_selcol_pub_ = node_handle.advertise<std_msgs::Float64>("/mpcc/sel_min_dist", 1);
  mpcc_envcol_pub_ = node_handle.advertise<std_msgs::Float64>("/mpcc/env_min_dist", 1);
  mpcc_Ec_pub_ = node_handle.advertise<std_msgs::Float64>("/mpcc/contour_error", 1);

  tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);

  std::ifstream iConfig(mpcc::pkg_path + "Params/config.json");
  mpcc::json jsonConfig;
  iConfig >> jsonConfig;

  json_paths_ =  {mpcc::pkg_path + std::string(jsonConfig["model_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["cost_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["bounds_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["track_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["normalization_path"]),
                  mpcc::pkg_path + std::string(jsonConfig["sqp_path"])};
  Ts_mpcc_ = jsonConfig["Ts"];
  mpcc_ = std::make_unique<mpcc::MPC>(Ts_mpcc_, json_paths_);
  selcolNN_ = make_unique<mpcc::SelCollNNmodel>();
  envcolNN_ = make_unique<mpcc::EnvCollNNmodel>();
  selcolNN_->setNeuralNetwork(mpcc::PANDA_DOF, 1,(Eigen::VectorXd(2) << 256, 64).finished(), true);
  envcolNN_->setNeuralNetwork(mpcc::PANDA_DOF+3 ,PANDA_NUM_LINKS,(Eigen::VectorXd(4) << 256, 256, 256, 256).finished(), true);
  mpcc_trigger_ = franka_hw::TriggerRate(1./Ts_mpcc_);
  async_mpcc_thread_ = std::thread(&mpcc_controller::asyncMPCCProc, this);

  s_info_.setZero();
  obs_posi_.setZero();

  joint_info_file_.open(mpcc::pkg_path + "result_data/joint_info.txt");
  s_info_file_.open(mpcc::pkg_path + "result_data/s_info.txt");
  ee_vel_info_file_.open(mpcc::pkg_path + "result_data/ee_vel_info.txt");
  min_dist_info_file_.open(mpcc::pkg_path + "result_data/min_dist_info.txt");
  mani_info_file_.open(mpcc::pkg_path + "result_data/mani_info.txt");
  contour_error_info_file_.open(mpcc::pkg_path + "result_data/contour_error_info.txt"); 
  mpcc_ref_path_info_file_.open(mpcc::pkg_path + "result_data/mpcc_ref_path_info.txt");
  mpcc_opt_traj_info_file_.open(mpcc::pkg_path + "result_data/mpcc_opt_traj_info.txt"); 
  mpcc_comp_time_info_file_.open(mpcc::pkg_path + "result_data/mpcc_comp_time_info.txt");
  obs_info_file_.open(mpcc::pkg_path + "result_data/obs_info.txt");

  joint_info_file_ << "time(0)[sec], mode(1), q(2-8)[rad], qdot(9-15)[rad/s]" << std::endl;
  s_info_file_ << "time(0)[sec], mode(1), s(2)[m], vs(3)[m/s], dVs(4)[m/s^2]" << std::endl;
  ee_vel_info_file_ << "time(0)[sec], mode(1), v(2-4)[m/s], w(5-7)[rad/s]" << std::endl;
  min_dist_info_file_ << "time(0)[sec], mode(1), self(2)[m], env(3)[m]" << std::endl;
  mani_info_file_ << "time(0)[sec], mode(1), manipulability(2)" << std::endl;
  contour_error_info_file_ << "time(0)[sec], mode(1), contouring error(2)[m], heading error(3)[rad]" << std::endl;
  mpcc_ref_path_info_file_ << "mode(0), position(1-3)[m], quaternion(4-7)" << std::endl;
  mpcc_opt_traj_info_file_ << "time(0)[sec], mode(1), N=0(2-8), ..., N=10(72-78)" << std::endl;
  mpcc_comp_time_info_file_ << "time(0)[sec], mode(1), total(2)[sec], set_self(3)[sec], set_env(4)[sec], set_qp(5)[sec], solve_qp(6)[sec], get_alpha(7)[sec]" << std::endl;
  obs_info_file_ << "time(0)[sec], mode(1), radius(2)[m], position(3-5)[m]" << std::endl;
  // ======================================
  state_pub_thread_ = std::thread(&mpcc_controller::StatePubProc, this);
  return true;
}

void mpcc_controller::starting(const ros::Time& time) 
{
  start_time_ = time;
  play_time_ = time;
  control_start_time_ = time;

  for (size_t i=0; i<7; ++i) 
  {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
  }
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  

  q_init_ = q_;
  qdot_init_ = qdot_;
  q_desired_ = q_;
  qdot_desired_ = qdot_;
  x_ = transform_.translation();
  rotation_ = transform_.rotation();
  transform_init_ = transform_;
  x_init_ = x_;
  rotation_init_ = rotation_;
  x_dot_desired_.setZero();

  is_initialized_ = true;

}

void mpcc_controller::update(const ros::Time& time, const ros::Duration& period) 
{

  mpcc_controller::getCurrentState(); // compute q(dot), dynamic, jacobian, EE pose(velocity)

  play_time_ += period;
  if(calculation_mutex_.try_lock())
  {
      calculation_mutex_.unlock();
      if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
      async_calculation_thread_ = std::thread(&mpcc_controller::asyncCalculationProc, this);
  }
  ros::Rate r(30000);
  for(size_t i=0; i<9; ++i)
  {
      r.sleep();
      if(calculation_mutex_.try_lock())
      {
          calculation_mutex_.unlock();
          if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
          break;
      }
  }

  mpcc_controller::printState();
  mpcc_controller::setDesiredJointVel(qdot_desired_);
}

void mpcc_controller::stopping(const ros::Time & /*time*/)
{
  ROS_INFO("mpcc_controller::stopping");
}
// ------------------------------------------------------------------------------------------------

// --------------------------- funciotn from robotics class -----------------------------------------
void mpcc_controller::printState()
{
  if (print_rate_trigger_()) 
    {
    std::cout << "-------------------------------------------------------------------" << std::endl;
    std::cout << "MODE          : " << control_mode_ << std::endl;
    std::cout << "time          : " << std::fixed << std::setprecision(3) << play_time_.toSec() << std::endl;
		std::cout << "q now         :\t";
		std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
		std::cout << "q desired     :\t";
		std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
    std::cout << "qdot now      :\t";
		std::cout << std::fixed << std::setprecision(3) << qdot_.transpose() << std::endl;
    std::cout << "qdot desired  :\t";
		std::cout << std::fixed << std::setprecision(3) << qdot_desired_.transpose() << std::endl;
		std::cout << "x             :\t";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R             :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << rotation_ << std::endl;
    std::cout << "J             :\t" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << j_ << std::endl;
    std::cout << "Mani          : " << std::fixed << std::setprecision(3) << mani_ << std::endl;
    std::cout << "self min dist :\t";
    std::cout << pred_sel_min_dist_ << std::endl;
    std::cout << "env min dist  :\t";
    std::cout << pred_env_min_dist_ << std::endl;
    if(mpcc_thread_enabled_)
    {
      std::cout << "s           :" << std::fixed << std::setprecision(3) << s_info_.s << std::endl;
      std::cout << "vs          :" << std::fixed << std::setprecision(3) << s_info_.vs << std::endl;
      std::cout << "dVs         :" << std::fixed << std::setprecision(3) << s_info_.dVs << std::endl;
      std::cout << "Ec          :" << std::fixed << std::setprecision(3) << contour_error_ << std::endl;
    }
    std::cout << "-------------------------------------------------------------------\n\n" << std::endl;
  }
}

void mpcc_controller::moveJointPosition(const Eigen::Matrix<double, 7, 1> &target_q, double duration)
{
  for(size_t i=0; i<7;i++)
  {
    q_desired_(i) = DyrosMath::cubic(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
    qdot_desired_(i) = DyrosMath::cubicDot(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + duration,
                                        q_init_(i), target_q(i), 0, 0);
  }
}

int mpcc_controller::kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}
// --------------------------- Controller Core Methods -----------------------------------------
void mpcc_controller::setMode(const CTRL_MODE & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  std_msgs::Float64 ctrl_mode_msg;
  ctrl_mode_msg.data = mode;
  control_mode_pub_.publish(ctrl_mode_msg);
  std::cout << "Current mode (changed): " << mode << std::endl;
}

void mpcc_controller::getCurrentState()
{
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  // const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  // const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();

  // for velocity control, these code did not work!!!
  // q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
  // qdot_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
  // torque_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
  // g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity_array.data());
  // m_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
  // m_inv_ = m_.inverse();
  // c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());

  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    for (size_t i=0; i<7; ++i) 
    {
      q_(i) = joint_handles_[i].getPosition();
      qdot_(i) = joint_handles_[i].getVelocity();
    }

    j_ = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
    j_v_ = j_.block<3, 7>(0, 0);
    j_w_ = j_.block<3, 7>(3, 0);
    transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
    x_ = transform_.translation();
    rotation_ = transform_.rotation();
    x_dot_ = j_ * qdot_;

    // ============== MPCC ==============
    mani_ = sqrt((j_*j_.transpose()).determinant());
    if(mpcc_thread_enabled_)
    {
      s_info_.s += s_info_.vs / hz_;
      s_info_.vs += s_info_.dVs / hz_;
    }
    // ==================================
  }
}

void mpcc_controller::setDesiredJointVel(const Eigen::Matrix<double, 7, 1> & desired_qdot)
{
    Eigen::Matrix<double, 7, 1> lpf_command = LowPassFilter(desired_qdot, qdot_, hz_, 20.0);
    for (size_t i = 0; i < 7; ++i) 
    {
      joint_handles_[i].setCommand(lpf_command(i));
      // joint_handles_[i].setCommand(desired_qdot(i));
    }
}

void mpcc_controller::asyncCalculationProc()
  {
    // bench_timer_.reset();
    {
      std::lock_guard<std::mutex> lock(calculation_mutex_);
      if(is_mode_changed_)
      {
        is_mode_changed_ = false;
        control_start_time_ = play_time_;
        q_init_ = q_;
        qdot_init_ = qdot_;
        q_desired_ = q_init_;
        x_dot_init_ = x_dot_;
        qdot_desired_ = qdot_init_;
        x_dot_desired_ = x_dot_init_;
        x_init_ = x_;
        rotation_init_ = rotation_;
        transform_init_ = transform_;
        mpcc_thread_enabled_ = false;

        if(control_mode_ == HOME)
        {
          std::cout << "======================== Mode cahnge: Home position ========================" << std::endl;
        }
        else if(control_mode_ == MPCC)
        {
          std::cout << "======================== Mode cahnge: MPCC ========================" << std::endl;
          is_reached_ = false;
          s_info_.setZero();
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0.;

          mpcc::Track track = mpcc::Track(json_paths_.track_path);
          mpcc::TrackPos track_xyzr = track.getTrack(transform_init_.matrix());
          mpcc_->setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);
          mpcc_thread_enabled_ = true;

          spline_track_ = mpcc_->getTrack();
          mpcc::PathData spline_path = spline_track_.getPathData();
          nav_msgs::Path mpcc_ref_path;
          mpcc_ref_path.header.frame_id = "panda_link0";
          for(size_t i=0; i<spline_path.n_points; i++)
          {
            geometry_msgs::PoseStamped path_point;
            path_point.pose.position.x = spline_path.X(i);
            path_point.pose.position.y = spline_path.Y(i);
            path_point.pose.position.z = spline_path.Z(i);
            Rot2Quat(spline_path.R[i], path_point.pose);
            path_point.header.frame_id = "panda_link0";

            mpcc_ref_path.poses.push_back(path_point);

            // logging data
            mpcc_ref_path_info_file_ << control_mode_ << " "
                                     << path_point.pose.position.x << " "
                                     << path_point.pose.position.y << " "
                                     << path_point.pose.position.z << " "
                                     << path_point.pose.orientation.x << " "
                                     << path_point.pose.orientation.y << " "
                                     << path_point.pose.orientation.z << " "
                                     << path_point.pose.orientation.w << std::endl;
          }
          mpcc_ref_path_pub_.publish(mpcc_ref_path);
        }
        else if(control_mode_ == READY)
        {
          std::cout << "======================== Mode cahnge: ready position ========================" << std::endl;
        }
        else if(control_mode_ == MPCC_PICK)
        {
          std::cout << "======================== Mode cahnge: MPCC Pick ========================" << std::endl;
          is_reached_ = false;
          s_info_.setZero();
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0.;

          geometry_msgs::TransformStamped transformStamped;
          if(getTransform("panda_link0", "object_frame", transformStamped))
          {
            Eigen::Vector3d waypoint_x, waypoint_y, waypoint_z;
            std::vector<Eigen::Matrix3d> waypoint_r;
            waypoint_x << x_init_(0), x_init_(0),                               transformStamped.transform.translation.x - 0.059 - 0.021;
            waypoint_y << x_init_(1), transformStamped.transform.translation.y, transformStamped.transform.translation.y;
            waypoint_z << x_init_(2), x_init_(2),                               transformStamped.transform.translation.z + 0.07;

            waypoint_r.resize(waypoint_x.size());
            waypoint_r[0] = rotation_init_;
            waypoint_r[1] << 1.,  0.,  0., 
                             0., -1.,  0.,
                             0.,  0., -1.;
            waypoint_r[2] << 1.,  0.,  0., 
                             0., -1.,  0.,
                             0.,  0., -1.;
            std::cout << "set waypoint!" << std::endl;
            mpcc::Track track = mpcc::Track(json_paths_.track_path);
            track.setTrack(waypoint_x, waypoint_y, waypoint_z, waypoint_r);
            mpcc::TrackPos track_xyzr = track.getTrack(transform_init_.matrix());
            mpcc_->setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);
            std::cout << "Set track!" << std::endl;
            mpcc_thread_enabled_ = true;

            spline_track_ = mpcc_->getTrack();
            mpcc::PathData spline_path = spline_track_.getPathData();
            nav_msgs::Path mpcc_ref_path;
            mpcc_ref_path.header.frame_id = "panda_link0";
            for(size_t i=0; i<spline_path.n_points; i++)
            {
              geometry_msgs::PoseStamped path_point;
              path_point.pose.position.x = spline_path.X(i);
              path_point.pose.position.y = spline_path.Y(i);
              path_point.pose.position.z = spline_path.Z(i);
              Rot2Quat(spline_path.R[i], path_point.pose);
              path_point.header.frame_id = "panda_link0";

              mpcc_ref_path.poses.push_back(path_point);

              // logging data
              mpcc_ref_path_info_file_ << path_point.pose.position.x << " "
                                      << path_point.pose.position.y << " "
                                      << path_point.pose.position.z << " "
                                      << path_point.pose.orientation.x << " "
                                      << path_point.pose.orientation.y << " "
                                      << path_point.pose.orientation.z << " "
                                      << path_point.pose.orientation.w << std::endl;
            }
            mpcc_ref_path_pub_.publish(mpcc_ref_path);
          } 
          else
          {
            ROS_INFO("Object can not be found!!");
          }
        }
        else if(control_mode_ == MPCC_DROP)
        {
          std::cout << "======================== Mode cahnge: MPCC Drop ========================" << std::endl;
          is_reached_ = false;
          s_info_.setZero();
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0.;

          Eigen::Vector4d waypoint_x, waypoint_y, waypoint_z;
          std::vector<Eigen::Matrix3d> waypoint_r;
          waypoint_x << 0., 0.0, -0.15, -0.15;
          waypoint_y << 0., 0.0,  0.15,  0.17;
          waypoint_z << 0., 0.1,  0.1,  0.18;

          waypoint_r.resize(waypoint_x.size());
          for(size_t i=0; i<waypoint_r.size(); i++)
          {
            if(i == waypoint_r.size()-1)
            {
              waypoint_r[i] << 1.0, 0.0,            0.0,
                               0.0, cos(M_PI*4/3), -sin(M_PI*4/3),
                               0.0, sin(M_PI*4/3),  cos(M_PI*4/3);
            }
            else
            {
              waypoint_r[i].setIdentity();
            }
          }
          std::cout << "set waypoint!" << std::endl;
          mpcc::Track track = mpcc::Track(json_paths_.track_path);
          track.setTrack(waypoint_x, waypoint_y, waypoint_z, waypoint_r);
          mpcc::TrackPos track_xyzr = track.getTrack(transform_init_.matrix());
          mpcc_->setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);
          std::cout << "Set track!" << std::endl;
          mpcc_thread_enabled_ = true;

          spline_track_ = mpcc_->getTrack();
          mpcc::PathData spline_path = spline_track_.getPathData();
          nav_msgs::Path mpcc_ref_path;
          mpcc_ref_path.header.frame_id = "panda_link0";
          for(size_t i=0; i<spline_path.n_points; i++)
          {
            geometry_msgs::PoseStamped path_point;
            path_point.pose.position.x = spline_path.X(i);
            path_point.pose.position.y = spline_path.Y(i);
            path_point.pose.position.z = spline_path.Z(i);
            Rot2Quat(spline_path.R[i], path_point.pose);
            path_point.header.frame_id = "panda_link0";

            mpcc_ref_path.poses.push_back(path_point);

            // logging data
            mpcc_ref_path_info_file_ << path_point.pose.position.x << " "
                                    << path_point.pose.position.y << " "
                                    << path_point.pose.position.z << " "
                                    << path_point.pose.orientation.x << " "
                                    << path_point.pose.orientation.y << " "
                                    << path_point.pose.orientation.z << " "
                                    << path_point.pose.orientation.w << std::endl;
          }
          mpcc_ref_path_pub_.publish(mpcc_ref_path);
        }
        else if(control_mode_ == MPCC_PLACE)
        {
          std::cout << "======================== Mode cahnge: MPCC Place ========================" << std::endl;
          is_reached_ = false;
          s_info_.setZero();
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0.;

          Eigen::Vector4d waypoint_x, waypoint_y, waypoint_z;
          std::vector<Eigen::Matrix3d> waypoint_r;
          waypoint_x << 0., 0.,  0.15,  0.15;
          waypoint_y << 0., -0.02, -0.17, -0.17;
          waypoint_z << 0., -0.08,  -0.08, -0.18;

          waypoint_r.resize(waypoint_x.size());
          for(size_t i=0; i<waypoint_r.size(); i++)
          {
            if(i == 0)
            {
              waypoint_r[i].setIdentity();
            }
            else
            {
              waypoint_r[i] << 1.0, 0.0,            0.0,
                               0.0, cos(-M_PI*4/3), -sin(-M_PI*4/3),
                               0.0, sin(-M_PI*4/3),  cos(-M_PI*4/3);
            }
          }
          std::cout << "set waypoint!" << std::endl;
          mpcc::Track track = mpcc::Track(json_paths_.track_path);
          track.setTrack(waypoint_x, waypoint_y, waypoint_z, waypoint_r);
          mpcc::TrackPos track_xyzr = track.getTrack(transform_init_.matrix());
          mpcc_->setTrack(track_xyzr.X,track_xyzr.Y,track_xyzr.Z,track_xyzr.R);
          std::cout << "Set track!" << std::endl;
          mpcc_thread_enabled_ = true;

          spline_track_ = mpcc_->getTrack();
          mpcc::PathData spline_path = spline_track_.getPathData();
          nav_msgs::Path mpcc_ref_path;
          mpcc_ref_path.header.frame_id = "panda_link0";
          for(size_t i=0; i<spline_path.n_points; i++)
          {
            geometry_msgs::PoseStamped path_point;
            path_point.pose.position.x = spline_path.X(i);
            path_point.pose.position.y = spline_path.Y(i);
            path_point.pose.position.z = spline_path.Z(i);
            Rot2Quat(spline_path.R[i], path_point.pose);
            path_point.header.frame_id = "panda_link0";

            mpcc_ref_path.poses.push_back(path_point);

            // logging data
            mpcc_ref_path_info_file_ << path_point.pose.position.x << " "
                                    << path_point.pose.position.y << " "
                                    << path_point.pose.position.z << " "
                                    << path_point.pose.orientation.x << " "
                                    << path_point.pose.orientation.y << " "
                                    << path_point.pose.orientation.z << " "
                                    << path_point.pose.orientation.w << std::endl;
          }
          mpcc_ref_path_pub_.publish(mpcc_ref_path);
        }
      }

      if(control_mode_ == HOME)
      {
        Eigen::Matrix<double, 7, 1> target_q;
        target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
        mpcc_controller::moveJointPosition(target_q, 4.0);
      }
      else if(control_mode_ == READY)
      {
        Eigen::Matrix<double, 7, 1> target_q;
        // target_q << 0.000, -1.087,  0.026, -2.369, 0.00,  0.990,  M_PI/4;
        target_q << 0.0, -0.824,  0.0, -2.254, 0.0,  0.963,  M_PI/4;
        mpcc_controller::moveJointPosition(target_q, 4.0);
      }
      else if(control_mode_ == MPCC)
      {
        if(is_mpcc_solved_)
        {
          is_mpcc_solved_ = false;
          qdot_desired_ = mpcc_qdot_desired_;
          s_info_.dVs = mpcc_dVs_desired_;
        }
        if(is_reached_)
        {
          qdot_desired_.setZero();
        }
        q_desired_ = q_ + qdot_desired_ / hz_;
      }
      else if(control_mode_ == MPCC_PICK)
      {
        if(is_mpcc_solved_)
        {
          is_mpcc_solved_ = false;
          qdot_desired_ = mpcc_qdot_desired_;
          s_info_.dVs = mpcc_dVs_desired_;
        }
        if(is_reached_)
        {
          qdot_desired_.setZero();
          gripper_ac_close_.waitForServer();  
          franka_gripper::GraspGoal goal;
          goal.speed = 0.1;
          goal.force = 0.001;
          goal.epsilon.inner = 0.06;
          goal.epsilon.outer = 7.;
          gripper_ac_close_.sendGoal(goal);
        }
        q_desired_ = q_ + qdot_desired_ / hz_;
      }
      else if(control_mode_ == MPCC_DROP)
      {
        if(is_mpcc_solved_)
        {
          is_mpcc_solved_ = false;
          qdot_desired_ = mpcc_qdot_desired_;
          s_info_.dVs = mpcc_dVs_desired_;
        }
        if(is_reached_)
        {
          qdot_desired_.setZero();
        }
        q_desired_ = q_ + qdot_desired_ / hz_;
      }
      else if(control_mode_ == MPCC_PLACE)
      {
        if(is_mpcc_solved_)
        {
          is_mpcc_solved_ = false;
          qdot_desired_ = mpcc_qdot_desired_;
          s_info_.dVs = mpcc_dVs_desired_;
        }
        if(is_reached_)
        {
          qdot_desired_.setZero();
          gripper_ac_open_.waitForServer();  
          franka_gripper::MoveGoal goal;
          goal.speed = 0.1;
          goal.width = 0.08;
          gripper_ac_open_.sendGoal(goal);
        }
        q_desired_ = q_ + qdot_desired_ / hz_;
      }
      else
      {
        for(size_t i=0; i<7;i++)
        {
          qdot_desired_(i) = DyrosMath::cubic(play_time_.toSec(), control_start_time_.toSec(), control_start_time_.toSec() + 3.0,
                                              qdot_init_(i), 0.0, 0.0, 0.0);
        }
        q_desired_ = q_ + qdot_desired_ / hz_;
      }
    }
    // double elapsed_time = bench_timer_.elapsedAndReset();
    // if(print_rate_trigger_()) std::cout << "calculation proc freq: " << 1./elapsed_time << std::endl;
  }

void mpcc_controller::modeChangeReaderProc()
{
   while (!quit_all_proc_)
    {
      if(kbhit())
      {
        {
          std::lock_guard<std::mutex> lock(calculation_mutex_);
          int key = getchar();
          switch (key)
          {
            case 'h':
              mpcc_controller::setMode(HOME);
              break;
            case 'm':
              mpcc_controller::setMode(MPCC);
              break;
            case 'r':
              mpcc_controller::setMode(READY);
              break;
            case 'p':
              mpcc_controller::setMode(MPCC_PICK);
              break;
            case 'd':
              mpcc_controller::setMode(MPCC_DROP);
              break;
            case 'l':
              mpcc_controller::setMode(MPCC_PLACE);
              break;
            case 'o':
              {
                gripper_ac_open_.waitForServer();  
                franka_gripper::MoveGoal goal;
                goal.speed = 0.1;
                goal.width = 0.08;
                gripper_ac_open_.sendGoal(goal);
              }
            case 'c':
            {
              gripper_ac_close_.waitForServer();  
              franka_gripper::GraspGoal goal;
              goal.speed = 0.1;
              goal.force = 0.001;
              goal.epsilon.inner = 0.06;
              goal.epsilon.outer = 7.;
              gripper_ac_close_.sendGoal(goal);
            }

            default:
              mpcc_controller::setMode(NONE);
              break;
          }
        }
      }
    }
}

void mpcc_controller::StatePubProc()
{
  while(!quit_all_proc_)
  {
    if(state_pub_trigger_() && is_initialized_)
    {
      // publish ee pose msg
      geometry_msgs::PoseStamped ee_pose;
      ee_pose.header.frame_id = "panda_link0";
      ee_pose.header.stamp = ros::Time::now();
      ee_pose.pose.position.x = x_(0);
      ee_pose.pose.position.y = x_(1);
      ee_pose.pose.position.z = x_(2);
      Rot2Quat(rotation_, ee_pose.pose);
      mpcc_ee_pose_pub_.publish(ee_pose);

      // get self minimum distance
      auto pred_sel_NN = selcolNN_->calculateMlpOutput(q_, false);
      pred_sel_min_dist_ = pred_sel_NN.first.value();

      // get environment minimum distance
      geometry_msgs::TransformStamped transformStamped;
      if(getTransform("panda_link0", "obstacle_frame", transformStamped))
      {
        obs_posi_(0) = transformStamped.transform.translation.x;
        obs_posi_(1) = transformStamped.transform.translation.y;
        obs_posi_(2) = transformStamped.transform.translation.z;
        Eigen::VectorXd env_NN_input(mpcc::PANDA_DOF+obs_posi_.size());
        env_NN_input << q_, obs_posi_;
        auto pred_env_NN = envcolNN_->calculateMlpOutput(env_NN_input, false);
        pred_env_min_dist_ = std::max(std::min(pred_env_NN.first.minCoeff() - obs_radi_, 20.), 1.);
      }
      else
      {
        pred_env_min_dist_ = 20.;
      }

      // get Contouring error
      if((control_mode_ == MPCC || control_mode_ == MPCC_PICK || control_mode_ == MPCC_PLACE || control_mode_ == MPCC_DROP) && mpcc_thread_enabled_)
      {
        Eigen::Vector3d ref_posi = spline_track_.getPosition(s_info_.s);
        Eigen::Matrix3d ref_ori = spline_track_.getOrientation(s_info_.s);
        contour_error_ = (ref_posi - x_).norm();
        heading_error_ = mpcc::getInverseSkewVector(mpcc::LogMatrix(ref_ori.transpose()*rotation_)).norm();
      }
      else
      {
        contour_error_ = 0.;
        heading_error_ = 0.;
      }

      // publish speed of EE, manipulability, self minimum distance, env minimum distance, contouring error
      std_msgs::Float64 ee_speed, mani, sel_min_dist, env_min_dist, contour_error;
      ee_speed.data = x_dot_.norm();
      mani.data = mani_;
      sel_min_dist.data = pred_sel_min_dist_;
      env_min_dist.data = pred_env_min_dist_;
      contour_error.data = contour_error_;

      mpcc_ee_speed_pub_.publish(ee_speed);
      mpcc_mani_pub_.publish(mani);
      mpcc_selcol_pub_.publish(sel_min_dist);
      mpcc_envcol_pub_.publish(env_min_dist);
      mpcc_Ec_pub_.publish(contour_error);

      // logging data
      if((control_mode_ == MPCC || control_mode_ == MPCC_PICK || control_mode_ == MPCC_PLACE || control_mode_ == MPCC_DROP) && mpcc_thread_enabled_)
      {
        joint_info_file_         << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << q_.transpose()     << " " << qdot_.transpose()    << std::endl;
        s_info_file_             << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << s_info_.s          << " " << s_info_.vs            << " " << s_info_.dVs << std::endl;
        ee_vel_info_file_        << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << x_dot_.transpose() << std::endl;
        min_dist_info_file_      << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << pred_sel_min_dist_ << " " << pred_env_min_dist_    << std::endl;
        mani_info_file_          << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << mani_              << std::endl;
        contour_error_info_file_ << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << contour_error_     << " " << heading_error_        << std::endl;
        obs_info_file_           << (ros::Time::now()-control_start_time_).toSec() << " " << control_mode_ << " " << obs_radi_          << " " << obs_posi_.transpose() << std::endl;
      }
    }
  }
}

void mpcc_controller::asyncMPCCProc()
{
  while(!quit_all_proc_)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
    if(mpcc_thread_enabled_ && mpcc_trigger_())
    {
      bench_timer_.reset();
      mpcc::StateVector x0_vec;
      mpcc::InputVector u0_vec;
      {
        std::lock_guard<std::mutex> lock(mpcc_input_mutex_);
        x0_vec << q_, s_info_.s, s_info_.vs;
        u0_vec << qdot_, s_info_.dVs;
        // x0_vec << q_desired_, s_info_.s, s_info_.vs;
        // u0_vec << qdot_desired_, s_info_.dVs;
      }
      mpcc::State x0 = mpcc::vectorToState(x0_vec);
      mpcc::Input u0 = mpcc::vectorToInput(u0_vec);

      // Get the transform between "panda_link0" and "obstacle_frame"
      geometry_msgs::TransformStamped transformStamped;
      if(getTransform("panda_link0", "obstacle_frame", transformStamped))
      {
        obs_posi_(0) = transformStamped.transform.translation.x;
        obs_posi_(1) = transformStamped.transform.translation.y;
        obs_posi_(2) = transformStamped.transform.translation.z;
      }
      else
      {
        obs_posi_ << 5., 5., 5.;
      }

      mpcc_->updateS(x0);
      {
        std::lock_guard<std::mutex> lock(mpcc_output_mutex_);
        s_info_.s = x0.s;
        s_info_.vs = x0.vs;
      }
      if(mpcc_->checkIsEnd(x0))
      {
        std::lock_guard<std::mutex> lock(mpcc_output_mutex_);
        std::cout << "MPCC reached its end path!!" << std::endl;
        is_reached_ = true;
        mpcc_thread_enabled_ = false;
        qdot_desired_.setZero();
        continue;
      }
      else
      {


      mpcc::MPCReturn mpc_sol;
      bool mpc_status = mpcc_->runMPC_(mpc_sol, x0, u0, obs_posi_, obs_radi_);
      if(mpc_status == false)
      {
          ROS_WARN("MPC did not solved properly!!");
          mpcc_qdot_desired_.setZero();
          mpcc_dVs_desired_ = 0.;
          is_mpcc_solved_ = true;
      }
      else
      {
        {
          std::lock_guard<std::mutex> lock(mpcc_output_mutex_);
          // s_info_.s = x0.s;
          // s_info_.vs = x0.vs;
          mpcc_qdot_desired_ = mpcc::inputTodJointVector(mpc_sol.u0);
          mpcc_dVs_desired_ = mpc_sol.u0.dVs;
        }
        is_mpcc_solved_ = true;

        // publish optimal trajectory from MPCC
        nav_msgs::Path mpcc_opt_traj;
        mpcc_opt_traj.header.frame_id = "panda_link0";
        for(size_t i=0; i<mpc_sol.mpc_horizon.size(); i++)
        {
          Eigen::Matrix<double, 3, 1> xk;
          Eigen::Matrix<double, 3, 3> rk;
          xk = mpcc_->robot_->getEEPosition(mpcc::stateToJointVector(mpc_sol.mpc_horizon[i].xk));
          rk = mpcc_->robot_->getEEOrientation(mpcc::stateToJointVector(mpc_sol.mpc_horizon[i].xk));

          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = xk(0);
          pose.pose.position.y = xk(1);
          pose.pose.position.z = xk(2);
          Rot2Quat(rk, pose.pose);
          pose.header.frame_id = "panda_link0";

          mpcc_opt_traj.poses.push_back(pose);

          // logging data
          mpcc_opt_traj_info_file_ << (ros::Time::now()-control_start_time_).toSec() << " "
                                   << control_mode_ << " "
                                   << pose.pose.position.x << " "
                                   << pose.pose.position.y << " "
                                   << pose.pose.position.z << " "
                                   << pose.pose.orientation.x << " "
                                   << pose.pose.orientation.y << " "
                                   << pose.pose.orientation.z << " "
                                   << pose.pose.orientation.w;
        }
        mpcc_opt_traj_pub_.publish(mpcc_opt_traj);
        mpcc_opt_traj_info_file_ << "\n";

        double elapsed_time = bench_timer_.elapsedAndReset();
        if(print_rate_trigger_())
        {
          std::cout << "===============================================" << std::endl;
          std::cout << "mpcc proc freq[hz]: " << 1./elapsed_time << std::endl;
          std::cout << "mpcc total    [ms]: " << mpc_sol.compute_time.total*1000 << std::endl;
          std::cout << "mpcc set qp   [ms]: " << mpc_sol.compute_time.set_qp*1000 << std::endl;
          std::cout << "mpcc set env  [ms]: " << mpc_sol.compute_time.set_env*1000 << std::endl;
          std::cout << "mpcc get alpha[ms]: " << mpc_sol.compute_time.get_alpha*1000 << std::endl;
          std::cout << "mpcc solve qp [ms]: " << mpc_sol.compute_time.solve_qp*1000 << std::endl;
          std::cout << "===============================================" << std::endl;
        }

        // logging data
        mpcc_comp_time_info_file_ << (ros::Time::now()-control_start_time_).toSec() << " "
                                  << control_mode_ << " "
                                  << mpc_sol.compute_time.total << " "
                                  << mpc_sol.compute_time.set_self << " "
                                  << mpc_sol.compute_time.set_env << " "
                                  << mpc_sol.compute_time.set_qp << " "
                                  << mpc_sol.compute_time.solve_qp << " "
                                  << mpc_sol.compute_time.get_alpha << std::endl;
      }
    }
    }
  }
}

Eigen::MatrixXd mpcc_controller::LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq)
{

  double rc = 1. / (cutoff_freq * 2 * M_PI);
  double dt = 1. / sampling_freq;
  double a = dt / (rc + dt);
  return prev_res + a * (input - prev_res);
}

void mpcc_controller::Rot2Quat(const Eigen::Matrix<double, 3, 3> &rot, geometry_msgs::Pose &pose)
{
  // Convert the Eigen rotation matrix to a tf2::Matrix3x3
  tf2::Matrix3x3 tf2_matrix(
      rot(0, 0), rot(0, 1), rot(0, 2),
      rot(1, 0), rot(1, 1), rot(1, 2),
      rot(2, 0), rot(2, 1), rot(2, 2)
  );

  // Convert the tf2::Matrix3x3 to a tf2::Quaternion
  tf2::Quaternion tf2_quaternion;
  tf2_matrix.getRotation(tf2_quaternion);

  // Convert the tf2::Quaternion to a geometry_msgs::Quaternion
  tf2::convert(tf2_quaternion, pose.orientation);
}

bool mpcc_controller::getTransform(const std::string& parent_frame, const std::string& child_frame, geometry_msgs::TransformStamped& transformStamped)
{
  if (tfBuffer_.canTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(1./(hz_*10.))))
  {
      try
      {
          // Retrieve the transform
          transformStamped = tfBuffer_.lookupTransform(parent_frame, child_frame, ros::Time(0));
          if(transformStamped.header.stamp != ros::Time(0)) return true;
          else return false;
      }
      catch (tf2::TransformException &ex)
      {
          ROS_WARN("Failed to get transform: %s", ex.what());
          return false;
      }
  }
  else
  {
      // Do not log warnings if the transform is not available
      return false;
  }
}
// ------------------------------------------------------------------------------------------------




} // namespace mpcc_franka



PLUGINLIB_EXPORT_CLASS(mpcc_franka::mpcc_controller,
                       controller_interface::ControllerBase)