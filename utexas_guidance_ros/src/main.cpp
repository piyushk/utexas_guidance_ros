
utexas_guidance::State system_state_;
YAML::Node robot_start_locations_;

std::vector<geometry_msgs::Pose> robot_location_;
std::vector<geometry_msgs::Pose> previous_robot_location_;
std::vector<std::string, boost::mutex> robot_location_mutex_;
std::vector<geometry_msgs::Pose> robot_location_;

std::map<std::string, geometry_msgs::Pose> request_location_;
std::map<std::string, geometry_msgs::Pose> previous_request_location_;
std::map<std::string, boost::mutex> request_location_mutex_;

boost::mutex system_state_mutex_;

void humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose,
                          std::string namespace) {
  human_location_ = human_pose->pose.pose;
}

void requestServiceHandler(const utexas_guidance_msgs::RequestMsgtamped::ConstPtr &robot_pose,
                           int robot_idx) {
  // Add a request to the current system state. When the controller thread updates next, this should get pulled in.
  sys
  boost::mutex::scoped_lock lock(*(robot_location_mutex_[robot_idx]));
  robot_location_[robot_idx] = robot_pose->pose.pose;
  robot_location_available_[robot_idx] = true;

  publishSystemState();
}

void updateRobotState(utexas_guidance::RobotState& robot,
                      const geometry_msgs

void updateRequestState
utexas_guidance_ros::StateMsg toStateMsg(const utexas_guidance::State& state) {

  // If a person is really close to a robot (< 2m), map the person to the robot's location.
}

void publishSystemState() {
  system_state_publisher_(toStateMsg(system_state_));
}

void updateRobotControl() {
  // POSSIBLE_ROBOT_STATUS
  //
  // MOVE_AND_NO_WAIT
  // MOVE_AND_WAIT
  //
  // If a lead action just completed, release the robot.

}

void sendRobotToDestination(int robot_id) {

}

void rotateRobotToFaceNearestHuman(int nearest_human) {

}

void setupRobotCallbacks() {
  for (int i = 0; i < num_robots; ++i) {
    const std::string &robot_name = available_robot_list_[i];

    // Add a controller for this robot.
    ROS_INFO_STREAM_NAMED("BaseRobotNavigator", "Waiting for action server for robot: " << robot_name);
    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > ac;
    ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/" + robot_name + "/move_base_interruptable", true));
    ac->waitForServer();
    robot_controller_.push_back(ac);

    boost::shared_ptr<ros::Subscriber> loc_sub(new ros::Subscriber);
    *loc_sub = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_name + "/amcl_pose", 1,
                                                                        boost::bind(&BaseRobotNavigator::robotLocationHandler,
                                                                                    this, _1, i));
    robot_location_subscriber_.push_back(loc_sub);

    boost::shared_ptr<ros::ServiceClient> gui_client(new ros::ServiceClient);
    *gui_client = nh_->serviceClient<bwi_guidance_msgs::UpdateGuidanceGui>("/" + robot_name + "/update_gui");
    robot_gui_controller_.push_back(gui_client);

    // Finally get a new task for this robot and setup the system state for this robot.
    RobotState rs;
    // rs.loc will be setup once the subscriber kicks in. set the loc to -1 for now to indicate the location has
    // not been received.
    rs.loc_u = -1;
    rs.loc_v = -1;
    rs.loc_p = 0.0f;

    rs.tau_d = -1; // Get Starting task.
    getNextTaskForRobot(i, rs);

    rs.help_destination = NONE;

    system_state_.robots.push_back(rs);
  }
}

int main(int argc, char *argv[]) {

  /* Read in the same file read by the multi-robot launcher to know the start location of all the robots. */
  // Read the parameters from file and initialize the RNG.
  std::string robot_locations_file;
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("robot_locations_file", robot_locations_file)) {
    ROS_FATAL_STREAM("RobotPosition: ~robot_locations_file parameter required");
    exit(-1);
  }
  robot_start_locations = YAML::LoadFile(robot_locations_file);
  
  /* Setup subcribers and controllers for all robots, and wait for their locations to be tracked. */

  
  return 0;
}
