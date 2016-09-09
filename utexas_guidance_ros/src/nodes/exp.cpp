#include <utexas_guidance_ros/robot_navigator.h>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bwi_msgs/QuestionDialog.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <utexas_guidance_msgs/MultiRobotNavigationAction.h>

#include <utexas_planning/common/record_writer.h>

using namespace utexas_guidance_ros;

bool waiting_for_robots = true;
int num_total_robots = 1000;
std::vector<std::string> available_robot_list;
std::vector<geometry_msgs::Pose> robot_start_locations;
std::string solver_alias;

std::vector<geometry_msgs::Pose> problem_start_locations;
std::vector<int> problem_goal_idx;

geometry_msgs::PoseWithCovarianceStamped getStartPoseWithCovariance(int robot_idx) {
  geometry_msgs::PoseWithCovarianceStamped retval;
  retval.header.frame_id = available_robot_list[robot_idx] + "/level_mux/map";
  retval.header.stamp = ros::Time::now();
  retval.pose.pose = robot_start_locations[robot_idx];
  for (int i = 0; i < 36; ++i) {
    retval.pose.covariance[0] = 0.0f;
  }
  retval.pose.covariance[0] = 0.1;
  retval.pose.covariance[7] = 0.1f;
  retval.pose.covariance[35] = 0.25f;   
  return retval;
}

geometry_msgs::Pose getPoseBehindRobot(int robot_idx) {
  geometry_msgs::Pose retval = robot_start_locations[robot_idx];
  float yaw = float(tf::getYaw(retval.orientation));
  retval.position.x -= 0.7f * cosf(yaw);
  retval.position.z -= 0.7f * sinf(yaw);
  return retval;
}

geometry_msgs::Pose getPose(float x, float y, float yaw) {
  geometry_msgs::Pose retval;

  retval.position.x = x;
  retval.position.y = y;
  retval.position.z = 0.0f;
  retval.orientation = tf::createQuaternionMsgFromYaw(yaw);

  return retval;
}

void displayMessageWithPause(const std::string& msg,
                    ros::ServiceClient& gui_service) {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  srv.request.message = msg;
  srv.request.options.resize(1);
  srv.request.options[0] = "Continue!";
  srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT;
  gui_service.call(srv);
}

void displayMessage(const std::string& msg,
                    ros::ServiceClient& gui_service) {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialogRequest::DISPLAY;
  srv.request.message = msg;
  gui_service.call(srv);
}

bool checkClosePoses(const geometry_msgs::Pose& p1, 
                     const geometry_msgs::Pose& p2, 
                     float threshold = 0.05f, 
                     bool check_yaw = true) {

  float dist_diff = sqrtf(pow((p1.position.x - p2.position.x), 2) + pow((p1.position.y - p2.position.y), 2));
  if (dist_diff > threshold) {
    return false;
  }
  double yaw1 = tf::getYaw(p1.orientation);
  double yaw2 = tf::getYaw(p2.orientation);
  if (check_yaw && fabs(yaw1 - yaw2) > 0.1) {
    return false;
  }

  return true;
}

bool teleportEntity(const std::string& entity,
                    const geometry_msgs::Pose& pose,
                    ros::ServiceClient& get_gazebo_model_client,
                    ros::ServiceClient& set_gazebo_model_client) {

  int count = 0;
  int attempts = 5;
  bool location_verified = false;
  while (count < attempts and !location_verified) {
    gazebo_msgs::GetModelState get_srv;
    get_srv.request.model_name = entity;
    get_gazebo_model_client.call(get_srv);
    location_verified = checkClosePoses(get_srv.response.pose, pose);
    if (!location_verified) {
      gazebo_msgs::SetModelState set_srv;
      set_srv.request.model_state.model_name = entity;
      set_srv.request.model_state.pose = pose;
      set_gazebo_model_client.call(set_srv);
      if (!set_srv.response.success) {
        ROS_WARN_STREAM("SetModelState service call failed for " << entity << " to " << pose);
      }
    }
    ++count;
  }

  if (!location_verified) {
    ROS_ERROR_STREAM("Unable to teleport " << entity << " to " << pose
                     << " despite " << attempts << " attempts.");
    return false;
  }

  return true;
}

void readExperimentFile(const std::string& experiment_file) {
  YAML::Node config = YAML::LoadFile(experiment_file);
  solver_alias = config["solver_alias"].as<std::string>();
  YAML::Node problems = config["problems"];
  problem_start_locations.resize(problems.size());
  problem_goal_idx.resize(problems.size());
  for (std::size_t i = 0; i < problems.size(); i++) {
    problem_start_locations[i] = getPoseBehindRobot(problems[i]["start_robot_idx"].as<int>());
    problem_goal_idx[i] = problems[i]["goal_idx"].as<int>();
  }
}

void readInitialConfigurationFile(const std::string& initial_config_file) {
  YAML::Node config = YAML::LoadFile(initial_config_file);
  YAML::Node robots = config["robots"];
  num_total_robots = robots.size();
  robot_start_locations.resize(num_total_robots);
  for (std::size_t i = 0; i < num_total_robots; i++) {
    robot_start_locations[i] = getPose(robots[i]["location"][0].as<float>(),
                                       robots[i]["location"][1].as<float>(),
                                       robots[i]["location"][2].as<float>());
  }
}

void availableRobotHandler(const bwi_msgs::AvailableRobotArray::ConstPtr available_robots) {
  BOOST_FOREACH(const bwi_msgs::AvailableRobot &robot, available_robots->robots) {
    if (std::find(available_robot_list.begin(), available_robot_list.end(), robot.name) == available_robot_list.end()) {
      available_robot_list.push_back(robot.name);
    }
  }

  // TODO: parametrize.
  if (available_robot_list.size() == num_total_robots) {
    waiting_for_robots = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_robot_navigator");

  boost::this_thread::sleep(boost::posix_time::milliseconds(15000));

  boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

  ros::Subscriber available_robots_subscriber = nh->subscribe("/available_robots", 1, availableRobotHandler);
  ros::ServiceClient get_gazebo_model_client =
      nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  get_gazebo_model_client.waitForExistence();
  ros::ServiceClient set_gazebo_model_client =
      nh->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  set_gazebo_model_client.waitForExistence();

  ros::ServiceClient gui_service = 
    nh->serviceClient<bwi_msgs::QuestionDialog>("question_dialog");

  boost::shared_ptr<actionlib::SimpleActionClient<utexas_guidance_msgs::MultiRobotNavigationAction> > mrn_client;

  ros::NodeHandle private_nh("~");
  std::string problems_file, config_file;
  if (!private_nh.getParam("problems_file", problems_file)) {
    ROS_FATAL_STREAM("RobotPosition: ~problems_file parameter required");
    exit(-1);
  }
  if (!private_nh.getParam("config_file", config_file)) {
    ROS_FATAL_STREAM("RobotPosition: ~config_file parameter required");
    exit(-1);
  }
  readInitialConfigurationFile(config_file);
  ROS_INFO("robot_navigator_exp: READ CONFIG FILE!");
  readExperimentFile(problems_file);
  ROS_INFO("robot_navigator_exp: READ EXPERIMENT FILE!");

  ROS_INFO("robot_navigator_exp: ALL SERVICES FOUND AND INITIALIZED!");

  while(waiting_for_robots) {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }

  std::vector<ros::Publisher> robot_pose_pubilshers(num_total_robots);
  for (int robot_idx = 0; robot_idx < num_total_robots; ++robot_idx) {
    robot_pose_pubilshers[robot_idx] = 
      nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + available_robot_list[robot_idx] + "/initialpose", 1);
  }

  int problem_start_idx = 0;
  int problem_end_idx = problem_start_locations.size();
  int only_problem_idx = -1;
  private_nh.getParam("only_problem_idx", only_problem_idx);
  if (only_problem_idx != -1) {
    problem_start_idx = only_problem_idx;
    problem_end_idx = only_problem_idx + 1;
  }

  for (int problem_idx = problem_start_idx; problem_idx < problem_end_idx; ++problem_idx) {

    RobotNavigator brn(nh, available_robot_list);

    // Teleport robots to simulation file locations.
    for (int robot_idx = 0; robot_idx < num_total_robots; ++robot_idx) {
      teleportEntity(available_robot_list[robot_idx],
                     robot_start_locations[robot_idx],
                     get_gazebo_model_client,
                     set_gazebo_model_client);
      robot_pose_pubilshers[robot_idx].publish(getStartPoseWithCovariance(robot_idx));
    }

    // Teleport person behind one of the robots.
    teleportEntity("person",
                   problem_start_locations[problem_idx],
                   get_gazebo_model_client,
                   set_gazebo_model_client);

    // Wait for user to start experiment.
    displayMessageWithPause("Start next experiment?", gui_service);
    //displayMessage("Follow the robots' advice to reach the goal location!", gui_service);
    displayMessage("", gui_service);

    for (int robot_idx = 0; robot_idx < num_total_robots; ++robot_idx) {
      robot_pose_pubilshers[robot_idx].publish(getStartPoseWithCovariance(robot_idx));
    }

    // Start the solver for this problem.
    brn.start();

    // Wait for the actionlib client, while waiting an additional 1 seconds for it.
    mrn_client.reset(new actionlib::SimpleActionClient<utexas_guidance_msgs::MultiRobotNavigationAction>("/guidance", true));
    ROS_INFO_NAMED("guidance_gui_controller", "nav: Waiting for guidance action server.");
    mrn_client->waitForServer();
    ROS_INFO_NAMED("guidance_gui_controller", "nav: Guidance action server found.");
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    // Send the current goal and see what happens.
    utexas_guidance_msgs::MultiRobotNavigationGoal goal;
    goal.goal_node_id = problem_goal_idx[problem_idx];
    goal.solver_alias = solver_alias;
    mrn_client->sendGoal(goal);

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration time_since_start;

    // Don't check for completion in the first 20 seconds.
    while (time_since_start.total_milliseconds() < 20000) {
      ros::spinOnce();
      time_since_start = boost::posix_time::microsec_clock::local_time() - start_time;
    }

    while (!mrn_client->getState().isDone() &&
           time_since_start.total_milliseconds() < 300000) {
      ros::spinOnce();
      time_since_start = boost::posix_time::microsec_clock::local_time() - start_time;
    }

    std::vector<std::map<std::string, std::string> > records;
    std::map<std::string, std::string> record;
    record["problem_idx"] = boost::lexical_cast<std::string>(problem_idx);
    record["problems_file"] = problems_file;
    record["solver"] = solver_alias;
    if (!mrn_client->getState().isDone()) {
      // TODO get reward somehow before cancelling goal, or maybe not amortize this reward.
      mrn_client->cancelGoal();
      if (problem_idx != problem_end_idx - 1) {
        displayMessageWithPause("Oh no! It looks like the robots were unable to help you. Let's proceed to the next problem!",
                                gui_service);
      } else {
        displayMessageWithPause("Oh no! It looks like the robots were unable to help you.", gui_service);
      }
      ROS_INFO_STREAM("Accrued  " << mrn_client->getResult()->reward << " reward in " << time_since_start.total_milliseconds() << " ms.");
      record["success"] = boost::lexical_cast<std::string>(false);
      record["estimated_reward"] = boost::lexical_cast<std::string>(mrn_client->getResult()->reward);
      record["time"] = boost::lexical_cast<std::string>(time_since_start.total_milliseconds() / 1000.0f);
    } else {

      if (problem_idx != problem_end_idx - 1) {
        displayMessageWithPause("Yay! You found the goal location. Let's proceed to the next problem!",
                                gui_service);
      } else {
        displayMessageWithPause("Yay! You found the goal location.", gui_service);
      }
      ROS_INFO_STREAM("Accrued  " << mrn_client->getResult()->reward << " reward in " << time_since_start.total_milliseconds() << " ms.");
      record["success"] = boost::lexical_cast<std::string>(true);
      record["estimated_reward"] = boost::lexical_cast<std::string>(mrn_client->getResult()->reward);
      record["time"] = boost::lexical_cast<std::string>(time_since_start.total_milliseconds() / 1000.0f);
    }

    mrn_client.reset();

    records.push_back(record);
    utexas_planning::writeRecordsAsCSV("/home/piyushk/guidance_ws/result." + 
                                       boost::lexical_cast<std::string>(problem_idx) + "." + 
                                       boost::lexical_cast<std::string>(time_since_start.total_milliseconds()), 
                      records);

    // Results
    // TODO: you'll need to figure out a way of getting the MDP reward back.
  }

  return 0;
}
