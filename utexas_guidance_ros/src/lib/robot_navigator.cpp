#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/highgui.h>
#include <tf/transform_datatypes.h>

#include <utexas_guidance_msgs/UpdateGuidanceGui.h>
#include <utexas_guidance_ros/robot_navigator.h>
#include <utexas_planning/common/utils.h>
#include <opencv/highgui.h>

#include <utexas_planning/planners/mcts/mcts.h>
#include <utexas_guidance/mdp/single_robot_solver.h>
#include <utexas_guidance/mdp/pdpt_solver.h>

namespace utexas_guidance_ros {

  RobotNavigator::RobotNavigator(const boost::shared_ptr<ros::NodeHandle>& nh,
                                 const std::vector<std::string>& available_robot_list) {

    nh_ = nh;

    available_robot_list_ = available_robot_list;

    // This decides whether an episode inside the MDP is running or not.
    episode_in_progress_ = false;
    episode_completed_ = false;
    terminate_episode_ = false;
    at_episode_start_ = false;
    pause_robot_ = utexas_guidance::NONE;

    // TODO: Parametrize!
    controller_thread_frequency_ = 2.0f;

    // Read the parameters from file and initialize the RNG.
    std::string experiment_file;
    ros::NodeHandle private_nh("~");
    if (!private_nh.getParam("experiment_file", experiment_file)) {
      ROS_FATAL_STREAM("RobotPosition: ~experiment_file parameter required");
      exit(-1);
    }

    YAML::Node params = YAML::LoadFile(experiment_file);
    model_params_ = params["models"][0];
    all_planner_params_ = params["planners"];

    master_rng_.reset(new RNG(0));

    human_location_available_ = false;
    human_location_subscriber_ = nh_->subscribe("person/pose", 1, &RobotNavigator::humanLocationHandler, this);

    model_.reset(new utexas_guidance::GuidanceModel);
    model_->init(model_params_, "", master_rng_);
    model_->getUnderlyingGraph(graph_);

  }

  RobotNavigator::~RobotNavigator() {
    // TODO join the controller thread here if initialized?
  }

  void RobotNavigator::start() {

    ROS_INFO_NAMED("RobotNavigator", "Starting up...");

    // Setup variables/controllers for each robot.
    int num_robots = available_robot_list_.size();

    robot_location_available_.resize(num_robots, false);
    robot_location_.resize(num_robots); // We're just resizing the vector here.
    robot_location_mutex_.resize(num_robots);
    for (int i = 0; i < num_robots; ++i) {
      robot_location_mutex_[i].reset(new boost::mutex);
    }

    robot_service_task_start_time_.resize(num_robots);
    robot_command_status_.resize(num_robots, INITIALIZED);

    for (int i = 0; i < num_robots; ++i) {
      const std::string &robot_name = available_robot_list_[i];

      // Add a controller for this robot.
      ROS_INFO_STREAM_NAMED("RobotNavigator", "Waiting for action server for robot: " << robot_name);
      boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > ac;
      ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/" + robot_name + "/move_base_interruptable", true));
      ac->waitForServer();
      robot_controller_.push_back(ac);

      boost::shared_ptr<ros::Subscriber> loc_sub(new ros::Subscriber);
      *loc_sub = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_name + "/amcl_pose", 1,
                                                                          boost::bind(&RobotNavigator::robotLocationHandler,
                                                                                      this, _1, i));
      robot_location_subscriber_.push_back(loc_sub);

      boost::shared_ptr<ros::ServiceClient> gui_client(new ros::ServiceClient);
      *gui_client = nh_->serviceClient<utexas_guidance_msgs::UpdateGuidanceGui>("/" + robot_name + "/update_gui");
      robot_gui_controller_.push_back(gui_client);

      // Finally get a new task for this robot and setup the system state for this robot.
      utexas_guidance::RobotState rs;
      // rs.loc will be setup once the subscriber kicks in. set the loc to -1 for now to indicate the location has
      // not been received.
      rs.loc_u = utexas_guidance::NONE;
      rs.loc_v = utexas_guidance::NONE;
      rs.loc_p = 0.0f;

      rs.tau_d = utexas_guidance::NONE; // Get Starting task.
      model_->getUnderlyingTaskModel()->generateNewTaskForRobot(i, rs, *master_rng_);

      rs.help_destination = utexas_guidance::NONE;
      rs.is_leading_person = false;

      system_state_.robots.push_back(rs);
    }

    // Now that all robots are initialized, start the controller thread.
    controller_thread_.reset(new boost::thread(&RobotNavigator::runControllerThread, this));

    as_.reset(new actionlib::SimpleActionServer<utexas_guidance_msgs::MultiRobotNavigationAction>(*nh_,
                                                                                                  "/guidance",
                                                                                                  boost::bind(&RobotNavigator::execute, this, _1),
                                                                                                  false));

    as_->start();
    ros::spin();
  }

  void RobotNavigator::execute(const utexas_guidance_msgs::MultiRobotNavigationGoalConstPtr &goal) {

    solver_.reset();

    for (unsigned planner_idx = 0; planner_idx < all_planner_params_.size(); ++planner_idx) {
      std::string planner_alias = all_planner_params_[planner_idx]["alias"].as<std::string>();
      if (planner_alias == goal->solver_alias) {
        std::string planner_name = all_planner_params_[planner_idx]["name"].as<std::string>();
        if (planner_name == "utexas_planning::MCTS") {
          solver_.reset(new utexas_planning::MCTS);
        } else if (planner_name == "utexas_guidance::SingleRobotSolver") {
          solver_.reset(new utexas_guidance::SingleRobotSolver);
        } else if (planner_name == "utexas_guidance::PDPTSolver") {
          solver_.reset(new utexas_guidance::PDPTSolver);
        }
        utexas_planning::GenerativeModel::Ptr planner_model = model_;
        if (all_planner_params_[planner_idx]["model"]) {
          planner_model.reset(new utexas_guidance::GuidanceModel);
          planner_model->init(all_planner_params_[planner_idx]["model"], "", master_rng_);
        }
        solver_->init(planner_model, all_planner_params_[planner_idx], "", master_rng_, true);
        break;
      }
    }

    if (!solver_) {
      ROS_ERROR_STREAM_NAMED("robot_navigator", "Could not find solver with alias " << goal->solver_alias); 
      as_->setAborted();
      return;
    }

    episode_modification_mutex_.lock();
    episode_completed_ = false;
    terminate_episode_ = false;
    episode_in_progress_ = true;
    at_episode_start_ = true;
    goal_node_id_ = goal->goal_node_id;
    human_location_available_ = false;

    episode_modification_mutex_.unlock();

    while (ros::ok()) {
      episode_modification_mutex_.lock();
      if (as_->isPreemptRequested()) {
        terminate_episode_ = true;
        as_->setAborted();
        break;
      }
      if (episode_completed_) {
        as_->setSucceeded();
        break;
      }
      episode_modification_mutex_.unlock();
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

  }

  void RobotNavigator::humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose) {
    human_location_available_ = true;
    human_location_ = human_pose->pose.pose;
  }

  void RobotNavigator::robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose,
                                            int robot_idx) {
    boost::mutex::scoped_lock lock(*(robot_location_mutex_[robot_idx]));
    robot_location_[robot_idx] = robot_pose->pose.pose;
    robot_location_available_[robot_idx] = true;
  }

  void RobotNavigator::runControllerThread() {

    ROS_INFO_NAMED("RobotNavigator", "Controller Thread Running...");

    while(ros::ok()) {

      /* restricted_mutex_scope */ {
        boost::mutex::scoped_lock episode_modification_lock(episode_modification_mutex_);

        /* ESTIMATE ROBOT LOCATIONS */
        bool all_robot_locations_available = true;
        // Let's see if we can update the position of all the robots first!
        for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
          utexas_guidance::RobotState &rs = system_state_.robots[robot_idx];
          if (rs.loc_u == -1) {
            // This robot's location has never been initialized.
            boost::mutex::scoped_lock lock(*(robot_location_mutex_[robot_idx]));
            if (robot_location_available_[robot_idx]) {
              determineStartLocation(robot_location_[robot_idx], rs.loc_u, rs.loc_v, rs.loc_p);
            } else {
              all_robot_locations_available = false;
            }
          } else if (robot_command_status_[robot_idx] != INITIALIZED) {
            determineRobotTransitionalLocation(robot_location_[robot_idx], rs);
            actionlib::SimpleClientGoalState robot_state = robot_controller_[robot_idx]->getState();
            if (robot_state == actionlib::SimpleClientGoalState::SUCCEEDED ||
                robot_state == actionlib::SimpleClientGoalState::LOST) {
              roundOffRobotLocation(rs);
              if (robot_command_status_[robot_idx] == GOING_TO_HELP_DESTINATION_LOCATION) {
                robot_command_status_[robot_idx] = AT_HELP_DESTINATION_LOCATION;
              } else if (robot_command_status_[robot_idx] == GOING_TO_SERVICE_TASK_LOCATION) {
                robot_command_status_[robot_idx] = AT_SERVICE_TASK_LOCATION;
                if (!episode_in_progress_) {
                  utexas_guidance_msgs::UpdateGuidanceGui srv;
                  srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START;
                  robot_gui_controller_[robot_idx]->call(srv);
                }
              }
              // else {
              //   ROS_FATAL_STREAM("BAD STATE TRANSITION 1 - robot successfully completed navigation action in command status" << robot_command_status_[robot_idx]);
              //   throw std::runtime_error("");
              // }
            } else if (robot_state == actionlib::SimpleClientGoalState::RECALLED ||
                       robot_state == actionlib::SimpleClientGoalState::REJECTED ||
                       robot_state == actionlib::SimpleClientGoalState::PREEMPTED ||
                       robot_state == actionlib::SimpleClientGoalState::ABORTED) {
              if (robot_command_status_[robot_idx] == GOING_TO_HELP_DESTINATION_LOCATION) {
                robot_command_status_[robot_idx] = HELP_DESTINATION_NAVIGATION_FAILED;
              } else if (robot_command_status_[robot_idx] == GOING_TO_SERVICE_TASK_LOCATION) {
                robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
              }
              // else {
              //   ROS_FATAL_STREAM("BAD STATE TRANSITION 2 - robot failed navigation action in command status" << robot_command_status_[robot_idx]);
              //   throw std::runtime_error("");
              // }
            }
          }
        }
        /* end ESTIMATE ROBOT LOCATIONS */

        /* ROS_WARN_STREAM("2"); */

        if (all_robot_locations_available) {
          bool its_decision_time = false;
          if (episode_in_progress_ && !terminate_episode_) {

            utexas_guidance::State ask_state;
            if (at_episode_start_) {
              // Ensure that all robots switch to a scenario where they are no longer showing anything.
              utexas_guidance_msgs::UpdateGuidanceGui srv;
              srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
              for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                robot_gui_controller_[robot_idx]->call(srv);
              }

              // Determine the human's start location.
              int loc;
              determineStartLocation(human_location_, loc);

              system_state_.requests.resize(1);
              system_state_.requests[0].request_id = utexas_guidance::generateNewRequestId();
              system_state_.requests[0].loc_node = system_state_.requests[0].loc_prev = loc;
              system_state_.requests[0].loc_p = 1.0f;
              system_state_.requests[0].assist_loc = utexas_guidance::NONE;
              system_state_.requests[0].assist_type = utexas_guidance::NONE;
              system_state_.requests[0].goal = goal_node_id_;
              system_state_.requests[0].is_new_request = true;

              system_state_.actions_since_wait.clear();

              // Now figure out if we can assign a robot to the human's current location.
              // std::vector<utexas_guidance::Action> actions;
              // model_->getAllActions(system_state_, actions);
              int colocated_robot_id = utexas_guidance::NONE;
              for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                const utexas_guidance::RobotState& rs = system_state_.robots[robot_idx];
                if (isRobotExactlyAt(rs, system_state_.requests[0].loc_node)) {
                  //     // We should assign this robot to
                  //     utexas_guidance::State temp_next_state;
                  //     bool unused_terminal; /* The resulting state can never be terminal via a non utexas_guidance::WAIT action */
                  //     float unused_reward_value;
                  //     int unused_depth_count;

                  //     // Note that the RNG won't be used as it is a deterministic action.
                  //     model_->takeAction(system_state_,
                  //                        utexas_guidance::Action(ASSIGN_ROBOT, -1, system_state_.loc_node),
                  //                        unused_reward_value,
                  //                        temp_next_state,
                  //                        unused_terminal,
                  //                        unused_depth_count,
                  //                        master_rng_);
                  //     system_state_ = temp_next_state;

                  robot_command_status_[robot_idx] = AT_HELP_DESTINATION_LOCATION;
                  colocated_robot_id = robot_idx;
                  break;
                }
              }

              // system_state_.prev_action = utexas_guidance::WAIT;
              // system_state_.released_locations.clear();
              // wait_action_next_states_.clear();

              ROS_INFO_STREAM_NAMED("base_robot_navigator", "System state at start determined: " << system_state_);

              if (colocated_robot_id != utexas_guidance::NONE) {
                utexas_guidance_msgs::UpdateGuidanceGui srv;
                srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_PLEASEWAIT;
                robot_gui_controller_[colocated_robot_id]->call(srv);
              }

              at_episode_start_ = false;
              its_decision_time = true;
              mcts_state_ = system_state_;
            } else {
              /* ROS_WARN_STREAM("2b"); */
              int next_loc = system_state_.requests[0].loc_node;
              int lead_action_idx = utexas_guidance::NONE;
              for (int action_idx = 0; action_idx < system_state_.actions_since_wait.size(); ++action_idx) {
                utexas_guidance::Action& prev_action = system_state_.actions_since_wait[action_idx];
                if (prev_action.type == utexas_guidance::LEAD_PERSON) {
                  lead_action_idx = action_idx;
                }
              }

              if (lead_action_idx != utexas_guidance::NONE) {
                utexas_guidance::Action& prev_action = system_state_.actions_since_wait[lead_action_idx];
                // Check if the assigned robot has completed its navigation action, and the human is close by.
                if (robot_command_status_[prev_action.robot_id] == AT_HELP_DESTINATION_LOCATION) {
                  float human_robot_xdiff =
                    robot_location_[prev_action.robot_id].position.x - human_location_.position.x;
                  float human_robot_ydiff =
                    robot_location_[prev_action.robot_id].position.y - human_location_.position.y;
                  float human_robot_distance =
                    sqrtf(human_robot_xdiff * human_robot_xdiff + human_robot_ydiff * human_robot_ydiff);
                  if (human_robot_distance <= 2.0f) {
                    next_loc = prev_action.node;
                    // Clear the previous robot's GUI.
                    // utexas_guidance_msgs::UpdateGuidanceGui srv;
                    // srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
                    // robot_gui_controller_[prev_action.robot_id]->call(srv);
                  }
                }
              } else {
                determineHumanTransitionalLocation(human_location_, system_state_.requests[0].loc_node, next_loc);
              }

              /* ROS_WARN_STREAM("2c"); */
              boost::posix_time::time_duration time_since_wait_start =
                boost::posix_time::microsec_clock::local_time() - wait_action_start_time_;

              /* std::cout << next_loc << std::endl; */

              if (next_loc != system_state_.requests[0].loc_node ||
                  (system_state_.requests[0].wait_time_left != 0.0f && 
                   time_since_wait_start.total_milliseconds() > system_state_.requests[0].wait_time_left * 1000)) {
                if (next_loc == goal_node_id_) {
                  ROS_INFO_STREAM("Reached destination!");
                  // TODO maybe do something special here if a robot is leading the person.
                  for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                    if (system_state_.robots[robot_idx].help_destination != utexas_guidance::NONE) {
                      system_state_.robots[robot_idx].help_destination = utexas_guidance::NONE;
                    }
                  }
                  episode_completed_ = true;
                  episode_in_progress_ = false;
                } else {
                  system_state_.requests[0].loc_prev = system_state_.requests[0].loc_node;
                  system_state_.requests[0].loc_node = next_loc;
                  system_state_.requests[0].assist_type = utexas_guidance::NONE;
                  system_state_.requests[0].assist_loc = utexas_guidance::NONE;
                  system_state_.requests[0].wait_time_left = 0.0f;
                  system_state_.requests[0].is_new_request = false;
                  for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                    if (system_state_.robots[robot_idx].is_leading_person) {
                      system_state_.robots[robot_idx].help_destination = utexas_guidance::NONE;
                      system_state_.robots[robot_idx].is_leading_person = false;
                    }
                  }
                  system_state_.actions_since_wait.clear();
                  mcts_state_ = getTransformedSystemState();
                  its_decision_time = true;
                }
              }
            }
            /* ROS_WARN_STREAM("2d"); */
            //publishCurrentSystemState();
            /* ROS_WARN_STREAM("2e"); */
          } else if (terminate_episode_) {
            terminate_episode_ = false;
            for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
              if (system_state_.robots[robot_idx].help_destination != utexas_guidance::NONE) {
                system_state_.robots[robot_idx].help_destination = utexas_guidance::NONE;
                system_state_.robots[robot_idx].is_leading_person = false;
              }
            }
          }

          /* ROS_WARN_STREAM("3"); */
          if (its_decision_time) {
            while (true) {
              ROS_INFO_STREAM_NAMED("base_robot_navigator", "taking action at system state: " << system_state_);
              /* std::vector<utexas_guidance::Action> actions; */
              /* getAllActions(system_state_, actions); */
              // BOOST_FOREACH(utexas_guidance::Action& action, actions) {
              //   ROS_INFO_STREAM_NAMED("base_robot_navigator", "  action: " << action);
              // }

              utexas_guidance::Action action = getBestAction();
              ROS_INFO_STREAM_NAMED("base_robot_navigator", "taking action " << action);
              if (action.type != utexas_guidance::WAIT) {
                // See if the action requires some interaction with the GUI.
                if (action.type == utexas_guidance::DIRECT_PERSON) {
                  pause_robot_ = action.robot_id;
                  utexas_guidance_msgs::UpdateGuidanceGui srv;
                  srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION;
                  utexas_guidance::Point3f ori_dest_pt = utexas_guidance::getLocationFromGraphId(action.node, graph_);
                  srv.request.orientation_destination.position.x = ori_dest_pt.get<0>();
                  srv.request.orientation_destination.position.y = ori_dest_pt.get<1>();
                  robot_gui_controller_[action.robot_id]->call(srv);
                } else if (action.type == utexas_guidance::LEAD_PERSON) {
                  utexas_guidance_msgs::UpdateGuidanceGui srv;
                  srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
                  if (isRobotExactlyAt(system_state_.robots[action.robot_id], action.node)) {
                    srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_PLEASEWAIT;
                  }
                  robot_gui_controller_[action.robot_id]->call(srv);
                  robot_command_status_[action.robot_id] = SERVICE_TASK_NAVIGATION_RESET;
                }
                // Perform the deterministic transition as per the model
                utexas_guidance::State next_state;
                // Note that the RNG won't be used as it is a deterministic action.
                takeAction(system_state_, 
                           action, 
                           next_state);

                // If a robot was assigned a task, and it is not exactly at the location it was at, then reset the nav
                // task.
                for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
                  if (system_state_.robots[robot_idx].help_destination != next_state.robots[robot_idx].help_destination) {
                    robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
                  }
                }

                system_state_ = next_state;

                /* This is a state in the MCTS search tree that corresponds to the current system state. */
                takeAction(mcts_state_,
                           action,
                           next_state);

                // Move the root node of the planner along.
                utexas_guidance::State::ConstPtr search_state_ptr(new utexas_guidance::State(mcts_state_));
                utexas_guidance::Action::ConstPtr action_ptr(new utexas_guidance::Action(action));
                // TODO make sure that the planning isn't getting reset.
                solver_->performPostActionProcessing(search_state_ptr,
                                                     action_ptr,
                                                     0.0f);

                mcts_state_ = next_state;
              } else {
                // Let's switch to non-deterministic transition logic.
                ROS_WARN_STREAM("Restart called!");
                mcts_search_start_state_ = system_state_;
                wait_action_next_states_.clear();
                for (int i = 0; i < 100; ++i) {
                  // Perform the deterministic transition as per the model
                  utexas_guidance::State next_state;
                  takeAction(system_state_, 
                             action, 
                             next_state);
                  wait_action_next_states_.insert(next_state);
                }
                ROS_WARN_STREAM("Expecting the following possible next states after wait action: ");
                BOOST_FOREACH(const utexas_guidance::State& ns, wait_action_next_states_) {
                  ROS_WARN_STREAM("  " << ns);
                }
                wait_action_start_time_ = boost::posix_time::microsec_clock::local_time();
                break;
              }
            }
          }

          /* ROS_WARN_STREAM("4"); */
          for (int robot_idx = 0; robot_idx < system_state_.robots.size(); ++robot_idx) {
            /* ROS_WARN_STREAM("  idx: " << robot_idx); */
            utexas_guidance::RobotState &rs = system_state_.robots[robot_idx];

            // Check if a robot service task is still being initialized, or was just completed.
            if (robot_command_status_[robot_idx] == INITIALIZED) {
              robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
              if (!episode_in_progress_) {
                utexas_guidance_msgs::UpdateGuidanceGui srv;
                srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
                robot_gui_controller_[robot_idx]->call(srv);
              }
            } else if ((robot_command_status_[robot_idx] == AT_HELP_DESTINATION_LOCATION && isRobotExactlyAt(rs, rs.tau_d)) ||
                       robot_command_status_[robot_idx] == AT_SERVICE_TASK_LOCATION) {
              if (rs.tau_t == 0.0f) {
                ROS_DEBUG_STREAM_NAMED("RobotNavigator", available_robot_list_[robot_idx] << " is at service task location.");
                rs.tau_t = 1.0f / controller_thread_frequency_;
                // The second term tries to average for discretization errors. TODO think about this some more when
                // not sleepy.
                robot_service_task_start_time_[robot_idx] = boost::posix_time::microsec_clock::local_time() -
                  boost::posix_time::milliseconds(0.5 * 1.0f / controller_thread_frequency_);
              } else {
                boost::posix_time::time_duration diff =
                  boost::posix_time::microsec_clock::local_time() - robot_service_task_start_time_[robot_idx];
                rs.tau_t = diff.total_milliseconds() / 1000.0f;
                ROS_DEBUG_STREAM_NAMED("RobotNavigator", available_robot_list_[robot_idx] << " has been at service task location for " << rs.tau_t << " seconds.");
              }
              if (rs.tau_t > rs.tau_total_task_time) {
                // This service task is now complete. Get a new task.
                model_->getUnderlyingTaskModel()->generateNewTaskForRobot(robot_idx, rs, *master_rng_);
                if (robot_command_status_[robot_idx] == AT_SERVICE_TASK_LOCATION) {
                  robot_command_status_[robot_idx] = SERVICE_TASK_NAVIGATION_RESET;
                  if (!episode_in_progress_) {
                    utexas_guidance_msgs::UpdateGuidanceGui srv;
                    srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
                    robot_gui_controller_[robot_idx]->call(srv);
                  }
                }
              }
            }

            // Check if a robot that was manually paused no longer needs to be paused.
            if (pause_robot_ == robot_idx) {
              boost::posix_time::time_duration time_since_wait_start =
                boost::posix_time::microsec_clock::local_time() - wait_action_start_time_;
              if (time_since_wait_start.total_milliseconds() > 1500) {
                pause_robot_ = utexas_guidance::NONE;
                utexas_guidance_msgs::UpdateGuidanceGui srv;
                srv.request.type = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
                robot_gui_controller_[robot_idx]->call(srv);
              }
            } else {
              bool robot_aiding_human = rs.help_destination != utexas_guidance::NONE;
              if (robot_aiding_human) {
                if (robot_command_status_[robot_idx] != AT_HELP_DESTINATION_LOCATION &&
                    robot_command_status_[robot_idx] != GOING_TO_HELP_DESTINATION_LOCATION) {
                  int destination = rs.help_destination;
                  float orientation = utexas_guidance::getNodeAngle(system_state_.requests[0].loc_node, rs.help_destination, graph_);
                  sendRobotToDestination(robot_idx, destination, orientation);
                  robot_command_status_[robot_idx] = GOING_TO_HELP_DESTINATION_LOCATION;
                } /* TODO see if the robot needs to be rotated. */
              } else {
                if (robot_command_status_[robot_idx] != AT_SERVICE_TASK_LOCATION &&
                    robot_command_status_[robot_idx] != GOING_TO_SERVICE_TASK_LOCATION) {
                  int destination = rs.tau_d;
                  float orientation = utexas_guidance::getNodeAngle(rs.loc_u, rs.tau_d, graph_);
                  sendRobotToDestination(robot_idx, destination, orientation);
                  robot_command_status_[robot_idx] = GOING_TO_SERVICE_TASK_LOCATION;
                }
              }
            }
          }

          /* ROS_WARN_STREAM("4done"); */
        } else {
          ROS_WARN_THROTTLE_NAMED(1.0f, "MultiRobotNavigator", " still waiting for robot locations. This shouldn't take too long...");
        }
      }

      /* ROS_WARN_STREAM("5"); */
      if (!episode_in_progress_) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f/controller_thread_frequency_));
      } else {

        utexas_guidance::State::ConstPtr search_state_ptr(new utexas_guidance::State(mcts_search_start_state_));
        utexas_guidance::Action::ConstPtr wait_action(new utexas_guidance::Action(utexas_guidance::WAIT));
        // TODO make sure that the planning isn't getting reset.
        solver_->performPostActionProcessing(search_state_ptr,
                                             wait_action,
                                             1.0f / controller_thread_frequency_);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0f/controller_thread_frequency_));
      }

    }
  }

  void RobotNavigator::sendRobotToDestination(int robot_idx, int destination, float orientation) {
    utexas_guidance::Point3f dest = utexas_guidance::getLocationFromGraphId(destination, graph_);
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped &target_pose = goal.target_pose;
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = available_robot_list_[robot_idx] + "/level_mux/map";
    target_pose.pose.position.x = dest.get<0>();
    target_pose.pose.position.y = dest.get<1>();
    target_pose.pose.position.z = 0;
    target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
    ROS_INFO_STREAM("Sending " << available_robot_list_[robot_idx] << " to " << 
                    destination << " (" << dest.get<0>() << "," << dest.get<1>() << ")");
    robot_controller_[robot_idx]->sendGoal(goal);
  }

  void RobotNavigator::determineHumanTransitionalLocation(const geometry_msgs::Pose &pose,
                                                          int current_loc,
                                                          int& next_loc) {
    utexas_guidance::Point3f human_pt(pose.position.x, pose.position.y);
    int next_graph_id = utexas_guidance::getClosestEdgeOnGraphGivenId(human_pt, graph_, current_loc);
    utexas_guidance::Point3f current_pt = utexas_guidance::getLocationFromGraphId(current_loc, graph_);
    utexas_guidance::Point3f next_pt = utexas_guidance::getLocationFromGraphId(next_graph_id, graph_);
    // ROS_WARN_STREAM("Human's current location: " << current_pt.get<0>() << "," << current_pt.get<1>() <<
    //                 " and next_pt " << next_pt.get<0>() << "," << next_pt.get<1>());
    // ROS_WARN_STREAM("  Diff in magnitude: " << boost::geometry::distance(next_pt, human_pt));
    if (boost::geometry::distance(next_pt, human_pt) <= 2.0f) {
      next_loc = next_graph_id;
    } else {
      next_loc = current_loc;
    }
  }

  void RobotNavigator::determineStartLocation(const geometry_msgs::Pose &pose, int &u, int &v, float &p) {
    utexas_guidance::Point3f pt(pose.position.x, pose.position.y);
    u = utexas_guidance::getClosestIdOnGraph(pt, graph_);
    v = utexas_guidance::getClosestEdgeOnGraphGivenId(pt, graph_, u);
    utexas_guidance::Point3f u_loc = utexas_guidance::getLocationFromGraphId(u, graph_);
    utexas_guidance::Point3f v_loc = utexas_guidance::getLocationFromGraphId(v, graph_);
    p = boost::geometry::distance(pt, u_loc) /
      (boost::geometry::distance(pt, u_loc) + boost::geometry::distance(pt, v_loc));
  }

  void RobotNavigator::determineStartLocation(const geometry_msgs::Pose &pose, int &u) {
    utexas_guidance::Point3f pt(pose.position.x, pose.position.y);
    u = utexas_guidance::getClosestIdOnGraph(pt, graph_);
  }

  void RobotNavigator::determineRobotTransitionalLocation(const geometry_msgs::Pose &pose, utexas_guidance::RobotState &rs) {
    // Change loc_v everytime, but keep loc_u constant.
    utexas_guidance::Point3f pt(pose.position.x, pose.position.y);
    rs.loc_v = utexas_guidance::getClosestEdgeOnGraphGivenId(pt, graph_, rs.loc_u);
    utexas_guidance::Point3f u_loc = utexas_guidance::getLocationFromGraphId(rs.loc_u, graph_);
    utexas_guidance::Point3f v_loc = utexas_guidance::getLocationFromGraphId(rs.loc_v, graph_);
    rs.loc_p = boost::geometry::distance(pt, u_loc) /
      (boost::geometry::distance(pt, u_loc) + boost::geometry::distance(pt, v_loc));
    // Swapping allows the robot to transition from one node to the next.
    if (rs.loc_p >= 0.6f) {
      int temp = rs.loc_u;
      rs.loc_u = rs.loc_v;
      rs.loc_v = temp;
      rs.loc_p = 1 - rs.loc_p;
    }
  }

  void RobotNavigator::roundOffRobotLocation(utexas_guidance::RobotState &rs) {
    if (rs.loc_p >= 0.5f) {
      rs.loc_p = 0.0f;
      int temp = rs.loc_v;
      rs.loc_u = rs.loc_v;
      rs.loc_v = temp;
    } else {
      // u and v are correctly set anyway!
      rs.loc_p = 0.0f;
    }
  }

  utexas_guidance::Action RobotNavigator::getBestAction() {
    utexas_planning::State::ConstPtr state_ptr(new utexas_guidance::State(mcts_state_));
    utexas_planning::Action::ConstPtr action_ptr = solver_->getBestAction(state_ptr);
    utexas_guidance::Action::ConstPtr action_derived_ptr = 
      boost::dynamic_pointer_cast<const utexas_guidance::Action>(action_ptr);
    return *action_derived_ptr;
  }

  utexas_guidance::State RobotNavigator::getTransformedSystemState() {

    utexas_guidance::State ask_state = system_state_;
    if (wait_action_next_states_.size() != 0) {
      // Look through all possible next states and figure out positionally which one we are closest to. Then set
      // the robot locations to that one as long as that does not change the current action set.
      std::vector<utexas_guidance::Action> current_state_actions;
      getAllActions(system_state_, current_state_actions);
      std::set<utexas_guidance::State> candidates;
      BOOST_FOREACH(const utexas_guidance::State& next_state, wait_action_next_states_) {
        if (next_state.requests[0].loc_node == system_state_.requests[0].loc_node) {
          candidates.insert(next_state);
        }
      }

      ROS_WARN_STREAM("The size of candidate next states is " << candidates.size());

      // TODO you could still probably do this better!
      // Make sure that changing robot values won't change the set of actions.
      BOOST_FOREACH(const utexas_guidance::State& next_state, candidates) {
        utexas_guidance::State temp_state = ask_state;
        for (int robot_idx = 0; robot_idx < next_state.robots.size(); ++robot_idx) {
          temp_state.robots[robot_idx] = next_state.robots[robot_idx];
          // .loc_u =
          //   next_state.robots[robot_idx].loc_u;
          // temp_state.robots[robot_idx].loc_v =
          //   next_state.robots[robot_idx].loc_v;
          // temp_state.robots[robot_idx].loc_p =
          //   next_state.robots[robot_idx].loc_p;
          // temp_state.robots[robot_idx].tau_t =
          //   next_state.robots[robot_idx].tau_t;
          // temp_state.robots[robot_idx].tau_u =
          //   next_state.robots[robot_idx].tau_u;
          // temp_state.robots[robot_idx].tau_d =
          //   next_state.robots[robot_idx].tau_d;
          // temp_state.robots[robot_idx].tau_total_task_time =
          //   next_state.robots[robot_idx].tau_total_task_time;
        }
        std::vector<utexas_guidance::Action> temp_state_actions;
        getAllActions(temp_state, temp_state_actions);
        if (current_state_actions == temp_state_actions) {
          ROS_WARN_STREAM("Close candidate " << next_state << " found!");
          ROS_WARN_STREAM("  Constructing state " << temp_state << " from this candidate to search in old tree!");
          ask_state = temp_state;
          break;
        } else {
          ROS_WARN("Resetting values changed action space!");
          BOOST_FOREACH(utexas_guidance::Action& action, temp_state_actions) {
            ROS_WARN_STREAM("  " << action);
          }
        }
      }
    }
    return ask_state;

  }

  void RobotNavigator::takeAction(const utexas_guidance::State& state, 
                                  const utexas_guidance::Action& action,
                                  utexas_guidance::State& next_state) {
    
    // TODO This reward should go somewhere.
    float unused_reward = 0.0f;
    utexas_planning::State::ConstPtr state_ptr(new utexas_guidance::State(state));
    utexas_planning::Action::ConstPtr action_ptr(new utexas_guidance::Action(action));
    utexas_planning::State::ConstPtr next_state_ptr;

    int unused_depth_count;
    float unused_action_timeout;

    model_->takeAction(state_ptr,
                       action_ptr,
                       unused_reward,
                       utexas_planning::RewardMetrics::Ptr(),
                       next_state_ptr,
                       unused_depth_count,
                       unused_action_timeout,
                       master_rng_);

    utexas_guidance::State::ConstPtr next_state_derived_ptr = 
      boost::dynamic_pointer_cast<const utexas_guidance::State>(next_state_ptr);
    next_state = *next_state_derived_ptr;

  }

  void RobotNavigator::getAllActions(const utexas_guidance::State& state,
                                     std::vector<utexas_guidance::Action>& actions) {
    
    utexas_planning::State::ConstPtr state_ptr(new utexas_guidance::State(state));
    std::vector<utexas_planning::Action::ConstPtr> action_ptrs;
    model_->getActionsAtState(state_ptr, action_ptrs);

    actions.clear();
    BOOST_FOREACH(const utexas_planning::Action::ConstPtr& action_ptr, action_ptrs) {
      utexas_guidance::Action::ConstPtr action_derived_ptr = 
        boost::dynamic_pointer_cast<const utexas_guidance::Action>(action_ptr);
      actions.push_back(*action_derived_ptr);
    }

  }

} /* utexas_guidance */
