#ifndef UTEXAS_GUIDANCE_ROBOT_NAVIGATOR_H
#define UTEXAS_GUIDANCE_ROBOT_NAVIGATOR_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/shared_ptr.hpp>
#include <bwi_mapper/point_utils.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_mapper/graph.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <bwi_msgs/AvailableRobotArray.h>
#include <utexas_guidance_msgs/ExperimentStatus.h>
#include <utexas_guidance_msgs/MultiRobotNavigationAction.h>
#include <utexas_guidance/mdp/guidance_model.h>
#include <utexas_planning/core/abstract_planner.h>

namespace utexas_guidance_ros {

  enum RobotCommandStatus {
    INITIALIZED,
    GOING_TO_SERVICE_TASK_LOCATION,
    AT_SERVICE_TASK_LOCATION,
    SERVICE_TASK_NAVIGATION_RESET,
    GOING_TO_HELP_DESTINATION_LOCATION,
    AT_HELP_DESTINATION_LOCATION,
    HELP_DESTINATION_NAVIGATION_FAILED,
  };

  class RobotNavigator {

    public:

      RobotNavigator(const boost::shared_ptr<ros::NodeHandle>& nh,
                     const std::vector<std::string>& available_robot_list);

      virtual ~RobotNavigator();

      bool human_location_available_;
      ros::Subscriber human_location_subscriber_;
      void humanLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose);
      geometry_msgs::Pose human_location_;

      // std_srvs/Empty call that freezes the available robot list and starts patrolling them via the concurrent thread.
      // Doesn't actually provide the goal.
      void start();

      std::vector<std::string> available_robot_list_;

      std::vector<boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > > robot_controller_; /* Directly corresponds to vector id in current state. */

      std::vector<boost::posix_time::ptime> robot_service_task_start_time_;
      std::vector<bool> robot_location_available_;
      std::vector<geometry_msgs::Pose> robot_location_;
      std::vector<boost::shared_ptr<boost::mutex> > robot_location_mutex_;

      std::vector<RobotCommandStatus> robot_command_status_;
      std::vector<boost::shared_ptr<ros::Subscriber> > robot_location_subscriber_;
      std::vector<boost::shared_ptr<ros::ServiceClient> > robot_gui_controller_;
      void robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose, int robot_idx);

      std::vector<bool> robot_offered_help_;
      std::vector<boost::posix_time::ptime> robot_offered_help_start_time_;
      /* Once WAIT is returned, clean the MCTS state - DOWNSTREAM! */
      utexas_guidance::Action getBestAction();

      utexas_guidance::State getTransformedSystemState();
      /* void getNextTaskForRobot(int robot_id, RobotState &rs); */

    protected:

      cv::Mat base_image_;

      void execute(const utexas_guidance_msgs::MultiRobotNavigationGoalConstPtr &goal);
      void sendRobotToDestination(int robot_idx, int destination, float orientation = 0.0f);
      void determineHumanTransitionalLocation(const geometry_msgs::Pose &pose, int current_loc, int &next_loc);
      void determineStartLocation(const geometry_msgs::Pose &pose, int &u, int &v, float &p);
      void determineStartLocation(const geometry_msgs::Pose &pose, int &u);
      void determineRobotTransitionalLocation(const geometry_msgs::Pose &pose, utexas_guidance::RobotState &rs);
      void roundOffRobotLocation(utexas_guidance::RobotState &rs);

      void takeAction(const utexas_guidance::State& state, 
                      const utexas_guidance::Action& action,
                      utexas_guidance::State& next_state);

      void getAllActions(const utexas_guidance::State& state,
                         std::vector<utexas_guidance::Action>& actions);

      /* bwi_mapper::Point2f getLocationFromGraphId(int destination); */

      utexas_guidance::State system_state_;
      utexas_guidance::State mcts_state_;
      boost::mutex episode_modification_mutex_;

      boost::shared_ptr<ros::NodeHandle> nh_;
      boost::shared_ptr<actionlib::SimpleActionServer<utexas_guidance_msgs::MultiRobotNavigationAction> > as_;

      bool episode_in_progress_;
      bool episode_completed_;
      bool terminate_episode_;
      bool at_episode_start_;

      int goal_node_id_;
      int pause_robot_;

      /* TODO: This may have become unnecessary. */
      utexas_guidance::State mcts_search_start_state_;
      boost::posix_time::ptime wait_action_start_time_;
      std::set<utexas_guidance::State> wait_action_next_states_;

      boost::shared_ptr<boost::thread> controller_thread_;
      void runControllerThread();
      float controller_thread_frequency_;

      YAML::Node model_params_;
      YAML::Node all_planner_params_;
      utexas_guidance::Graph graph_;
      utexas_guidance::GuidanceModel::Ptr model_;
      utexas_planning::AbstractPlanner::Ptr solver_;
      boost::shared_ptr<RNG> master_rng_;
  };

} /* utexas_guidance_ros */

#endif /* end of include guard: UTEXAS_GUIDANCE_ROBOT_NAVIGATOR_H */
