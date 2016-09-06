#include <utexas_guidance_ros/robot_navigator.h>
#include <boost/foreach.hpp>

using namespace utexas_guidance_ros;

bool waiting_for_robots = true;
std::vector<std::string> available_robot_list;

void availableRobotHandler(const bwi_msgs::AvailableRobotArray::ConstPtr available_robots) {
  BOOST_FOREACH(const bwi_msgs::AvailableRobot &robot, available_robots->robots) {
    if (std::find(available_robot_list.begin(), available_robot_list.end(), robot.name) == available_robot_list.end()) {
      available_robot_list.push_back(robot.name);
    }
  }

  // TODO: parametrize.
  if (available_robot_list.size() == 3) {
    waiting_for_robots = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_robot_navigator");

  boost::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

  ros::Subscriber available_robots_subscriber = nh->subscribe("/available_robots", 1, availableRobotHandler);

  while(waiting_for_robots) {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }

  RobotNavigator brn(nh, available_robot_list);
  brn.start();

  return 0;
}
