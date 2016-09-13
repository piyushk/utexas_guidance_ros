#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <bwi_msgs/QuestionDialog.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>

#include <utexas_guidance_msgs/MultiRobotNavigationAction.h>
#include <utexas_guidance_msgs/UpdateGuidanceGui.h>

int system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;

std::vector<std::string> goal_names;
std::vector<int> goal_graph_ids;
boost::shared_ptr<boost::thread> episode_start_thread;
boost::shared_ptr<actionlib::SimpleActionClient<utexas_guidance_msgs::MultiRobotNavigationAction> > mrn_client;
ros::ServiceClient gui_service;

cv::Mat map_image;
nav_msgs::MapMetaData map_info;

std::string tf_prefix;
std::string robot_name;
ros::Publisher image_publisher;
cv::Mat u_turn_image, up_arrow_image;
geometry_msgs::Pose robot_location;

bool use_rqt_visualizer = false;
bool use_overhead_directions = false;

  void drawArrowOnImage(cv::Mat &image, const cv::Point2f &arrow_center, float orientation,
                        const cv::Scalar &color, int size, int thickness) {

    cv::Point arrow_start = arrow_center +
      cv::Point2f(size * cosf(orientation + M_PI/2),
                  size * sinf(orientation + M_PI/2));
    cv::Point arrow_end = arrow_center -
      cv::Point2f(size * cosf(orientation + M_PI/2),
                  size * sinf(orientation + M_PI/2));

    cv::line(image, arrow_start, arrow_end, color, thickness, CV_AA);

    // http://mlikihazar.blogspot.com/2013/02/draw-arrow-opencv.html
    cv::Point p(arrow_start), q(arrow_end);

    //Draw the first segment
    float angle = atan2f(p.y - q.y, p.x - q.x);
    p.x = (int) (q.x + (0.1 * size - 1) * cos(angle + M_PI/4));
    p.y = (int) (q.y + (0.1 * size - 1) * sin(angle + M_PI/4));
    cv::line(image, p, q, color, thickness, CV_AA);

    //Draw the second segment
    p.x = (int) (q.x + (0.1 * size + 1) * cos(angle - M_PI/4));
    p.y = (int) (q.y + (0.1 * size + 1) * sin(angle - M_PI/4));
    cv::line(image, p, q, color, thickness, CV_AA);

  }

void readGoalsFromFile(std::string &filename) {

  std::ifstream fin(filename.c_str());
  YAML::Node doc;
  doc = YAML::Load(fin);
  for (size_t i = 0; i < doc.size(); ++i) {
    std::string name;
    int node;
    name = doc[i]["name"].as<std::string>();
    node = doc[i]["node"].as<int>();
    goal_names.push_back(name);
    goal_graph_ids.push_back(node);
  }
  fin.close();

}

void monitorEpisodeStartThread() {
  bwi_msgs::QuestionDialog srv;
  srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION;
  srv.request.message = "Where would you like to go?";
  BOOST_FOREACH(const std::string& name, goal_names) {
    srv.request.options.push_back(name);
  }
  ROS_INFO_NAMED("guidance_gui_controller", "sending req with %i options", int(srv.request.options.size()));
  srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT;
  if (gui_service.call(srv)) {
    if (srv.response.index >= 0) {
      ROS_INFO_NAMED("guidance_gui_controller", "req timed out, sending ep start request.");
      utexas_guidance_msgs::MultiRobotNavigationGoal goal;
      goal.goal_node_id = goal_graph_ids[srv.response.index];
      goal.solver_alias = "MCTS";
      mrn_client->sendGoal(goal);
    } else {
      // Do nothing. The request probably got preempted.
    }
  } else {
    ROS_INFO_NAMED("guidance_gui_controller", "request failed for unspecified reasons.");
  }
}

void displayMessage(const std::string& msg) {
  if (use_rqt_visualizer) {
    bwi_msgs::QuestionDialog srv;
    srv.request.type = bwi_msgs::QuestionDialogRequest::DISPLAY;
    srv.request.message = msg;
    srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT;
    gui_service.call(srv);
  }
}

void displayImage(const cv::Mat& image) {
  cv_bridge::CvImage out_image;
  out_image.header.frame_id = "laptop_screen_link";
  if (tf_prefix != "") {
    out_image.header.frame_id = tf_prefix + "laptop_screen_link";
  }
  out_image.header.stamp = ros::Time::now();
  out_image.encoding = sensor_msgs::image_encodings::BGR8;
  out_image.image = image;
  image_publisher.publish(out_image.toImageMsg());
}

void clearImage() {
  cv::Mat blank_image = cv::Mat::zeros(120, 160, CV_8UC3);
  if (!robot_name.empty()) {
    cv::putText(blank_image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  }
  displayImage(blank_image);
}

void showArrowToDestination(const geometry_msgs::Pose &orientation_destination) {

  cv::Mat image;

  // First figure out what exact orientation to show on the screen based on the location of the robot, the orientation
  // of the robot and the destination which the robot wants to indicate.
  float destination_yaw = atan2(orientation_destination.position.y - robot_location.position.y,
                                orientation_destination.position.x - robot_location.position.x);
  float orientation = destination_yaw - tf::getYaw(robot_location.orientation);
  while (orientation <= -M_PI) orientation += 2 * M_PI;
  while (orientation > M_PI) orientation -= 2 * M_PI;

  cv::Mat rotated_image;
  int height = u_turn_image.rows, width = u_turn_image.cols;
  if (fabs(orientation) > 5.0 * M_PI / 6.0) {
    rotated_image = u_turn_image;
  } else {
    height = up_arrow_image.rows, width = up_arrow_image.cols;
    cv::Point2f center(width/2, height/2);
    cv::Mat rotation_matrix =
      cv::getRotationMatrix2D(center, orientation * 180 / M_PI, 1.0);

    cv::warpAffine(up_arrow_image, rotated_image, rotation_matrix,
                   cv::Size(width, height));
  }

  float height_ratio = 119.0 / height;
  float width_ratio = 159.0 / width;
  float min_ratio = std::min(height_ratio, width_ratio);

  cv::Mat resized_image;
  cv::resize(rotated_image, resized_image,
             cv::Size(0,0), min_ratio, min_ratio);

  image = cv::Mat::zeros(120, 160, CV_8UC3);
  int top = (image.rows - resized_image.rows) / 2;
  int bottom = image.rows - resized_image.rows - top;
  int left = (image.cols - resized_image.cols) / 2;
  int right = image.cols - resized_image.cols - left;

  cv::copyMakeBorder(resized_image, image, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  displayImage(image);

}

void showArrowToDestination2(const geometry_msgs::Pose &robot_location,
                             const geometry_msgs::Pose &orientation_destination) {

  cv::Mat image;

  bwi_mapper::Point2f robot_pt(robot_location.position.x, robot_location.position.y);
  float yaw = tf::getYaw(robot_location.orientation);

  bwi_mapper::Point2f dest_pt(orientation_destination.position.x, orientation_destination.position.y);
  bwi_mapper::Point2f diff_pt = dest_pt - robot_pt;
  float yaw2 = atan2f(diff_pt.y, diff_pt.x);

  bwi_mapper::Point2f dest_proj_pt = robot_pt + 
    cv::norm(dest_pt - robot_pt) * cosf(yaw2 - yaw) * bwi_mapper::Point2f(cosf(yaw), sinf(yaw));

  bwi_mapper::Point2f robot_grid(bwi_mapper::toGrid(robot_pt, map_info));
  bwi_mapper::Point2f dest_grid(bwi_mapper::toGrid(dest_pt, map_info));
  bwi_mapper::Point2f dest_proj_grid(bwi_mapper::toGrid(dest_proj_pt, map_info));
  
  cv::RotatedRect rect;
  rect.center = 0.5 * (robot_grid + dest_proj_grid);
  rect.size = cv::Size(cv::norm(dest_proj_grid - robot_grid) * 1.2 * 4.0 / 3.0,
                       cv::norm(dest_proj_grid - robot_grid) * 1.2);
  rect.angle = 180. / M_PI * (atan2f((dest_proj_grid - robot_grid).y, (dest_proj_grid - robot_grid).x)) + 90;

  cv::Mat map_image_mutable = map_image.clone();
  cv::circle(map_image_mutable, robot_grid, 10, cv::Scalar(0,0,255), -1, CV_AA);

  drawArrowOnImage(map_image_mutable, 
                   0.5 * (robot_grid + dest_grid), 
                   atan2f((dest_grid - robot_grid).y, (dest_grid - robot_grid).x) + M_PI/2,
                   cv::Scalar(0,0,255), 
                   cv::norm(dest_grid - robot_grid) * 0.5, 
                   10);

  // Got this code from somewhere, thought I've forgotten where.

  // matrices we'll use
  cv::Mat M, rotated, cropped;
  // get angle and size from the bounding box
  float angle = rect.angle;
  cv::Size rect_size = rect.size;
  // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
  if (rect.angle < -45.) {
    angle += 90.0;
    std::swap(rect_size.width, rect_size.height);
  }
  // get the rotation matrix
  M = cv::getRotationMatrix2D(rect.center, angle, 1.0);
  // perform the affine transformation
  cv::warpAffine(map_image_mutable, rotated, M, map_image.size(), cv::INTER_CUBIC);

  // crop the resulting image
  cv::getRectSubPix(rotated, rect_size, rect.center, cropped);
  cv::flip(cropped, image, 1);

  // Get grid location and orientation_destination location.
  // rotate image. get locations in this new image.  
  float height = image.rows, width = image.cols;
  
  float height_ratio = 479.0 / height;
  float width_ratio = 639.0 / width;
  float min_ratio = std::min(height_ratio, width_ratio);

  cv::Mat resized_image;
  cv::resize(image, resized_image,
             cv::Size(0,0), min_ratio, min_ratio);

  image = cv::Mat::zeros(480, 640, CV_8UC3);
  int top = (image.rows - resized_image.rows) / 2;
  int bottom = image.rows - resized_image.rows - top;
  int left = (image.cols - resized_image.cols) / 2;
  int right = image.cols - resized_image.cols - left;

  cv::copyMakeBorder(resized_image, image, top, bottom, left, right, 
                     cv::BORDER_CONSTANT, cv::Scalar(128,128,128));
  displayImage(image);

}
void showAllDoneImage() {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
  // if (!robot_name.empty()) {
  //   cv::putText(image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
  //               CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  // }
  cv::putText(image, "All Done!", cv::Point2f(10, image.rows/2-5),
              CV_FONT_HERSHEY_SIMPLEX, 0.90, cv::Scalar(0, 215, 255), 4);
  displayImage(image);
}

void showFollowMeImage() {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
  // if (!robot_name.empty()) {
  //   cv::putText(image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
  //               CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  // }
  cv::putText(image, "Follow Me!", cv::Point2f(10, image.rows/2-5),
              CV_FONT_HERSHEY_SIMPLEX, 0.90, cv::Scalar(0, 215, 255), 4);
  displayImage(image);
}

void showPleaseWaitImage() {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC3);
  // if (!robot_name.empty()) {
  //   cv::putText(image, ("Hi! I'm " + robot_name), cv::Point2f(10, 25),
  //               CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 215, 255), 2);
  // }
  cv::putText(image, "Please Wait!", cv::Point2f(10, image.rows/2-5),
              CV_FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 215, 255), 3);
  displayImage(image);
}

bool updateGui(utexas_guidance_msgs::UpdateGuidanceGui::Request& request,
               utexas_guidance_msgs::UpdateGuidanceGui::Response& response) {
  response.success = true;
  response.message = "";
  /* ROS_WARN_STREAM_NAMED("guidance_gui_controller", "received request type: " << request.type); */
  switch(request.type) {
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START:
      if (system_state == utexas_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START) {
        response.success = false;
        response.message = "Episode start already enabled!";
      } else {
        clearImage();
        system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::ENABLE_EPISODE_START;
        // if (use_rqt_visualizer) {
        //   episode_start_thread.reset(new boost::thread(&monitorEpisodeStartThread));
        // }
      }
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START:
      clearImage();
      displayMessage("If you would like navigation assistance, please follow me till I stop and let me know.");
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::DISABLE_EPISODE_START;
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING:
      clearImage();
      displayMessage("");
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_NOTHING;
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION:
      if (!use_overhead_directions) {
        showArrowToDestination(request.orientation_destination);
        displayMessage("Please walk ahead in the indicated direction!");
      } else {
        showArrowToDestination2(request.robot_location, request.orientation_destination);
        displayMessage("Please walk to the location indicated by the arrow! (Red dot indicates your location, Black dot indicates the robot's location)");
      }
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ORIENTATION;
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME:
      showFollowMeImage();
      /* displayMessage("Follow Me!"); */
      displayMessage("");
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_PLEASEWAIT:
      showPleaseWaitImage();
      /* displayMessage("Please wait here!"); */
      displayMessage("");
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
      break;
    case utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_ALLDONE:
      showAllDoneImage();
      /* displayMessage("All Done!"); */
      displayMessage("");
      system_state = utexas_guidance_msgs::UpdateGuidanceGuiRequest::SHOW_FOLLOWME;
      break;
  };

  /* ROS_WARN_STREAM_NAMED("guidance_gui_controller", "  request done!"); */
  return true;
}

void locationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
  robot_location = pose->pose.pose;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "guidance_gui_controller");
  ros::NodeHandle nh, private_nh("~");

  use_rqt_visualizer = false;
  private_nh.getParam("use_rqt_visualizer", use_rqt_visualizer);
    
  // if (use_rqt_visualizer) {
  //   std::string goals_file_param_key;
  //   std::string goals_file;
  //   if (private_nh.searchParam("goals_file", goals_file_param_key)) {
  //     if (!private_nh.getParam(goals_file_param_key, goals_file)) {
  //       ROS_FATAL("Goals file parameter goals_file not specified!");
  //       return -1;
  //     }
  //   } else {
  //     ROS_FATAL("Goals file parameter goals_file not specified!");
  //     return -1;
  //   }
  //   readGoalsFromFile(goals_file);
  // }

  // Read images from parameters
  std::string up_arrow_image_file, u_turn_image_file;
  std::string images_dir = ros::package::getPath("utexas_guidance_ros") + "/images";

  std::string key;
  tf_prefix = "";
  if (private_nh.searchParam("tf_prefix", key)) {
    private_nh.getParam(key, tf_prefix);
  }

  if (tf_prefix != "" && tf_prefix[tf_prefix.size() - 1] != '/') {
    tf_prefix = tf_prefix + "/";
  }

  robot_name = tf_prefix.substr(0, tf_prefix.size() - 1);
  if (!robot_name.empty()) {
    robot_name[0] = toupper(robot_name[0]);
  }

  use_rqt_visualizer = false;
  private_nh.getParam("use_rqt_visualizer", use_rqt_visualizer);
    
  use_overhead_directions = false;
  private_nh.getParam("use_overhead_directions", use_overhead_directions);
  if (use_overhead_directions) {
    // TODO: this won't work with the multiple floors.
    std::string map_file;
    if (!private_nh.getParam("map_file", map_file)) {
      ROS_FATAL("Map file parameter ~map_file not specified!");
      return -1;
    }
    bwi_mapper::MapLoader mapper(map_file);
    nav_msgs::OccupancyGrid map;
    mapper.getMap(map);
    mapper.getMapInfo(map_info);
    mapper.drawMap(map_image, map);
  }

  private_nh.param<std::string>("up_arrow_image", up_arrow_image_file, images_dir + "/Up.png");
  up_arrow_image = cv::imread(up_arrow_image_file);
  private_nh.param<std::string>("u_turn_image", u_turn_image_file, images_dir + "/UTurn.png");
  u_turn_image = cv::imread(u_turn_image_file);

  mrn_client.reset(new actionlib::SimpleActionClient<utexas_guidance_msgs::MultiRobotNavigationAction>("/guidance", true));
  ROS_INFO_NAMED("guidance_gui_controller", "Waiting for guidance action server.");
  mrn_client->waitForServer();
  ROS_INFO_NAMED("guidance_gui_controller", "Guidance action server found.");

  ros::ServiceServer enable_episode_start_service = nh.advertiseService("update_gui", &updateGui);

  if (use_rqt_visualizer) {
    gui_service = nh.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
    ROS_INFO_NAMED("guidance_gui_controller", "Waiting for segbot_gui service.");
    gui_service.waitForExistence();
    ROS_INFO_NAMED("guidance_gui_controller", "segbot_gui service found.");
  }

  image_publisher = nh.advertise<sensor_msgs::Image>("image", 1, true);
  ros::Subscriber robot_location_subscriber = nh.subscribe("amcl_pose", 1, locationHandler);

  ros::spin();

  return 0;
}
