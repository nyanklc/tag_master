#include "geometry_msgs/PoseArray.h"
#include "tag_master/AddTagDescriptionRequest.h"
#include "tag_master/utils.h"
#include "tf2_ros/transform_listener.h"
#include <ros/xmlrpc_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tag_master/tag_master.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mutex>
#include <tag_master/AddDetector.h>
#include <tag_master/AddTagDescription.h>
#include <tag_master/DebugCall.h>
#include <tag_master/EnableDetector.h>
#include "../include/tag_description.h"

tag_master::TagMaster tm;
sensor_msgs::Image img;
bool img_received = false;
std::mutex img_mutex;
std::string img_frame_id;

sensor_msgs::CameraInfo camera_info;
bool camera_info_received = false;
std::mutex camera_info_mutex;

cv::Mat convertToMat(sensor_msgs::Image &i)
{
  cv_bridge::CvImagePtr cv_img_msg = cv_bridge::toCvCopy(i, "bgr8");
  cv::Mat frame = cv_img_msg.get()->image;
  cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  return frame;
}

void cameraCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(img_mutex);

  if (img_received)
    return;

  img = sensor_msgs::Image(*msg);
  img_received = true;
  img_frame_id = img.header.frame_id;
  // ROS_INFO("img_frame_id: %s", img_frame_id.c_str());
}

void cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex);

  if (camera_info_received)
    return;

  camera_info = msg;
  camera_info_received = true;
}

bool addDetectorService(tag_master::AddDetector::Request &req, tag_master::AddDetector::Response &res)
{
  if (req.type == "apriltag_detector")
  {
    auto det = std::make_shared<tag_detection::AprilTagDetector>(req.name, 1.0, 0.8, 8, false);
    tm.addDetector<tag_detection::AprilTagDetector>(det);
    return true;
  }
  return false;
}

bool addTagDescriptionService(tag_master::AddTagDescription::Request &req, tag_master::AddTagDescription::Request &res)
{
  if (req.id < 0)
    tm.clearTagDescriptions();
  else
    tm.addTagDescription(req.id, req.type, req.pub_frame, req.obj_name, req.objtransform);
  return true;
}

bool debugCallService(tag_master::DebugCall::Request &req, tag_master::DebugCall::Request &res)
{
  tm.debugOutput();
  return true;
}

bool enableDetectorService(tag_master::EnableDetector::Request &req, tag_master::EnableDetector::Request &res)
{
  tm.enableDetector(req.name, req.enable);
  return true;
}

std::vector<std::pair<uint32_t, double>> readIdSizes(ros::NodeHandle &nh)
{
  std::vector<std::pair<uint32_t, double>> id_size_pairs;
  if (nh.hasParam("id_sizes"))
  {
    XmlRpc::XmlRpcValue vec;
    nh.getParam("id_sizes", vec);
    if (vec.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < vec.size(); i++)
      {
        int id = vec[i][0];
        double size = vec[i][1];
        id_size_pairs.push_back(std::pair<uint32_t, double>(id, size));
      }
    }
  }
  return id_size_pairs;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_master_node");
  ros::NodeHandle nh("~");

  std::string robot_name = nh.param<std::string>("robot_name", "noyan");
  std::string camera_topic_name = "/" + robot_name + "/camera/publisher/color/image";
  std::string camera_info_topic_name = "/" + robot_name + "/camera/publisher/color/camera_info";

  ROS_INFO("/camera_topic_name: %s", camera_topic_name.c_str());
  ROS_INFO("/camera_info_topic_name: %s", camera_info_topic_name.c_str());

  ros::Subscriber camera_sub = nh.subscribe(camera_topic_name, 10, cameraCallback);
  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic_name, 20, cameraInfoCallback);
  ros::Publisher cube_pub = nh.advertise<visualization_msgs::MarkerArray>("apriltag_cubes", 10, true);
  ros::Publisher tags_pub = nh.advertise<tag_master::TagPose>("tag_master_detections", 10, true);
  ros::Publisher objs_pub = nh.advertise<tag_master::TagPose>("tag_master_object_detections", 10, true);
  ros::Publisher tags_vis_pub = nh.advertise<geometry_msgs::PoseArray>("tag_master_detections_vis", 10, true);
  ros::Publisher objs_vis_pub = nh.advertise<geometry_msgs::PoseArray>("tag_master_object_detections_vis", 10, true);
  ros::Publisher original_pose_pub = nh.advertise<geometry_msgs::PoseArray>("tag_master_camera_detections_vis", 10, true);

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);
  tm.setBuffer(&tf2_buffer);
  tm.setImageFrameId(img_frame_id);
  // read id-size pairs and send them to detectors
  auto id_sizes = readIdSizes(nh);
  tm.setIdSizes(id_sizes);

  ros::ServiceServer add_detector_service = nh.advertiseService("add_detector", addDetectorService);
  ros::ServiceServer add_tag_description_service = nh.advertiseService("add_tag_description", addTagDescriptionService);
  ros::ServiceServer debug_output_service = nh.advertiseService("debug_output", debugCallService);
  ros::ServiceServer detector_enable_service = nh.advertiseService("enable_detector", enableDetectorService);

  ros::Rate r(10);
  while (ros::ok())
  {
    // wait for subscribed topics
    while (1)
    {
      img_mutex.lock();
      camera_info_mutex.lock();

      if (!img_received || !camera_info_received)
      {
        img_mutex.unlock();
        camera_info_mutex.unlock();
        ros::spinOnce();
        r.sleep();
        ROS_DEBUG("haven't received img and camera_info yet");
        continue;
      }
      break;
    }
    img_received = false;
    camera_info_received = false;

    // update parameters of all detectors
    double camera_fx = camera_info.K[0];
    double camera_cx = camera_info.K[2];
    double camera_fy = camera_info.K[4];
    double camera_cy = camera_info.K[5];
    tm.updateCameraParams(camera_fx, camera_fy, camera_cx, camera_cy);
    tm.setImageFrameId(img_frame_id);

    auto frame = convertToMat(img);
    cv::Mat frame_color;
    cv::cvtColor(frame, frame_color, cv::COLOR_GRAY2BGR);

    img_mutex.unlock();
    camera_info_mutex.unlock();

    // run detectors
    tm.runAll(frame);
    tm.publishTags(tags_pub, objs_pub, tags_vis_pub, objs_vis_pub, original_pose_pub);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
