#include <ros/xmlrpc_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <tag_master/tag_master.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tag_master/apriltag_detector.h>

sensor_msgs::Image img;
bool img_received = false;

cv::Mat convertToMat(sensor_msgs::Image &i)
{
  cv_bridge::CvImagePtr cv_img_msg = cv_bridge::toCvCopy(i, "bgr8");
  cv::Mat frame = cv_img_msg.get()->image;
  cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
  return frame;
}

void cameraCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  img = sensor_msgs::Image(*msg);
  img_received = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_master_node");
  ros::NodeHandle nh;

  std::string camera_topic_name = nh.param<std::string>("camera_topic_name", "");
  ros::Subscriber camera_sub = nh.subscribe("/camera/color/image_raw", 10, cameraCallback);
  ros::Publisher cube_pub = nh.advertise<visualization_msgs::MarkerArray>("apriltag_cubes", 10, true);

  tag_master::TagMaster tm;

  // read params
  if (nh.hasParam("tags"))
  {
    // tags
    XmlRpc::XmlRpcValue tag_list;
    nh.getParam("tags", tag_list);
    for (int i = 0; i < tag_list.size(); i++)
    {
      int id = static_cast<int>(tag_list[i]["id"]);
      std::string type = static_cast<std::string>(tag_list[i]["type"]);
      std::string pub_frame = static_cast<std::string>(tag_list[i]["pub_frame"]);
      std::vector<double> objvector;
      objvector.push_back(static_cast<double>(tag_list[i]["objvector"]["x"]));
      objvector.push_back(static_cast<double>(tag_list[i]["objvector"]["y"]));
      objvector.push_back(static_cast<double>(tag_list[i]["objvector"]["z"]));
      // store tag descriptions
      tm.addTagDescription(id, type, pub_frame, objvector);
    }
    
    // detectors
    // TODO:
  }

  /* test */
  std::string atag_det_name = "atag_det";
  auto atag_ptr = std::make_shared<tag_detection::AprilTagDetector>(
      atag_det_name, 1.0, 0.8, 8, false, 619.32689027, 617.14607294, 364.50967726, 264.79765919, 0.075, true, true);
  tm.addDetector<tag_detection::AprilTagDetector>(atag_ptr);

  ros::Rate r(10);
  while (1)
  {
    while (!img_received)
    {
      ros::spinOnce();
      r.sleep();
    }
    img_received = false;

    auto frame = convertToMat(img);
    cv::Mat frame_color;
    cv::cvtColor(frame, frame_color, cv::COLOR_GRAY2BGR);

    tm.runSingle("atag_det", frame);
    tag_detection::DetectionOutput out = tm.getOutput("atag_det");
    
    if (cube_pub.getNumSubscribers() > 0 && out.success)
    {
      // clear markers
      visualization_msgs::MarkerArray cube_marker_array;
      visualization_msgs::Marker mark;
      mark.action = visualization_msgs::Marker::DELETEALL;
      cube_marker_array.markers.push_back(mark);
      cube_pub.publish(cube_marker_array);
      cube_marker_array.markers.clear();

      for (auto &detection : out.detection)
      {
        std::shared_ptr<tag_detection::AprilTagDetector> xd = std::dynamic_pointer_cast<tag_detection::AprilTagDetector>(tm.getDetector("atag_det"));
        xd->drawDetections(frame_color);
        xd->drawCubes(frame_color, "camera_link", cube_marker_array);
      }

      // publish markers
      cube_pub.publish(cube_marker_array);

      // cv::imshow("frame", frame_color);
      // cv::waitKey(1);
    }
  }

  return 0;
}
