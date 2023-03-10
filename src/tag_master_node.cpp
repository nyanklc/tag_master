#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <tag_master/tag_master.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
  // ROS_INFO("received image");
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

  /* test */
  std::string atag_det_name = "atag_det";
  auto atag_ptr = std::make_shared<tag_detection::AprilTagDetector>(
      atag_det_name, 1.0, 0.8, 8, false, 619.32689027, 617.14607294, 364.50967726, 264.79765919, 0.075, true, true);
  tm.addDetector<tag_detection::AprilTagDetector>(atag_ptr);

  ros::Rate r(10);
  int iter_count = 0;
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
    
    if (out.success)
    {
      // clear markers
      visualization_msgs::MarkerArray cube_marker_array;
      visualization_msgs::Marker mark;
      mark.action = visualization_msgs::Marker::DELETEALL;
      cube_marker_array.markers.push_back(mark);
      cube_pub.publish(cube_marker_array);
      cube_marker_array.markers.clear();

      // ROS_INFO("out.detection size: %zd", out.detection.size());
      for (auto &detection : out.detection)
      {
        // ROS_INFO("herhehrehrehhre");
        std::shared_ptr<tag_detection::AprilTagDetector> xd = std::dynamic_pointer_cast<tag_detection::AprilTagDetector>(tm.getDetector("atag_det"));
        // ROS_INFO(xd->getName().c_str());
        // ROS_INFO("asdjajsdjasjdjasd");
        xd->drawDetections(frame_color);
        xd->drawCubes(frame_color, "camera_link", cube_marker_array);
        // ROS_INFO("yyyyyyyyyyyyyyyyyyyyyyyyyy");
      }

      // publish markers
      cube_pub.publish(cube_marker_array);
    }

    // std::string img_path = "/home/noyan/ros_ws/src/tag_master/img/asd" + std::to_string(iter_count) + ".png";
    // iter_count++;
    // cv::imwrite(img_path, frame_color);
    cv::imshow("frame", frame_color);
    cv::waitKey(1);
  }

  return 0;
}
