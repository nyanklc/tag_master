#include "geometry_msgs/PoseStamped.h"
#include "tag_master/detector_base.h"
#include <tag_master/tag_master.h>

namespace tag_master
{
  TagMaster::TagMaster() {}

  void TagMaster::addTagDescription(int _id, std::string _type, std::string _pub_frame, std::string _obj_name, geometry_msgs::Transform _objtransform)
  {
    TagDescription td;
    td.id = _id;
    td.type = _type;
    td.pub_frame = _pub_frame;
    td.obj_name = _obj_name;
    td.objtransform = _objtransform;
    tag_descriptions_.push_back(td);
  }

  void TagMaster::clearTagDescriptions()
  {
    tag_descriptions_.clear();
  }

  void TagMaster::enableDetector(std::string name, bool enable)
  {
    if (enable)
      enableDetector(name);
    else
      disableDetector(name);
  }

  void TagMaster::enableDetector(std::string name)
  {
    auto det = findDetector(name);
    det->enable();
  }

  void TagMaster::disableDetector(std::string name)
  {
    auto det = findDetector(name);
    det->disable();
  }

  void TagMaster::runSingle(std::string name, cv::Mat &frame)
  {
    auto det = findDetector(name);
    if (det == nullptr)
    {
      ROS_WARN("Can't run a detector that doesn't exist.");
      return;
    }
    det->process(frame);
    return;
  }

  void TagMaster::runAll(cv::Mat &frame)
  {
    for (auto &det : detectors_)
    {
      if (det->isEnabled())
      {
        det->process(frame);
      }
    }
  }

  void TagMaster::publishTags(ros::Publisher &tag_pub, ros::Publisher &obj_pub, ros::Publisher &tag_vis_pub, ros::Publisher &obj_vis_pub, ros::Publisher &original_vis_pub)
  {
    for (int i = 0; i < detectors_.size(); i++)
    {
      if (!detectors_[i]->isEnabled())
        continue;

      if (!detectors_[i]->isIdSizesSet())
        continue;

      auto output = detectors_[i]->output(tag_descriptions_, tf2_buffer_);
      if (!output.success)
        continue;

      // for visualization purposes
      std::vector<geometry_msgs::Pose> tag_vis;
      std::vector<geometry_msgs::Pose> obj_vis;
      std::vector<geometry_msgs::Pose> or_vis;
      std::string vis_frame_id;
      std::string original_vis_frame_id;

      for (auto &detection : output.detection)
      {
        // find matching description for this detection
        for (auto &des : tag_descriptions_)
        {
          if (des.id == detection.tag.id)
          {
            // tag
            tag_master::TagPose msg;
            msg.id = detection.tag.id;
            msg.pose = detection.tag.pose;
            msg.shape = detection.tag.shape.geometry;
            msg.type = detection.tag.type;
            msg.object_name = des.obj_name;

            // obj
            tag_master::TagPose msg2;
            msg2.id = msg.id;
            msg2.pose = detection.obj_pose;
            msg2.shape = -1;
            msg2.type = -1;
            msg2.object_name = des.obj_name;

            msg.pose.header.stamp = ros::Time::now();
            msg2.pose.header.stamp = msg.pose.header.stamp;
            tag_pub.publish(msg);
            obj_pub.publish(msg2);

            // for visualization purposes
            vis_frame_id = msg.pose.header.frame_id;
            original_vis_frame_id = detection.tag.detection_pose.header.frame_id;
            tag_vis.push_back(msg.pose.pose);
            obj_vis.push_back(msg2.pose.pose);
            or_vis.push_back(detection.tag.detection_pose.pose);
          }
        }
      }
      // for visualization purposes
      geometry_msgs::PoseArray tparr;
      tparr.poses = tag_vis;
      tparr.header.frame_id = vis_frame_id;
      tparr.header.stamp = ros::Time::now();
      tag_vis_pub.publish(tparr);
      geometry_msgs::PoseArray oparr;
      oparr.poses = obj_vis;
      oparr.header.frame_id = vis_frame_id;
      oparr.header.stamp = ros::Time::now();
      obj_vis_pub.publish(oparr);
      geometry_msgs::PoseArray orparr;
      orparr.poses = or_vis;
      orparr.header.frame_id = original_vis_frame_id;
      orparr.header.stamp = ros::Time::now();
      original_vis_pub.publish(orparr);
    }
  }

  tag_detection::DetectionOutput TagMaster::getOutput(std::string name)
  {
    auto det = findDetector(name);
    if (!det || !det->isIdSizesSet())
    {
      tag_detection::DetectionOutput out;
      out.success = false;
      return out;
    }
    return det->output(tag_descriptions_, tf2_buffer_);
  }

  std::vector<tag_detection::DetectionOutput> TagMaster::getOutputs()
  {
    std::vector<tag_detection::DetectionOutput> outputv;
    for (int i = 0; i < detectors_.size(); i++)
    {
      if (!detectors_[i]->isIdSizesSet())
        continue;
      auto o = detectors_[i]->output(tag_descriptions_, tf2_buffer_);
      if (!o.success)
        continue;
      outputv.push_back(o);
    }
    return outputv;
  }

  std::shared_ptr<tag_detection::DetectorBase> TagMaster::getDetector(std::string name)
  {
    return findDetector(name);
  }

  std::shared_ptr<tag_detection::DetectorBase> TagMaster::findDetector(std::string name)
  {
    for (size_t i = 0; i < detectors_.size(); i++)
    {
      if (detectors_[i]->getName() == name)
        return detectors_[i];
    }
    ROS_WARN("Detector %s does not exist!", name.c_str());
    return nullptr;
  }

  void TagMaster::updateCameraParams(double fx, double fy, double cx, double cy)
  {
    for (auto &det : detectors_)
      det->updateCameraParams(fx, fy, cx, cy);
  }

  void TagMaster::debugOutput()
  {
    ROS_INFO("### TAG MASTER DEBUG OUTPUT ###");
    for (int i = 0; i < detectors_.size(); i++)
    {
      ROS_INFO("Detector %d: name %s", i, detectors_[i]->getName().c_str());
    }
    for (int i = 0; i < tag_descriptions_.size(); i++)
    {
      ROS_INFO("Tag Descriptions %d: id %d, type %s, pub_frame %s, obj_name %s, objvector [%f, %f, %f] -> [%f, %f, %f]", i, tag_descriptions_[i].id, tag_descriptions_[i].type.c_str(), tag_descriptions_[i].pub_frame.c_str(), tag_descriptions_[i].obj_name.c_str(), tag_descriptions_[i].objtransform.translation.x, tag_descriptions_[i].objtransform.translation.y, tag_descriptions_[i].objtransform.translation.z, tag_descriptions_[i].objtransform.rotation.x, tag_descriptions_[i].objtransform.rotation.y, tag_descriptions_[i].objtransform.rotation.z);
    }
    ROS_INFO("### TAG MASTER DEBUG OUTPUT END ###");
  }

  void TagMaster::setBuffer(tf2_ros::Buffer *buf)
  {
    tf2_buffer_ = buf;
  }

  void TagMaster::setImageFrameId(std::string fid)
  {
    img_frame_id_ = fid;
    for (auto &det : detectors_)
    {
      det->setImageFrameId(fid);
    }
  }

  void TagMaster::setIdSizes(std::vector<std::pair<uint32_t, double>> id_size_pairs)
  {
    for (int i = 0; i < id_size_pairs.size(); i++)
    {
      ROS_INFO("id_sizes[%d]: %d | %f", i, id_size_pairs[i].first, id_size_pairs[i].second);
      id_size_map_[id_size_pairs[i].first] = id_size_pairs[i].second;
    }
  }
} // namespace tag_master