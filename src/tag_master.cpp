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

  void TagMaster::publishTags(ros::Publisher &tag_pub, ros::Publisher &obj_pub, ros::Publisher &tag_vis_pub, ros::Publisher &obj_vis_pub)
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
      std::vector<visualization_msgs::Marker> tag_vis;
      std::vector<visualization_msgs::Marker> obj_vis;
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
            tag_vis.push_back(toMarker(msg, "tag"));
            tag_vis.push_back(toMarker(msg, "tag", true));
            obj_vis.push_back(toMarker(msg2, "object"));
            obj_vis.push_back(toMarker(msg2, "object", true));
          }
        }
      }
      // for visualization purposes
      visualization_msgs::MarkerArray tmarr;
      tmarr.markers = tag_vis;
      tag_vis_pub.publish(tmarr);
      visualization_msgs::MarkerArray omarr;
      omarr.markers = obj_vis;
      obj_vis_pub.publish(omarr);
    }
  }

  visualization_msgs::Marker TagMaster::toMarker(tag_master::TagPose tag_pose, std::string ns, bool text)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = tag_pose.pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = tag_pose.id + (text ? 1000 : 0); // both arrow and text can be shown this way, up to 1000 tags
    marker.pose = tag_pose.pose.pose;
    marker.scale.x = 0.03;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.a = 1.0;
    marker.action = visualization_msgs::Marker::ADD;
    if (ns == "tag")
    {
      marker.color.r = 0.3;
      marker.color.g = 0.8;
      marker.color.b = 0.1;
    }
    else
    {
      marker.color.r = 0.3;
      marker.color.g = 0.2;
      marker.color.b = 0.8;
    }
    if (text)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      std::string str_head = ns == "tag" ? "tag_" : "object_";
      marker.text = str_head + std::to_string(tag_pose.id) + ": " + tag_pose.object_name;
    }
    else
    {
      marker.type = visualization_msgs::Marker::ARROW;
    }

    return marker;
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
      // ROS_INFO("id_sizes[%d]: %d | %f", i, id_size_pairs[i].first, id_size_pairs[i].second);
      id_size_map_[id_size_pairs[i].first] = id_size_pairs[i].second;
    }
  }
} // namespace tag_master