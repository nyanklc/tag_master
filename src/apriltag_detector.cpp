#include <tag_master/apriltag_detector.h>

namespace tag_detection
{
  AprilTagDetector::AprilTagDetector(std::string name, double quad_decimate, double quad_sigma, int nthreads,
                                     bool refine_edges, bool initial_enable,
                                     bool enable_orthogonal_iteration, bool pose_estimation_enabled,
                                     double pose_estimation_error_max)
      : DetectorBase(name, initial_enable)
  {
    type_ = DetectorType::detector_type_apriltag;

    family_ = tagStandard41h12_create();
    detector_ = apriltag_detector_create();

    // 2 bits is recommended by the creators, >=3 uses a lot of memory
    // But higher the better
    // apriltag_detector_add_family_bits(detector_, family_, APRILTAG_FAMILY_BIT_COUNT);
    apriltag_detector_add_family(detector_, family_);

    if (errno == ENOMEM)
    {
      ROS_WARN(
          "Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the "
          "default maximum hamming value of 2. Try choosing an alternative tag family.\n");
      enabled_ = false;
    }
    enabled_ = true;

    detector_->quad_decimate = quad_decimate;
    detector_->quad_sigma = quad_sigma;
    detector_->nthreads = nthreads;
    detector_->refine_edges = refine_edges;
    detector_->debug = false;

    enable_orthogonal_iteration_ = enable_orthogonal_iteration;
    pose_estimation_enabled_ = pose_estimation_enabled;
    error_max_ = pose_estimation_error_max;
  }

  void AprilTagDetector::enable()
  {
    DetectorBase::enable();
  }

  void AprilTagDetector::disable()
  {
    DetectorBase::disable();
  }

  bool AprilTagDetector::process(cv::Mat &frame)
  {
    // base class checks if the detector is enabled
    if (!DetectorBase::process(frame))
      return false;

    if (!isIdSizesSet())
      return false;

    if (!detect(frame))
    {
      return false;
    }
    if (!estimatePose(frame, error_max_))
    {
      return false;
    }
    return true;
  }

  bool AprilTagDetector::detect(cv::Mat &frame)
  {
    // convert to apriltag image type
    image_u8_t im = {.width = frame.cols, .height = frame.rows, .stride = frame.cols, .buf = frame.data};

    errno = 0; // lol
    detections_ = apriltag_detector_detect(detector_, &im);
    if (errno == EAGAIN)
    {
      std::cout << "Unable to create the" << detector_->nthreads << " threads requested.\n";
      return false;
    }

    // not detected
    if (zarray_size(detections_) < 1)
      return false;

    return true;
  }

  bool AprilTagDetector::estimatePose(cv::Mat &frame, double error_max)
  {
    if (!pose_estimation_enabled_)
    {
      ROS_INFO("pose estimation is not enabled in detector %s", name_.c_str());
      return false;
    }

    bool success = true;

    std::vector<apriltag_pose_t> poses;
    poses_orthogonal_iteration_.clear();
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);

      detection_info_.det = det;
      detection_info_.tagsize = lookupTagSize(det->id);

      apriltag_pose_t pose;
      double err = estimate_tag_pose(&detection_info_, &pose);
      poses.push_back(pose);

      // ROS_INFO("estimated pose");
      // printMat(convertToMat(pose.R), "R");
      // printMat(convertToMat(pose.t), "t");

      if (enable_orthogonal_iteration_)
      {
        apriltag_pose_t pose1;
        apriltag_pose_t pose2;
        double err1;
        double err2;
        estimate_tag_pose_orthogonal_iteration(&detection_info_, &err1, &pose1, &err2, &pose2, 50);

        // ROS_INFO("estimated pose1");
        // printMat(convertToMat(pose1.R), "R1");
        // printMat(convertToMat(pose1.t), "t1");

        if (pose2.R)
        {
          // ROS_INFO("estimated pose2");
          // printMat(convertToMat(pose2.R), "R2");
          // printMat(convertToMat(pose2.t), "t2");
        }
        poses_orthogonal_iteration_.push_back(std::pair<apriltag_pose_t, apriltag_pose_t>(pose1, pose2));
      }

      if (err > error_max)
        success = false;
    }
    poses_.clear();
    poses_ = poses;

    return success;
  }

  std::vector<apriltag_pose_t> AprilTagDetector::getPoses()
  {
    return poses_;
  }

  zarray *AprilTagDetector::getDetections()
  {
    return detections_;
  }

  void AprilTagDetector::drawDetections(cv::Mat &frame)
  {
    // ROS_INFO("inside draw");
    // Draw detection outlines and midpoint
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);
      cv::circle(frame, cv::Point(det->c[0], det->c[1]), 1, cv::Scalar(255, 0, 0));
      cv::Point p1(det->p[0][0], det->p[0][1]);
      cv::Point p2(det->p[1][0], det->p[1][1]);
      cv::Point p3(det->p[2][0], det->p[2][1]);
      cv::Point p4(det->p[3][0], det->p[3][1]);

      cv::Point c(det->c[0], det->c[1]);

      cv::line(frame, p1, p2, cv::Scalar(100, 180, 0), 1);
      cv::line(frame, p1, p4, cv::Scalar(100, 180, 0), 1);
      cv::line(frame, p2, p3, cv::Scalar(100, 180, 0), 1);
      cv::line(frame, p3, p4, cv::Scalar(100, 180, 0), 1);
      cv::circle(frame, c, 2, cv::Scalar(0, 0, 255), 1);
    }
  }

  void AprilTagDetector::drawCubes(cv::Mat &frame, std::string frame_id, visualization_msgs::MarkerArray &marker_array)
  {
    for (size_t k = 0; k < poses_.size(); k++)
    {
      auto cube = defineCubeWithPoints();

      cv::Mat R = convertToMat(poses_[k].R);
      // cv::Rodrigues(R, R); // convert to rotation vector
      // printMat(R, "R");

      cv::Mat t = convertToMat(poses_[k].t);
      // printMat(t, "t");

      cv::Mat K = getCameraMatrix(detection_info_.fx, detection_info_.fy, detection_info_.cx, detection_info_.cy);
      // printMat(K, "K");

      cv::Mat distortion = getDistortionMatrix();
      // printMat(distortion, "distortion");

      auto color = cv::Scalar(0, 255, 0);
      drawCube(cube, frame, K, distortion, R, t, color, marker_array.markers, frame_id);

      if (enable_orthogonal_iteration_)
      {
        // cv::Mat R1 = convertToMat(poses_orthogonal_iteration_[k].first.R);
        // cv::Mat t1 = convertToMat(poses_orthogonal_iteration_[k].first.t);
        // auto color1 = cv::Scalar(255, 0, 0);
        // drawCube(cube, frame, K, distortion, R1, t1, color1, marker_array.markers, frame_id);

        if (poses_orthogonal_iteration_[k].second.R)
        {
          cv::Mat R2 = convertToMat(poses_orthogonal_iteration_[k].second.R);
          cv::Mat t2 = convertToMat(poses_orthogonal_iteration_[k].second.t);
          auto color2 = cv::Scalar(0, 0, 255);
          drawCube(cube, frame, K, distortion, R2, t2, color2, marker_array.markers, frame_id);
        }
      }
    }
  }

  DetectionOutput AprilTagDetector::output(std::vector<TagDescription> &tag_descriptions, tf2_ros::Buffer *tf2_buffer)
  {
    DetectionOutput out;
    if (zarray_size(detections_) == 0 || !isIdSizesSet())
    {
      out.success = false;
      return out;
    }

    for (size_t i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);
      for (int j = 0; j < tag_descriptions.size(); j++)
      {
        if (det->id != tag_descriptions[j].id)
          continue;

        // detection
        Detection detection;
        detection.tag.id = det->id;
        TagShape shape;
        shape.geometry = TagGeometry::square;
        TagSize size;
        size.width = detection_info_.tagsize;
        size.height = detection_info_.tagsize;
        shape.size = size;
        detection.tag.shape = shape;
        detection.tag.type = TagType::tag_type_apriltag;
        detection.tag.detection_pose = getTagPoseDetection(convertToEigenM3(poses_[i].R), convertToEigenV3(poses_[i].t));
        detection.tag.pose = getTagPose(detection.tag.detection_pose, tag_descriptions[j].pub_frame, tf2_buffer);
        detection.obj_pose = getObjectPose(tag_descriptions, detection.tag, tf2_buffer);
        out.detection.push_back(detection);
        // first id match is returned (in case a tag descriptions do not have unique ids)
        break;
      }
    }
    out.success = true;
    return out;
  }

  // returns tag's pose in the desired frame
  geometry_msgs::PoseStamped AprilTagDetector::getTagPose(geometry_msgs::PoseStamped &tag_pose_camera, std::string expected_frame_id, tf2_ros::Buffer *tf2_buffer)
  {
    // transform to expected frame
    geometry_msgs::PoseStamped tag_pose;
    tag_pose.header.frame_id = expected_frame_id;
    uint8_t trials = 0;
    ros::Rate r(20);
    tf2::Transform camera_to_tag;
    tf2::convert(tag_pose_camera.pose, camera_to_tag);
    while (ros::ok())
    {
    //   tf2_ros::TransformListener tf_listener(*tf2_buffer);
      try
      {
        auto pub_to_camera_msg = tf2_buffer->lookupTransform(expected_frame_id, tag_pose_camera.header.frame_id, ros::Time(0));
        tf2::Transform pub_to_camera;
        tf2::convert(pub_to_camera_msg.transform, pub_to_camera);
        tf2::Transform pub_to_tag = pub_to_camera * camera_to_tag;
        tf2::toMsg(pub_to_tag, tag_pose.pose);
        tag_pose.header.stamp = ros::Time::now();

        // // debug
        // Eigen::Isometry3d transform_matrix_iso;
        // Eigen::Matrix4d transform_matrix;
        // transform_matrix_iso = tf2::transformToEigen(pub_to_camera_msg.transform);
        // transform_matrix = transform_matrix_iso.matrix();
        // std::cout << "pub_to_camera: " << transform_matrix << "\n";
        // transform_matrix_iso = tf2::transformToEigen(tf2::toMsg(camera_to_tag));
        // transform_matrix = transform_matrix_iso.matrix();
        // std::cout << "camera_to_tag: " << transform_matrix << "\n";
        // transform_matrix_iso = tf2::transformToEigen(tf2::toMsg(pub_to_tag));
        // transform_matrix = transform_matrix_iso.matrix();
        // std::cout << "pub_to_tag: " << transform_matrix << "\n";

        break;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        if (trials > 9)
          break;
        trials++;
        r.sleep();
      }
    }
    return tag_pose;
  }

  // return the object's pose using the tag pose and object vector
  geometry_msgs::PoseStamped AprilTagDetector::getObjectPose(std::vector<TagDescription> &tag_descriptions, Tag &tag, tf2_ros::Buffer *tf2_buffer)
  {
    geometry_msgs::PoseStamped out;
    out.header.frame_id = tag.pose.header.frame_id;
    for (auto &des : tag_descriptions)
    {
      if (des.id == tag.id)
      {
        tf2::Transform obj_to_tag;
        tf2::fromMsg(des.objtransform, obj_to_tag);
        tf2::Transform tag_to_obj;
        tag_to_obj = obj_to_tag.inverse();
        tf2::Transform pub_to_tag;
        tf2::fromMsg(tag.pose.pose, pub_to_tag);
        tf2::Transform pub_to_obj = pub_to_tag * tag_to_obj;
        tf2::toMsg(pub_to_obj, out.pose);
        break;
      }
    }
    out.header.stamp = tag.pose.header.stamp;
    return out;
  }

  // returns tag's pose in optical frame, without any transformations
  geometry_msgs::PoseStamped AprilTagDetector::getTagPoseDetection(Eigen::Matrix3d R, Eigen::Vector3d t)
  {
    geometry_msgs::PoseStamped tag_pose_camera;
    tag_pose_camera.header.frame_id = img_frame_id_;
    tag_pose_camera.pose.position.x = t(0);
    tag_pose_camera.pose.position.y = t(1);
    tag_pose_camera.pose.position.z = t(2);
    Eigen::Quaterniond q(R);
    // // tag_pose_camera points along x axis of the tag right now, rotate it to point inside the tag
    // // rotate along y axis by 90 deg
    // Eigen::Vector3d rotation(0, -M_PI/2, 0);
    // double angle = rotation.norm();
    // Eigen::Vector3d axis = rotation.normalized();
    // q *= Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    geometry_msgs::Quaternion q_msg;
    q_msg.w = q.w();
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    tag_pose_camera.pose.orientation = q_msg;
    return tag_pose_camera;
  }

  void AprilTagDetector::updateCameraParams(double fx, double fy, double cx, double cy)
  {
    detection_info_.fx = fx;
    detection_info_.fy = fy;
    detection_info_.cx = cx;
    detection_info_.cy = cy;
  }

  AprilTagDetector::~AprilTagDetector()
  {
    apriltag_detections_destroy(detections_);
    apriltag_detector_destroy(detector_);
    // TODO: family type is hardcoded here
    tagStandard41h12_destroy(family_);
  }
} // namespace tag_detection
