#include <tag_master/apriltag_detector.h>

namespace tag_detection
{
  AprilTagDetector::AprilTagDetector(std::string name, double quad_decimate, double quad_sigma, int nthreads,
                                     bool refine_edges, double camera_fx, double camera_fy, double camera_cx,
                                     double camera_cy, double tag_size, bool initial_enable,
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

    detection_info_.fx = camera_fx;
    detection_info_.fy = camera_fy;
    detection_info_.cx = camera_cx;
    detection_info_.cy = camera_cy;
    detection_info_.tagsize = tag_size;

    enable_orthogonal_iteration_ = enable_orthogonal_iteration;
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
    // TODO: add mutex for detections
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
    // TODO: add mutex for poses
    bool success = true;

    std::vector<apriltag_pose_t> poses;
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);

      detection_info_.det = det;

      apriltag_pose_t pose;
      double err = estimate_tag_pose(&detection_info_, &pose);
      poses.push_back(pose);

      if (enable_orthogonal_iteration_)
      {
        apriltag_pose_t pose1;
        apriltag_pose_t pose2;
        double err1;
        double err2;
        estimate_tag_pose_orthogonal_iteration(&detection_info_, &err1, &pose1, &err2, &pose2, 3);
        poses_orthogonal_iteration_.clear();
        if (pose2.R)
        {
          // sometimes orthogonal iteration returns only one output
          poses_orthogonal_iteration_.push_back(std::pair<apriltag_pose_t, apriltag_pose_t>(pose1, pose2));
        }
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
    // TODO: add mutex for poses
    return poses_;
  }

  zarray *AprilTagDetector::getDetections()
  {
    // TODO: add mutex for detections
    return detections_;
  }

  void AprilTagDetector::drawDetections(cv::Mat &frame)
  {
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

      cv::line(frame, p1, p2, cv::Scalar(100, 180, 0), 3);
      cv::line(frame, p1, p4, cv::Scalar(100, 180, 0), 3);
      cv::line(frame, p2, p3, cv::Scalar(100, 180, 0), 3);
      cv::line(frame, p3, p4, cv::Scalar(100, 180, 0), 3);
      cv::circle(frame, c, 2, cv::Scalar(0, 0, 255), 3);
    }
  }

  void AprilTagDetector::drawCubes(cv::Mat &frame)
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
      drawCube(cube, frame, K, distortion, R, t, color);

      if (enable_orthogonal_iteration_)
      {
        if (poses_orthogonal_iteration_.size() > k)
        {
          cv::Mat R1 = convertToMat(poses_orthogonal_iteration_[k].first.R);
          // printMat(R1, "R1");
          cv::Mat R2 = convertToMat(poses_orthogonal_iteration_[k].second.R);
          // printMat(R2, "R2");
          cv::Mat t1 = convertToMat(poses_orthogonal_iteration_[k].first.t);
          // printMat(t1, "t1");
          cv::Mat t2 = convertToMat(poses_orthogonal_iteration_[k].second.t);
          // printMat(t2, "t2");
          auto color1 = cv::Scalar(255, 0, 0);
          drawCube(cube, frame, K, distortion, R1, t1, color1);
          auto color2 = cv::Scalar(0, 0, 255);
          drawCube(cube, frame, K, distortion, R2, t2, color2);
        }
      }
    }
  }

  DetectionOutput AprilTagDetector::output()
  {
    DetectionOutput out;
    for (size_t i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det);
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
      detection.tag.tf = getTf();
      out.detection.push_back(detection);
    }
  }

  geometry_msgs::TransformStamped AprilTagDetector::getTf()
  {
    /* TODO */
    geometry_msgs::TransformStamped tf;
    return tf;
  }
} // namespace tag_detection
