// ROS 2 adapter for SchurVINS. Upstream algorithm and licenses are retained.
#include <schurvins/factory.hpp>
#include <schurvins/visualization.hpp>
#include <svo/direct/depth_filter.h>
#include <schurvins/parameters.hpp>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>
#include <svo/common/frame.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <deque>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>

namespace schurvins {
class StereoNode : public rclcpp::Node {
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ExactTime<Image, Image>;
  struct ImagePair { Image::ConstSharedPtr left, right; };
 public:
  StereoNode() : Node("svo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    const auto calib = param<std::string>(*this, "calib_file", "");
    if (calib.empty()) throw std::runtime_error("calib_file is required; use euroc_vio_stereo.launch.py");
    if (!std::filesystem::is_regular_file(calib)) throw std::runtime_error("Calibration file does not exist: " + calib);
    if (!param<bool>(*this, "use_imu", true)) throw std::runtime_error("SchurVINS requires use_imu=true");
    for (const auto* option : {"runlc", "use_ceres_backend", "use_global_map"})
      if (param<bool>(*this, option, false)) throw std::runtime_error(std::string(option) + " is not ported to ROS 2");
    world_ = param<std::string>(*this, "world_frame", "world");
    body_ = param<std::string>(*this, "body_frame", "imu");
    if (world_.empty() || body_.empty() || world_ == body_) throw std::runtime_error("world_frame and body_frame must be distinct and nonempty");
    publish_tf_ = param<bool>(*this, "publish_tf", true);
    auto_restart_ = param<bool>(*this, "automatic_reinitialization", true);
    max_queue_ = param<int>(*this, "image_queue_size", 30);
    max_path_ = param<int>(*this, "path_max_poses", 10000);
    const auto sync_queue = param<int>(*this, "sync_queue_size", 30);
    if (max_queue_ < 1 || max_path_ < 1 || sync_queue < 1) throw std::runtime_error("queue sizes and path_max_poses must be positive");
    // Keep processing and IMU insertion on a single executor. Pending images are
    // only passed to the algorithm once the IMU buffer covers their timestamp.
    initializePipeline();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    visualization_ = std::make_unique<Visualization>(*this, world_);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto trajectory = param<std::string>(*this, "trajectory_file", "");
    if (!trajectory.empty()) {
      trajectory_.open(trajectory);
      if (!trajectory_) throw std::runtime_error("Cannot open trajectory_file: " + trajectory);
      trajectory_ << std::setprecision(17);
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(param<std::string>(*this, "imu_topic", "/imu0"),
        rclcpp::SensorDataQoS().keep_last(2000), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (stamp <= latest_imu_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping non-increasing IMU timestamp; restart the node after rewinding a bag");
        return;
      }
      latest_imu_ = stamp;
      imu_->addImuMeasurement(svo::ImuMeasurement(stamp * 1e-9,
          Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
          Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z)));
      processPending();
    });
    left_.subscribe(this, param<std::string>(*this, "cam0_topic", "/cam0/image_raw"), rmw_qos_profile_sensor_data);
    right_.subscribe(this, param<std::string>(*this, "cam1_topic", "/cam1/image_raw"), rmw_qos_profile_sensor_data);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(sync_queue), left_, right_);
    sync_->registerCallback(&StereoNode::imageCallback, this);
    command_sub_ = create_subscription<std_msgs::msg::String>("~/remote_key", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
      if (msg->data == "r") { paused_ = true; resetPipeline(); }
      else if (msg->data == "s" && paused_) { paused_ = false; vo_->start(); }
      else if (msg->data == "q") rclcpp::shutdown();
      else if (msg->data == "c") vo_->setCompensation(true);
      else if (msg->data == "C") vo_->setCompensation(false);
    });
    RCLCPP_INFO(get_logger(), "SchurVINS stereo + IMU ready; waiting for synchronized images and IMU");
  }
 private:
  int64_t imageTimestamp(const Image& image) const {
    return rclcpp::Time(image.header.stamp).nanoseconds() - camera_imu_delay_ns_;
  }
  void imageCallback(const Image::ConstSharedPtr& left, const Image::ConstSharedPtr& right) {
    const int64_t stamp = imageTimestamp(*left);
    if (stamp <= last_image_) return;
    last_image_ = stamp;
    pending_.push_back({left, right});
    if (pending_.size() > static_cast<size_t>(max_queue_)) {
      pending_.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image queue full; check IMU topic, timestamps and playback rate");
    }
    processPending();
  }
  void initializePipeline() {
    vo_ = svo::factory::makeStereo(*this);
    if (vo_->getNCamera()->numCameras() != 2) throw std::runtime_error("Stereo calibration must contain two cameras");
    imu_ = svo::factory::getImuHandler(*this);
    // Put camera frames and IMU samples on the same clock before calling the
    // Schur filter. Its Forward() integrates to frame timestamps directly.
    // Clear the handler offset so getMeasurements/rotation priors do not apply it twice.
    camera_imu_delay_ns_ = std::llround(imu_->imu_calib_.delay_imu_cam * 1e9);
    imu_->imu_calib_.delay_imu_cam = 0.0;
    RCLCPP_INFO(get_logger(), "Camera-to-IMU timestamp correction: %+.9f s (applied once)", -camera_imu_delay_ns_ * 1e-9);
    vo_->setImuHandler(imu_);
    vo_->start();
  }
  void resetPipeline() {
    // Reconstruct the filter as well as the frontend; upstream reset() alone
    // does not reconstruct SchurVINS's covariance and IMU propagation state.
    pending_.clear();
    path_.poses.clear();
    visualization_->reset();
    initializePipeline();
  }
  void processPending() {
    while (!pending_.empty()) {
      const auto pair = pending_.front();
      const int64_t stamp = imageTimestamp(*pair.left);
      if (latest_imu_ == std::numeric_limits<int64_t>::min() ||
          latest_imu_ * 1e-9 < stamp * 1e-9 - imu_->imu_calib_.delay_imu_cam) return;
      const auto measurements = imu_->getMeasurementsCopy();
      if (measurements.size() < 2) return;
      if (measurements.back().timestamp_ >= stamp * 1e-9 - imu_->imu_calib_.delay_imu_cam) {
        pending_.pop_front();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping image without an older IMU measurement");
        continue;
      }
      pending_.pop_front();
      if (paused_) continue;
      if (!vo_->hasStarted()) {
        svo::Quaternion attitude;
        if (!imu_->getInitialAttitude(stamp * 1e-9, attitude)) continue;
        vo_->setRotationPrior(attitude);
      } else if (vo_->getLastFrames()) {
        svo::Quaternion rotation;
        if (imu_->getRelativeRotationPrior(vo_->getLastFrames()->getMinTimestampSeconds(), stamp * 1e-9, false, rotation))
          vo_->setRotationIncrementPrior(rotation);
      }
      cv_bridge::CvImageConstPtr left, right;
      try {
        left = cv_bridge::toCvShare(pair.left, "mono8");
        right = cv_bridge::toCvShare(pair.right, "mono8");
      } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Image conversion failed: %s", e.what());
        continue;
      }
      const auto cameras = vo_->getNCamera();
      if (left->image.cols != static_cast<int>(cameras->getCamera(0).imageWidth()) ||
          left->image.rows != static_cast<int>(cameras->getCamera(0).imageHeight()) ||
          right->image.cols != static_cast<int>(cameras->getCamera(1).imageWidth()) ||
          right->image.rows != static_cast<int>(cameras->getCamera(1).imageHeight())) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image dimensions do not match calibration");
        continue;
      }
      vo_->addImageBundle({left->image, right->image}, stamp);
      visualization_->publish(vo_->getLastFrames(), vo_->map(), vo_->depth_filter_->feature_detector_mut_,
                              vo_->stage() == svo::Stage::kTracking);
      if (vo_->stage() == svo::Stage::kTracking) publish(stamp);
      if (vo_->stage() == svo::Stage::kPaused && auto_restart_) {
        resetPipeline();
        return;
      }
    }
  }
  void publish(int64_t stamp) {
    const auto pose = vo_->getLastFrames()->get_T_W_B();
    const auto position = pose.getPosition();
    const auto rotation = pose.getEigenQuaternion();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(stamp);
    odom.header.frame_id = world_;
    odom.child_frame_id = body_;
    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.w = rotation.w();
    odom.pose.pose.orientation.x = rotation.x();
    odom.pose.pose.orientation.y = rotation.y();
    odom.pose.pose.orientation.z = rotation.z();
    // The upstream public API does not expose filter covariance or velocity.
    // Mark unavailable quantities instead of claiming zero uncertainty.
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 7] = 1e6;
      odom.twist.covariance[i * 7] = 1e6;
    }
    odom_pub_->publish(odom);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose = odom.pose.pose;
    path_.header = odom.header;
    path_.poses.push_back(pose_msg);
    if (path_.poses.size() > static_cast<size_t>(max_path_)) path_.poses.erase(path_.poses.begin());
    path_pub_->publish(path_);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = body_;
      transform.transform.translation.x = position.x();
      transform.transform.translation.y = position.y();
      transform.transform.translation.z = position.z();
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(transform);
    }
    if (trajectory_) {
      trajectory_ << stamp * 1e-9 << ' ' << position.transpose() << ' '
          << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z() << ' ' << rotation.w() << '\n';
    }
  }
  std::unique_ptr<Visualization> visualization_;
  std::shared_ptr<svo::FrameHandlerStereo> vo_;
  svo::ImuHandler::Ptr imu_;
  message_filters::Subscriber<Image> left_, right_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::deque<ImagePair> pending_;
  nav_msgs::msg::Path path_;
  std::ofstream trajectory_;
  std::string world_, body_;
  int64_t camera_imu_delay_ns_ = 0;
  int64_t latest_imu_ = std::numeric_limits<int64_t>::min();
  int64_t last_image_ = std::numeric_limits<int64_t>::min();
  int max_queue_, max_path_;
  bool publish_tf_, auto_restart_, paused_ = false;
};
}
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<schurvins::StereoNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("schurvins"), "%s", e.what());
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return result;
}
