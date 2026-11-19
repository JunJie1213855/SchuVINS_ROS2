#include <schurvins/visualization.hpp>
#include <svo/common/point.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>

namespace schurvins {
Visualization::Visualization(rclcpp::Node& node, const std::string& world) : world_(world) {
  tracked_ = node.create_publisher<sensor_msgs::msg::Image>("~/tracked_image", 2);
  for (size_t i = 0; i < cameras_.size(); ++i)
    cameras_[i] = node.create_publisher<sensor_msgs::msg::Image>("~/cam" + std::to_string(i) + "/tracked_image", 2);
  landmarks_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/landmarks", rclcpp::QoS(1).reliable().transient_local());
}

void Visualization::reset() {
  for (auto& tracks : previous_) tracks.clear();
  history_.clear();
  publishCloud({}, last_stamp_);
}

void Visualization::publish(const svo::FrameBundlePtr& frames, const svo::Map::Ptr& map,
                            std::mutex& depth_filter_mutex, bool tracking) {
  if (!frames || frames->size() != 2) return;
  std::array<cv::Mat, 2> images;
  std::array<std::vector<Feature>, 2> features;
  std::vector<ColoredLandmark> observations;
  last_stamp_ = frames->getMinTimestampNanoseconds();
  {
    // Seed initialization can append/reallocate feature arrays in the worker.
    // Copy a snapshot while both worker initialization and updates are excluded;
    // release the lock before image drawing and DDS publication.
    std::lock_guard<std::mutex> lock(depth_filter_mutex);
    const auto collect = [&](const svo::FramePtr& frame) {
      for (size_t i = 0; i < frame->numFeatures(); ++i) {
        const auto& point = frame->landmark_vec_[i];
        if (!point || svo::isSeed(frame->type_vec_[i]) ||
            frame->type_vec_[i] == svo::FeatureType::kOutlier || !point->pos_.allFinite()) continue;
        const Eigen::Vector3f position = point->pos_.cast<float>();
        if (!position.allFinite()) continue;
        ColoredLandmark sample;
        sample.x = position.x(); sample.y = position.y(); sample.z = position.z();
        sample.id = point->id_;
        const auto pixel = frame->px_vec_.col(i);
        const auto& raw = frame->img();
        // TideSense supplies MONO8. Encode its actual source intensity as
        // RGB=(gray,gray,gray), before any feature annotations are drawn.
        if (raw.type() == CV_8UC1 && pixel.allFinite() && pixel.x() >= 0 && pixel.y() >= 0 &&
            pixel.x() < raw.cols && pixel.y() < raw.rows) {
          const int u = std::min(raw.cols - 1, static_cast<int>(std::lround(pixel.x())));
          const int v = std::min(raw.rows - 1, static_cast<int>(std::lround(pixel.y())));
          sample.r = sample.g = sample.b = raw.at<uint8_t>(v, u);
          sample.color_valid = true;
          sample.color_stamp = frame->timestamp_;
          sample.color_camera = frame->cam().get() == frames->at(0)->cam().get() ? 0 : 1;
        }
        observations.push_back(sample);
      }
    };
    for (size_t camera = 0; camera < 2; ++camera) {
      const auto frame = frames->at(camera);
      images[camera] = frame->img().clone();
      features[camera].reserve(frame->numFeatures());
      for (size_t i = 0; i < frame->numFeatures(); ++i) {
        const auto pixel = frame->px_vec_.col(i);
        if (!pixel.allFinite() || pixel.x() < 0 || pixel.y() < 0 ||
            pixel.x() >= images[camera].cols || pixel.y() >= images[camera].rows) continue;
        const auto& point = frame->landmark_vec_[i];
        features[camera].push_back({cv::Point2f(pixel.x(), pixel.y()), point ? point->id_ : -1,
                                   frame->type_vec_[i], bool(point)});
      }
      if (tracking) collect(frame);
    }
    if (tracking && map)
      for (const auto& item : map->keyframes_) collect(item.second);
  }
  std::array<cv::Mat, 2> overlays;
  const bool show_combined = tracked_->get_subscription_count() > 0;
  for (size_t camera = 0; camera < 2; ++camera) {
    std::unordered_map<int, cv::Point2f> current;
    const bool draw = show_combined || cameras_[camera]->get_subscription_count() > 0;
    if (draw) cv::cvtColor(images[camera], overlays[camera], cv::COLOR_GRAY2BGR);
    int landmarks = 0, seeds = 0;
    for (const auto& feature : features[camera]) {
      const bool outlier = feature.type == svo::FeatureType::kOutlier;
      const bool seed = svo::isSeed(feature.type);
      if (!outlier && feature.landmark && !seed) ++landmarks;
      else if (!outlier && seed) ++seeds;
      if (feature.id >= 0 && !outlier) current[feature.id] = feature.pixel;
      if (!draw) continue;
      cv::Scalar color(0, 255, 0);  // landmark
      if (outlier) color = cv::Scalar(0, 0, 255);
      else if (svo::isUnconvergedSeed(feature.type)) color = cv::Scalar(0, 255, 255);
      else if (seed) color = cv::Scalar(255, 255, 0);
      else if (!feature.landmark) color = cv::Scalar(255, 128, 0);
      const auto old = previous_[camera].find(feature.id);
      if (!outlier && feature.id >= 0 && old != previous_[camera].end())
        cv::line(overlays[camera], old->second, feature.pixel, color, 1, cv::LINE_AA);
      cv::circle(overlays[camera], feature.pixel, 3, color, 1, cv::LINE_AA);
    }
    previous_[camera] = std::move(current);
    if (draw) {
      const std::string text = "cam" + std::to_string(camera) + "  landmarks: " + std::to_string(landmarks) +
          "  seeds: " + std::to_string(seeds);
      cv::rectangle(overlays[camera], cv::Point(0, 0), cv::Point(overlays[camera].cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(overlays[camera], text, cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(last_stamp_);
      header.frame_id = "cam" + std::to_string(camera);
      if (cameras_[camera]->get_subscription_count() > 0)
        cameras_[camera]->publish(*cv_bridge::CvImage(header, "bgr8", overlays[camera]).toImageMsg());
    }
  }
  if (show_combined) {
    // Keep cam0 on the left and cam1 on the right; no flips or swaps are made.
    // TideSense's physical right camera is cam0, as indicated by the labels.
    cv::Mat combined;
    cv::hconcat(overlays[0], overlays[1], combined);
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(last_stamp_);
    header.frame_id = "cam0";
    tracked_->publish(*cv_bridge::CvImage(header, "bgr8", combined).toImageMsg());
  }
  for (const auto& point : observations) history_.update(point);
  publishCloud(history_.snapshot(), last_stamp_);
}

void Visualization::publishCloud(const std::vector<ColoredLandmark>& points, int64_t stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = world_;
  cloud.header.stamp = rclcpp::Time(stamp);
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32, "id", 1, sensor_msgs::msg::PointField::INT32);
  modifier.resize(points.size());
  if (points.empty()) { landmarks_->publish(cloud); return; }
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
  sensor_msgs::PointCloud2Iterator<int32_t> id(cloud, "id");
  for (const auto& point : points) {
    *x = point.x; *y = point.y; *z = point.z;
    *r = point.r; *g = point.g; *b = point.b;
    *id = point.id;
    ++x; ++y; ++z; ++r; ++g; ++b; ++id;
  }
  landmarks_->publish(cloud);
}
}
