#pragma once
#include <rclcpp/rclcpp.hpp>
#include <schurvins/landmark_history.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <opencv2/core.hpp>
#include <array>
#include <mutex>
#include <unordered_map>
#include <string>

namespace schurvins {
// Publishes actual feature observations and all historical colored landmarks.
class Visualization {
 public:
  Visualization(rclcpp::Node& node, const std::string& world);
  void publish(const svo::FrameBundlePtr& frames, const svo::Map::Ptr& map,
               std::mutex& depth_filter_mutex, bool tracking);
  void reset();
 private:
  struct Feature {
    cv::Point2f pixel;
    int id;
    svo::FeatureType type;
    bool landmark;
  };
  void publishCloud(const std::vector<ColoredLandmark>& points, int64_t stamp);
  LandmarkHistory history_;
  std::string world_;
  int64_t last_stamp_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracked_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 2> cameras_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_;
  std::array<std::unordered_map<int, cv::Point2f>, 2> previous_;
};
}
