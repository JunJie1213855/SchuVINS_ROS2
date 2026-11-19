#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace schurvins {
struct ColoredLandmark {
  float x = 0, y = 0, z = 0;
  int id = -1;
  uint8_t r = 128, g = 128, b = 128;
  bool color_valid = false;
  int64_t color_stamp = std::numeric_limits<int64_t>::min();
  int color_camera = 2;
};

// Own values only: no references to discarded keyframes or estimator points.
// Retain every observed finite landmark until the estimator starts a new map.
class LandmarkHistory {
 public:
  void update(const ColoredLandmark& point) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return;
    auto result = points_.try_emplace(point.id, point);
    if (result.second) return;
    auto& old = result.first->second;
    old.x = point.x; old.y = point.y; old.z = point.z;
    // Old keyframes may be visited after current observations. Never overwrite
    // a newer pixel sample; for a synchronized pair prefer cam0 on a tie.
    if (point.color_valid && (!old.color_valid || point.color_stamp > old.color_stamp ||
        (point.color_stamp == old.color_stamp && point.color_camera < old.color_camera))) {
      old.r = point.r; old.g = point.g; old.b = point.b;
      old.color_valid = true;
      old.color_stamp = point.color_stamp;
      old.color_camera = point.color_camera;
    }
  }
  std::vector<ColoredLandmark> snapshot() const {
    std::vector<ColoredLandmark> result;
    result.reserve(points_.size());
    for (const auto& item : points_) result.push_back(item.second);
    return result;
  }
  void clear() { points_.clear(); }
 private:
  std::unordered_map<int, ColoredLandmark> points_;
};
}
