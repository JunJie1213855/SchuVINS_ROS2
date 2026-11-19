#include <schurvins/landmark_history.hpp>
#include <stdexcept>
#include <limits>
#include <iostream>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
schurvins::ColoredLandmark find(const schurvins::LandmarkHistory& history, int id) {
  for (const auto& point : history.snapshot()) if (point.id == id) return point;
  throw std::runtime_error("Missing historical point");
}
int main() {
  schurvins::LandmarkHistory history;
  schurvins::ColoredLandmark a;
  a.id = 10; a.x = 1; a.y = 2; a.z = 3;
  a.r = a.g = a.b = 40; a.color_valid = true; a.color_stamp = 100; a.color_camera = 1;
  history.update(a);
  auto b = a; b.id = 20; b.x = 4;
  history.update(b); // A has left the current local map: retain it by value.
  require(history.snapshot().size() == 2 && find(history, 10).x == 1, "Lost historical point");
  a.x = 5; a.r = a.g = a.b = 80; a.color_stamp = 200;
  history.update(a);
  require(history.snapshot().size() == 2 && find(history, 10).x == 5 && find(history, 10).r == 80,
          "ID update must refresh position/color without duplication");
  a.color_stamp = 50; a.r = a.g = a.b = 1;
  history.update(a);
  require(find(history, 10).r == 80, "Old keyframe replaced newer image color");
  a.color_stamp = 200; a.color_camera = 0; a.r = a.g = a.b = 0;
  history.update(a);
  require(find(history, 10).r == 0, "Valid black pixel or cam0 tie priority was lost");
  a.color_valid = false; a.x = 6;
  history.update(a);
  require(find(history, 10).x == 6 && find(history, 10).r == 0, "Missing pixel must preserve previous color");
  a.x = std::numeric_limits<float>::quiet_NaN();
  history.update(a);
  require(find(history, 10).x == 6, "Nonfinite update replaced valid position");
  a.id = 30;
  history.update(a);
  require(history.snapshot().size() == 2, "Nonfinite new point was retained");
  history.clear();
  require(history.snapshot().empty(), "New coordinate frame must clear old history");
  std::cout << "PASS: historical retention, ID updates, grayscale color/timestamp priority, finite filtering, reset\n";
}
