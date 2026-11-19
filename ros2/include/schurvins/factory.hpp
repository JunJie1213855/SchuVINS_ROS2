#pragma once
#include <rclcpp/rclcpp.hpp>
#include <svo/common/camera_fwd.h>
namespace svo {
class ImuHandler;
class FrameHandlerMono;
class FrameHandlerStereo;
namespace factory {
std::shared_ptr<ImuHandler> getImuHandler(rclcpp::Node& node);
std::shared_ptr<FrameHandlerMono> makeMono(rclcpp::Node& node, const CameraBundlePtr& cam = nullptr);
std::shared_ptr<FrameHandlerStereo> makeStereo(rclcpp::Node& node, const CameraBundlePtr& cam = nullptr);
}
}
