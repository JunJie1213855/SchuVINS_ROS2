#pragma once
#include <rclcpp/rclcpp.hpp>
#include <type_traits>
namespace schurvins {
// ROS 1 accepted integer YAML values for floating point settings. Keep that
// behavior while preserving strict typing for booleans, strings and integers.
template<class T>
T param(rclcpp::Node& node, const std::string& name, const T& fallback) {
  if (!node.has_parameter(name)) node.declare_parameter<T>(name, fallback);
  const auto value = node.get_parameter(name);
  if constexpr (std::is_same_v<T, double>) {
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      return static_cast<double>(value.as_int());
    return value.as_double();
  } else if constexpr (std::is_same_v<T, int>) {
    return static_cast<int>(value.as_int());
  } else {
    return value.get_value<T>();
  }
}
}
