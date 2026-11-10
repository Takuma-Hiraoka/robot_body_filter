// SPDX-License-Identifier: BSD-3-Clause
#ifndef ROBOT_BODY_FILTER_ROS2_ROBOT_BODY_FILTER_HPP
#define ROBOT_BODY_FILTER_ROS2_ROBOT_BODY_FILTER_HPP

#include <memory>
#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robot_body_filter
{
// ROS 2 entry point. Uses geometric_shapes' scaled/padded collision bodies,
// including ConvexMesh, just as the original RobotBodyFilter implementation.
class RobotBodyFilterPointCloud2 : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
{
public:
  RobotBodyFilterPointCloud2();
  ~RobotBodyFilterPointCloud2() override;
  using filters::FilterBase<sensor_msgs::msg::PointCloud2>::configure;
  bool update(const sensor_msgs::msg::PointCloud2& input,
              sensor_msgs::msg::PointCloud2& output) override;

protected:
  bool configure() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
#endif
