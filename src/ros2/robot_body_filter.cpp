// SPDX-License-Identifier: BSD-3-Clause
#include <robot_body_filter/ros2/robot_body_filter.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>
#include <Eigen/Geometry>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

namespace robot_body_filter
{
namespace
{
Eigen::Isometry3d pose(const urdf::Pose& p)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  result.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return result;
}

std::unique_ptr<shapes::Shape> shape(const urdf::Geometry& geometry)
{
  switch (geometry.type) {
    case urdf::Geometry::SPHERE: {
      const auto& g = static_cast<const urdf::Sphere&>(geometry);
      return std::make_unique<shapes::Sphere>(g.radius);
    }
    case urdf::Geometry::BOX: {
      const auto& g = static_cast<const urdf::Box&>(geometry);
      return std::make_unique<shapes::Box>(g.dim.x, g.dim.y, g.dim.z);
    }
    case urdf::Geometry::CYLINDER: {
      const auto& g = static_cast<const urdf::Cylinder&>(geometry);
      return std::make_unique<shapes::Cylinder>(g.radius, g.length);
    }
    case urdf::Geometry::MESH: {
      const auto& g = static_cast<const urdf::Mesh&>(geometry);
      return std::unique_ptr<shapes::Shape>(shapes::createMeshFromResource(
          g.filename, Eigen::Vector3d(g.scale.x, g.scale.y, g.scale.z)));
    }
    default: throw std::runtime_error("Unsupported URDF collision geometry");
  }
}

struct Coordinate
{
  uint32_t offset;
  uint8_t datatype;
  double read(const uint8_t* point, bool big_endian) const
  {
    uint8_t bytes[8];
    const size_t size = datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
    std::memcpy(bytes, point + offset, size);
    const uint16_t endian_test = 1;
    const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&endian_test) == 0;
    if (big_endian != host_big_endian) std::reverse(bytes, bytes + size);
    if (size == 4) { float result; std::memcpy(&result, bytes, 4); return result; }
    double result; std::memcpy(&result, bytes, 8); return result;
  }
  void clear(uint8_t* point) const
  {
    // All-one IEEE float/double bits are NaN in either byte order.
    std::memset(point + offset, 0xff,
                datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8);
  }
};
Coordinate coordinate(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name)
{
  for (const auto& field : cloud.fields) {
    if (field.name != name) continue;
    const size_t size = field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
    if ((field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
         field.datatype != sensor_msgs::msg::PointField::FLOAT64) || field.count != 1 ||
        field.offset + size > cloud.point_step)
      throw std::runtime_error("Invalid point cloud coordinate field: " + name);
    return {field.offset, field.datatype};
  }
  throw std::runtime_error("Point cloud is missing coordinate: " + name);
}
}

struct RobotBodyFilterPointCloud2::Impl
{
  struct Collision
  {
    Eigen::Isometry3d origin;
    std::unique_ptr<bodies::Body> body;
  };
  struct Link
  {
    std::string frame;
    std::vector<Collision> collisions;
  };
  std::vector<Link> links;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  bool keep_organized = false;
};

RobotBodyFilterPointCloud2::RobotBodyFilterPointCloud2() : impl_(std::make_unique<Impl>()) {}
RobotBodyFilterPointCloud2::~RobotBodyFilterPointCloud2() = default;

bool RobotBodyFilterPointCloud2::configure()
{
  try {
    std::string description, prefix;
    std::vector<std::string> only, ignored;
    double padding, scale;
    getParam("robot_description", description);
    getParam("transforms.link_tf_prefix", prefix);
    getParam("body_model.only_links", only);
    getParam("body_model.ignore_links", ignored);
    getParam("body_model.inflation.padding", padding, true, 0.0);
    getParam("body_model.inflation.scale", scale, true, 1.0);
    getParam("filter.keep_clouds_organized", impl_->keep_organized, true, false);
    if (!std::isfinite(padding) || !std::isfinite(scale) || scale <= 0)
      throw std::runtime_error("Inflation requires finite padding and positive scale");
    urdf::Model model;
    if (!model.initString(description)) throw std::runtime_error("Cannot parse robot_description");
    impl_->links.clear();
    for (const auto& entry : model.links_) {
      const auto& link = entry.second;
      if ((!only.empty() && std::find(only.begin(), only.end(), link->name) == only.end()) ||
          std::find(ignored.begin(), ignored.end(), link->name) != ignored.end()) continue;
      auto collisions = link->collision_array;
      if (collisions.empty() && link->collision) collisions.push_back(link->collision);
      if (collisions.empty()) continue;
      double link_padding, link_scale;
      getParam("body_model.inflation.per_link." + link->name + ".padding", link_padding, true, padding);
      getParam("body_model.inflation.per_link." + link->name + ".scale", link_scale, true, scale);
      if (!std::isfinite(link_padding) || !std::isfinite(link_scale) || link_scale <= 0)
        throw std::runtime_error("Invalid inflation for " + link->name);
      Impl::Link body_link{prefix + link->name, {}};
      for (const auto& collision : collisions) {
        if (!collision || !collision->geometry) continue;
        auto geometry = shape(*collision->geometry);
        if (!geometry) throw std::runtime_error("Cannot load collision mesh for " + link->name);
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(geometry.get()));
        if (!body) throw std::runtime_error("Cannot construct collision body for " + link->name);
        body->setScale(link_scale);
        body->setPadding(link_padding);  // ConvexMesh implements padding in geometric_shapes.
        body_link.collisions.push_back({pose(collision->origin), std::move(body)});
      }
      if (!body_link.collisions.empty()) impl_->links.push_back(std::move(body_link));
    }
    if (impl_->links.empty()) throw std::runtime_error("No collision links selected");
    impl_->tf = std::make_unique<tf2_ros::Buffer>(impl_->clock);
    impl_->listener = std::make_unique<tf2_ros::TransformListener>(*impl_->tf);
    RCLCPP_INFO(logging_interface_->get_logger(),
                "Loaded %zu collision links; default padding %.3f m, scale %.3f",
                impl_->links.size(), padding, scale);
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Body filter configuration: %s", error.what());
    return false;
  }
}

bool RobotBodyFilterPointCloud2::update(const sensor_msgs::msg::PointCloud2& input,
                                       sensor_msgs::msg::PointCloud2& output)
{
  try {
    if (!configured_) return false;
    const auto x = coordinate(input, "x"), y = coordinate(input, "y"), z = coordinate(input, "z");
    if (input.row_step < uint64_t(input.width) * input.point_step ||
        input.data.size() < uint64_t(input.row_step) * input.height)
      throw std::runtime_error("Invalid PointCloud2 buffer dimensions");
    const rclcpp::Time stamp(input.header.stamp);
    // Do not publish an unfiltered cloud when a link transform is missing.
    // The node queues the cloud and retries after TF has arrived.
    for (auto& link : impl_->links) {
      const auto transform = impl_->tf->lookupTransform(input.header.frame_id, link.frame, stamp);
      const auto& q = transform.transform.rotation;
      const auto& t = transform.transform.translation;
      Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
      world.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
      world.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      for (auto& collision : link.collisions) collision.body->setPose(world * collision.origin);
    }
    output = input;
    if (!impl_->keep_organized) { output.data.clear(); output.data.reserve(input.data.size()); }
    for (size_t row = 0; row < input.height; ++row) {
      for (size_t column = 0; column < input.width; ++column) {
        const size_t offset = row * input.row_step + column * input.point_step;
        const auto* point = input.data.data() + offset;
        const Eigen::Vector3d position(x.read(point, input.is_bigendian),
                                       y.read(point, input.is_bigendian),
                                       z.read(point, input.is_bigendian));
        bool remove = !position.allFinite();
        if (!remove) {
          for (const auto& link : impl_->links) {
            for (const auto& collision : link.collisions) {
              if (collision.body->containsPoint(position)) { remove = true; break; }
            }
            if (remove) break;
          }
        }
        if (!remove && !impl_->keep_organized)
          output.data.insert(output.data.end(), point, point + input.point_step);
        if (remove && impl_->keep_organized) {
          x.clear(output.data.data() + offset);
          y.clear(output.data.data() + offset);
          z.clear(output.data.data() + offset);
          output.is_dense = false;
        }
      }
    }
    if (!impl_->keep_organized) {
      output.height = 1;
      output.width = output.data.size() / input.point_step;
      output.row_step = output.width * output.point_step;
      output.is_dense = true;
    }
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(logging_interface_->get_logger(), "Body filter update: %s", error.what());
    return false;
  }
}
}

PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterPointCloud2,
                      filters::FilterBase<sensor_msgs::msg::PointCloud2>)
