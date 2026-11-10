#include "stage_ros2/stage_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace stage_ros2 {

using namespace std::chrono_literals;

void StageNode::random_reset_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (vehicles_.size() != 1) {
    response->message = "Random reset requires exactly one robot";
    return;
  }
  if (!amcl_state_client_->service_is_ready() || !amcl_pose_client_->service_is_ready()) {
    response->message = "AMCL services are unavailable";
    return;
  }
  auto state = amcl_state_client_->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());
  if (state.wait_for(2s) != std::future_status::ready) {
    amcl_state_client_->remove_pending_request(state);
    response->message = "Timed out checking AMCL state";
    return;
  }
  if (state.get()->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    response->message = "AMCL is not active";
    return;
  }

  if (stop_follower_client_->service_is_ready()) {
    auto stop_result = stop_follower_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (stop_result.wait_for(2s) != std::future_status::ready) {
      stop_follower_client_->remove_pending_request(stop_result);
      response->message = "Timed out stopping the path follower";
      return;
    }
    const auto stop_response = stop_result.get();
    if (!stop_response->success) {
      response->message = "Could not stop the path follower: " + stop_response->message;
      return;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Path follower stop service unavailable; clearing /path without acknowledgement");
  }
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "map";
  {
    std::scoped_lock lock(msg_lock);
    empty_path.header.stamp = sim_time_;
  }
  clear_path_pub_->publish(empty_path);

  auto request = std::make_shared<RandomReset>();
  auto result = request->result.get_future();
  {
    std::scoped_lock lock(msg_lock);
    if (!map_) {
      response->message = "No map received";
      return;
    }
    if (pending_random_reset_) {
      response->message = "A random reset is already pending";
      return;
    }
    request->map = map_;
    pending_random_reset_ = request;
  }

  if (result.wait_for(5s) != std::future_status::ready) {
    std::scoped_lock lock(msg_lock);
    if (pending_random_reset_ == request) {
      pending_random_reset_.reset();
      response->message = "Timed out waiting for Stage to update; is it paused?";
      return;
    }
  }
  const auto pose = result.get();
  if (!pose) {
    response->message = "Could not find a collision-free pose in the map";
    return;
  }

  auto initial_pose = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  initial_pose->pose.header.frame_id = "map";
  {
    std::scoped_lock lock(msg_lock);
    initial_pose->pose.header.stamp = sim_time_;
  }
  initial_pose->pose.pose.pose.position.x = pose->x;
  initial_pose->pose.pose.pose.position.y = pose->y;
  initial_pose->pose.pose.pose.orientation = createQuaternionMsgFromYaw(pose->a);
  initial_pose->pose.pose.covariance[0] = 0.0025;
  initial_pose->pose.pose.covariance[7] = 0.0025;
  initial_pose->pose.pose.covariance[35] = 0.0025;
  auto amcl_result = amcl_pose_client_->async_send_request(initial_pose);
  if (amcl_result.wait_for(2s) != std::future_status::ready) {
    amcl_pose_client_->remove_pending_request(amcl_result);
    response->message = "Robot moved, but AMCL pose update timed out";
    return;
  }
  amcl_result.get();
  response->success = true;
  std::ostringstream message;
  message << "Robot teleported and AMCL initialized: x=" << pose->x <<
    ", y=" << pose->y << ", yaw=" << pose->a;
  response->message = message.str();
}

std::optional<Stg::Pose> StageNode::random_reset(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & info = map.info;
  const double resolution = info.resolution;
  const size_t width = info.width;
  const size_t height = info.height;
  if (map.header.frame_id != "map" || !std::isfinite(resolution) || resolution <= 0.0 ||
    width == 0 || height == 0 || width > map.data.size() / height ||
    width > static_cast<size_t>(std::numeric_limits<int>::max()) ||
    height > static_cast<size_t>(std::numeric_limits<int>::max()) ||
    !std::isfinite(info.origin.position.x) || !std::isfinite(info.origin.position.y))
  {
    RCLCPP_ERROR(get_logger(), "Invalid map for random reset");
    return std::nullopt;
  }

  const double origin_yaw = tf2::getYaw(info.origin.orientation);
  if (!std::isfinite(origin_yaw)) {
    RCLCPP_ERROR(get_logger(), "Invalid map origin orientation");
    return std::nullopt;
  }
  const double radius = reset_clearance_ + resolution * std::sqrt(2.0) / 2.0;
  if (!std::isfinite(radius) || radius / resolution >= std::min(width, height) / 2.0) {
    return std::nullopt;
  }
  const int cells = static_cast<int>(std::ceil(radius / resolution));
  if (width <= 2 * static_cast<size_t>(cells) || height <= 2 * static_cast<size_t>(cells)) {
    return std::nullopt;
  }
  std::uniform_int_distribution<int> column(cells, static_cast<int>(width) - cells - 1);
  std::uniform_int_distribution<int> row(cells, static_cast<int>(height) - cells - 1);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  auto * model = vehicles_.front()->positionmodel;
  const auto original = model->GetGlobalPose();

  for (int attempt = 0; attempt < 2000; ++attempt) {
    const int x = column(random_);
    const int y = row(random_);
    bool free = true;
    for (int dy = -cells; dy <= cells && free; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        if (std::hypot(dx * resolution, dy * resolution) <= radius &&
          map.data[static_cast<size_t>(y + dy) * width + static_cast<size_t>(x + dx)] != 0)
        {
          free = false;
          break;
        }
      }
    }
    if (!free) {
      continue;
    }
    const double local_x = (x + 0.5) * resolution;
    const double local_y = (y + 0.5) * resolution;
    const Stg::Pose candidate(
      info.origin.position.x + std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y,
      info.origin.position.y + std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y,
      original.z, angle(random_));
    model->SetGlobalPose(candidate);
    if (!model->HasCollision()) {
      vehicles_.front()->prepare_for_teleport();
      return candidate;
    }
  }
  model->SetGlobalPose(original);
  return std::nullopt;
}

}  // namespace stage_ros2
