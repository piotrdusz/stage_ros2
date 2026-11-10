#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stage_ros2::StageNode>(rclcpp::NodeOptions());
  node->init(argc - 1, argv);
  if (node->SubscribeModels() != 0) {exit(-1);}
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread t([&executor]() {executor.spin();});
  node->world->Start();
  Stg::World::Run();
  executor.cancel();
  t.join();
  return 0;
}
