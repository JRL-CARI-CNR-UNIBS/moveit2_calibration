/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/handeye_calibration_rviz_plugin/robot_model_provider.h>

#include <regex>

namespace moveit_rviz_plugin
{
namespace
{
// An empty SRDF for the given URDF, so link names are available before move_group publishes the real one.
std::string minimalSrdf(const std::string& urdf)
{
  std::smatch match;
  static const std::regex name_regex(R"(<robot[^>]*\sname\s*=\s*["']([^"']*)["'])");
  const std::string name = std::regex_search(urdf, match, name_regex) ? match[1].str() : "robot";
  return "<?xml version=\"1.0\"?><robot name=\"" + name + "\"/>";
}
}  // namespace

RobotModelProvider::RobotModelProvider(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  // Same QoS as robot_state_publisher and move_group use for these latched topics.
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  urdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_description", qos,
      [this](const std_msgs::msg::String::ConstSharedPtr& msg) { storeDescription(urdf_, msg->data); });
  srdf_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "robot_description_semantic", qos,
      [this](const std_msgs::msg::String::ConstSharedPtr& msg) { storeDescription(srdf_, msg->data); });
}

RobotModelProvider::~RobotModelProvider()
{
  urdf_sub_.reset();
  srdf_sub_.reset();
}

void RobotModelProvider::storeDescription(std::string& field, const std::string& value)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (field == value)
      return;
    field = value;
  }
  // Build the model on the GUI thread.
  QMetaObject::invokeMethod(this, "rebuildModel", Qt::QueuedConnection);
}

void RobotModelProvider::rebuildModel()
{
  std::string urdf, srdf;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    urdf = urdf_;
    srdf = srdf_;
  }
  if (urdf.empty())
    return;
  if (srdf.empty())
    srdf = minimalSrdf(urdf);

  robot_model_loader::RobotModelLoader::Options opt(urdf, srdf);
  opt.load_kinematics_solvers = false;  // planning is done by move_group
  auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, opt);
  if (!loader->getModel())
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to build the robot model from robot_description.");
    return;
  }
  loader_ = loader;
  RCLCPP_INFO(node_->get_logger(), "Robot model '%s' loaded (%zu planning groups).",
              loader_->getModel()->getName().c_str(), loader_->getModel()->getJointModelGroupNames().size());
  Q_EMIT robotModelChanged();
}

}  // namespace moveit_rviz_plugin
