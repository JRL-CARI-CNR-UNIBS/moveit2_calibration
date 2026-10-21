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

#pragma once

#include <QObject>

#ifndef Q_MOC_RUN
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#endif

namespace moveit_rviz_plugin
{
/**
 * Loads the robot model from the robot_description / robot_description_semantic
 * topics without blocking the GUI.
 *
 * MoveIt's RobotModelLoader waits on the GUI thread (10 s per topic) when no
 * description is available. Here the topics are received in the background and
 * the model is built on the GUI thread only once the URDF has arrived.
 * robotModelChanged() is emitted on the GUI thread every time the model is (re)built.
 */
class RobotModelProvider : public QObject
{
  Q_OBJECT
public:
  explicit RobotModelProvider(const rclcpp::Node::SharedPtr& node);
  ~RobotModelProvider() override;

  // Null until a valid URDF has been received. GUI thread only.
  robot_model_loader::RobotModelLoaderPtr getLoader() const
  {
    return loader_;
  }

  // Null until a valid URDF has been received. GUI thread only.
  moveit::core::RobotModelConstPtr getModel() const
  {
    return loader_ ? loader_->getModel() : moveit::core::RobotModelConstPtr();
  }

Q_SIGNALS:
  void robotModelChanged();

private Q_SLOTS:
  void rebuildModel();

private:
  // Called from the executor thread.
  void storeDescription(std::string& field, const std::string& value);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr srdf_sub_;

  std::mutex mutex_;
  std::string urdf_;
  std::string srdf_;

  robot_model_loader::RobotModelLoaderPtr loader_;
};

using RobotModelProviderPtr = std::shared_ptr<RobotModelProvider>;

}  // namespace moveit_rviz_plugin
