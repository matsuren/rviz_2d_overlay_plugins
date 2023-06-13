// Modified from https://github.com/Kotakku/sample_rviz_plugins/blob/main/src/twist_panel.hpp
#ifndef TELEOPWIDGET_H
#define TELEOPWIDGET_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QWidget>
#include "touchwidget.h"
#endif

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
class TeleopWidget;
}
QT_END_NAMESPACE

namespace rviz_2d_overlay_plugins
{
enum class Mode
{
  INACTIVE,
  AUTO,
  TELEOP
};
class TeleopWidget : public rviz_common::Panel
{
  Q_OBJECT

public:
  TeleopWidget(QWidget* parent = nullptr);
  ~TeleopWidget();

  virtual void onInitialize();

public Q_SLOTS:
  void tick();
  void clicked_main_btn();
  void clicked_chat_btn();
  void start_auto();
  void start_teleop(std::string target_name);
  void cancel_teleop();

private:
  Ui::TeleopWidget* ui;
  TouchWidget* touch_;

  // The ROS node handle.
  // ros::NodeHandle nh_;
  rclcpp::Node::SharedPtr nh_;

  // The ROS publisher for the command velocity.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

  // The ROS publisher for the ask for help from dump
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ask_for_help_subscriber_;
  void ask_for_help_callback(const std_msgs::msg::Int16::SharedPtr msg);
  std::string target_du_name_;


  Mode mode_;
};
}  // namespace rviz_2d_overlay_plugins
#endif  // TELEOPWIDGET_H
