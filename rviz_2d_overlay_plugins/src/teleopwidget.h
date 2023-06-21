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
#include <std_msgs/msg/string.hpp>
#include <dump_messages/msg/task_posting_req.hpp>
#include <dump_messages/msg/task_posting_res.hpp>

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

  void task_posting_res(const dump_messages::msg::TaskPostingRes::SharedPtr msg);
  void send_start_signal();

  // The ROS publisher for chat server.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chat_publisher_;


  // The ROS publisher and subscriber for the task posting.
  rclcpp::Publisher<dump_messages::msg::TaskPostingReq>::SharedPtr task_publisher_;
  rclcpp::Subscription<dump_messages::msg::TaskPostingRes>::SharedPtr task_posting_res_subscriber_;
  std::vector<geometry_msgs::msg::Point> loading_sites_;
  std::vector<geometry_msgs::msg::Point> disposal_sites_;
  std::vector<float> deadlines_;
  std::vector<float> amount_of_sands_;

  // Dictonary to save excavator id and its score
  std::map<int, std::vector<float>> excavator_scores_;

  // Timer for start signal
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  Mode mode_;
};
}  // namespace rviz_2d_overlay_plugins
#endif  // TELEOPWIDGET_H
