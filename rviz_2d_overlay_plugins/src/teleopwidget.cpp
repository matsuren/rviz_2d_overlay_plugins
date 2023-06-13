// Modified from https://github.com/Kotakku/sample_rviz_plugins/blob/main/src/twist_panel.cpp
#include "teleopwidget.h"
#include "ui_teleopwidget.h"
#include "touchwidget.h"
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <QLabel>
#include <QTimer>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>
using std::placeholders::_1;

namespace rviz_2d_overlay_plugins
{
TeleopWidget::TeleopWidget(QWidget* parent) : rviz_common::Panel(parent), ui(new Ui::TeleopWidget)
{
  mode_ = Mode::INACTIVE;
  ui->setupUi(this);
  touch_ = new TouchWidget();
  touch_->setEnabled(false);
  ui->layout->addWidget(touch_);

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);

  connect(ui->btn, SIGNAL(clicked()), this, SLOT(clicked_main_btn()));
  ui->status_line->setText(QString("Not active"));
  ui->btn->setStyleSheet("QPushButton {background-color:  white}");
}

TeleopWidget::~TeleopWidget()
{
  delete ui;
}

void TeleopWidget::onInitialize()
{
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  ask_for_help_subscriber_ = nh_->create_subscription<std_msgs::msg::Int16>(
      "/ask_for_help", rclcpp::QoS(10).reliable(), std::bind(&TeleopWidget::ask_for_help_callback, this, _1));
}

void TeleopWidget::ask_for_help_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rviz_plugin"), "Ask for help from dump: " + std::to_string(msg->data));
  ui->btn->setStyleSheet("QPushButton {background-color:  red}");
  ui->btn->setText("Start teleop mode");
  target_du_name_ = "/du_" + std::to_string(msg->data);
  ui->status_line->setText(QString::fromStdString(target_du_name_ + " is asking for help"));
  ui->btn->setEnabled(true);
}

void TeleopWidget::tick()
{
  if (mode_ == Mode::TELEOP && twist_publisher_)
  {
    float vel_linear_max = 10.0;
    float vel_angular_max = 2.0;
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = -1 * vel_linear_max * (touch_->y_value);
    msg->angular.z = -1 * vel_angular_max * (touch_->x_value);
    twist_publisher_->publish(*msg);
  }
}

void TeleopWidget::clicked_main_btn()
{
  // auto msg = std::make_shared<std_msgs::msg::Int16>();

  switch (mode_)
  {
    case Mode::INACTIVE:
      start_auto();
      // // Ask for help test
      // msg->data = 9;
      // ask_for_help_callback(msg);
      break;
    case Mode::AUTO:
      start_teleop(target_du_name_);
      break;
    case Mode::TELEOP:
      cancel_teleop();
      start_auto();
      break;
    default:
      break;
  }
}

void TeleopWidget::start_auto()
{
  ui->btn->setText("Auto mode");
  ui->btn->setEnabled(false);
  ui->btn->setStyleSheet("");
  ui->status_line->setText(QString::fromStdString(""));
  touch_->setEnabled(false);
  target_du_name_ = "";

  if (mode_ == Mode::INACTIVE)
  {
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;

    joy_publisher_ = nh_->create_publisher<sensor_msgs::msg::Joy>("/ex_000/cmd/joy", rclcpp::QoS(10).reliable());
    auto msg = std::make_shared<sensor_msgs::msg::Joy>();
    msg->buttons.resize(11);
    msg->buttons[10] = 1;
    joy_publisher_->publish(*msg);
    // for (size_t i = 0; i < 4; i++)
    // {
    //   // Make sure to reach to the excavator
    //   joy_publisher_->publish(*msg);
    // }
  }

  mode_ = Mode::AUTO;
}

void TeleopWidget::start_teleop(std::string target_name)
{
  mode_ = Mode::TELEOP;
  ui->btn->setText("Cancel Teleop mode");
  ui->btn->setStyleSheet("QPushButton {background-color: cyan}");
  std::string topic_name = target_name + "/cmd_vel";
  twist_publisher_ = nh_->create_publisher<geometry_msgs::msg::Twist>(topic_name, rclcpp::QoS(10));
  ui->status_line->setText(QString::fromStdString("Topic: " + topic_name));
  touch_->setEnabled(true);
}

void TeleopWidget::cancel_teleop()
{
  // Reset publisher
  twist_publisher_.reset();

  // // Publish reset obstacles
  // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_obstacle_publisher_;
  // reset_obstacle_publisher_ = nh_->create_publisher<std_msgs::msg::Empty>("/reset_obstacles", rclcpp::QoS(10));
  // auto msg = std::make_shared<std_msgs::msg::Empty>();
  // reset_obstacle_publisher_->publish(*msg);
}
}  // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::TeleopWidget, rviz_common::Panel)