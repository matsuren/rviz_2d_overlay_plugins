// Modified from https://github.com/Kotakku/sample_rviz_plugins/blob/main/src/twist_panel.cpp
#include "teleopwidget.h"
#include "ui_teleopwidget.h"
#include "touchwidget.h"
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <QLabel>
#include <QTimer>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <dump_messages/msg/task_posting_req.hpp>
#include <dump_messages/msg/task_posting_res.hpp>
#include <memory>
#include <chrono>
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rviz_2d_overlay_plugins
{
TeleopWidget::TeleopWidget(QWidget* parent) : rviz_common::Panel(parent), ui(new Ui::TeleopWidget)
{
  mode_ = Mode::INACTIVE;
  ui->setupUi(this);
  touch_ = new TouchWidget();
  touch_->setEnabled(false);
  ui->verticalLayout->addWidget(touch_);

  // Setup connect
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);
  connect(ui->btn, SIGNAL(clicked()), this, SLOT(clicked_main_btn()));
  connect(ui->chat_btn, SIGNAL(clicked()), this, SLOT(clicked_chat_btn()));

  // Setup appearance
  ui->logo_label->setPixmap(rviz_common::loadPixmap("package://rviz_2d_overlay_plugins/icons/cafe_logo.png"));
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

  chat_publisher_ = nh_->create_publisher<std_msgs::msg::String>("/chat", rclcpp::QoS(10).reliable());

  task_posting_res_subscriber_ = nh_->create_subscription<dump_messages::msg::TaskPostingRes>(
      "/task_posting/res", rclcpp::QoS(10).reliable(), std::bind(&TeleopWidget::task_posting_res, this, _1));

  task_publisher_ =
      nh_->create_publisher<dump_messages::msg::TaskPostingReq>("/task_posting/req", rclcpp::QoS(10).reliable());

  current_time_publisher_ = nh_->create_publisher<std_msgs::msg::Float32>("/monitor/current_time", rclcpp::QoS(1).best_effort());


  load_dump_sites_subscriber_ = nh_->create_subscription<dump_messages::msg::TaskPostingReq>(
      "/monitor/load_dump_sites", rclcpp::QoS(10).reliable(), std::bind(&TeleopWidget::load_dump_sites_callback, this, _1));

  // Setup loading and disposal site
  loading_sites_.resize(4);
  disposal_sites_.resize(4);

  ////////////////////////////////////////////
  // TEST
  // soil_hill_position
  auto pt = geometry_msgs::msg::Point();
  pt.x = 80.0;
  pt.y = -85.0;
  loading_sites_[0] = pt;
  pt.x = 40.0;
  pt.y = -25.0;
  loading_sites_[1] = pt;
  pt.x = 80.0;
  pt.y = 35.0;
  loading_sites_[2] = pt;
  pt.x = 40.0;
  pt.y = 95.0;
  loading_sites_[3] = pt;
  // dump_site_position
  pt.x = -75.0;
  pt.y = -115.0;
  disposal_sites_[0] = pt;
  pt.x = -75.0;
  pt.y = -55.0;
  disposal_sites_[1] = pt;
  pt.x = -75.0;
  pt.y = 5.0;
  disposal_sites_[2] = pt;
  pt.x = -75.0;
  pt.y = 65.0;
  disposal_sites_[3] = pt;

  deadlines_.clear();
  amount_of_sands_.clear();
  for (size_t i = 0; i < disposal_sites_.size(); i++)
  {
    deadlines_.push_back(400.0);
    amount_of_sands_.push_back(2500.0);
  }

}

void TeleopWidget::load_dump_sites_callback(const dump_messages::msg::TaskPostingReq::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "Load dump sites callback");
  loading_sites_ = msg->load_sites;
  disposal_sites_ = msg->dump_sites;
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "loading_sites_ length:" + std::to_string(loading_sites_.size()));
  // deadlines_ = msg->deadlines;
  // amount_of_sands_ = msg->amount_of_sands;
}

void TeleopWidget::ask_for_help_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "Ask for help from dump: " + std::to_string(msg->data));
  ui->btn->setStyleSheet("QPushButton {background-color:  red}");
  ui->btn->setText("Start teleop mode");
  target_du_name_ = "/du_" + std::to_string(msg->data);
  ui->status_line->setText(QString::fromStdString(target_du_name_ + " is asking for help"));
  ui->btn->setEnabled(true);
}

void TeleopWidget::task_posting_res(const dump_messages::msg::TaskPostingRes::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "Task posting Res" + std::to_string(msg->excavator_id));
  excavator_scores_[msg->excavator_id] = msg->scores;
  task_timer_.reset();
  task_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TeleopWidget::task_timer_callback, this));
}

void TeleopWidget::task_timer_callback()
{
  task_timer_.reset();
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "Timer callback");
  RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"), "Scores: " + std::to_string(excavator_scores_.size()));

  // Score is distance so find excavator with lowest score
  auto msg_task = std::make_shared<dump_messages::msg::TaskPostingReq>();
  msg_task->target_ids = {};
  msg_task->deadlines = deadlines_;
  msg_task->amount_of_sands = amount_of_sands_;
  msg_task->dump_sites = disposal_sites_;
  msg_task->load_sites = loading_sites_;
  // Get target excavator ids
  std::vector<int> excavator_ids;
  for (auto& it : excavator_scores_)
  {
    excavator_ids.push_back(it.first);
  }

  // Find excavator with lowest score
  for (size_t i = 0; i < disposal_sites_.size(); i++)
  {
    int best_id = -1;
    float best_score = 10000000000.0f;
    for (auto& it : excavator_scores_)
    {
      if (best_score > it.second[i])
      {
        best_score = it.second[i];
        best_id = it.first;
      }
    }
    msg_task->target_ids.push_back(best_id);
    RCLCPP_INFO(rclcpp::get_logger("TeleopWidget"),
                "Best id: " + std::to_string(best_id) + " score: " + std::to_string(best_score));
    excavator_scores_.erase(best_id);
  }

  task_publisher_->publish(*msg_task);

  send_start_signal();
}

void TeleopWidget::tick()
{
  if (mode_ == Mode::TELEOP && twist_publisher_)
  {
    float vel_linear_max = 10.0;
    float vel_angular_max = 30.0;
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = -1.0 * vel_linear_max * (touch_->y_value);
    msg->angular.z = -1.0 * vel_angular_max * (touch_->x_value);
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

void TeleopWidget::clicked_chat_btn()
{
  // send text if chat_text_edit is not empty
  if (!ui->chat_text_edit->toPlainText().isEmpty())
  {
    // Obtain text from ui->chat_text_edit and publish it
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = ui->chat_text_edit->toPlainText().toStdString();
    chat_publisher_->publish(*msg);
    ui->chat_text_edit->clear();
  }
}

void TeleopWidget::clock_timer_callback(){
  auto msg = std::make_shared<std_msgs::msg::Float32>();
  auto now = std::chrono::steady_clock::now();
  auto diff = now - start_time_;
  msg->data = std::chrono::duration<float>(diff).count();
  current_time_publisher_->publish(*msg);
}

void TeleopWidget::send_start_signal()
{
  // Start publishing current time
  start_time_ = std::chrono::steady_clock::now();
  clock_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TeleopWidget::clock_timer_callback, this));


  // Send start signal
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  joy_publisher_ = nh_->create_publisher<sensor_msgs::msg::Joy>("/ex_000/cmd/joy", rclcpp::QoS(10).reliable());
  auto msg = std::make_shared<sensor_msgs::msg::Joy>();
  msg->buttons.resize(11);
  msg->buttons[10] = 1;

  joy_publisher_->publish(*msg);
  rclcpp::sleep_for(500ms);
  joy_publisher_->publish(*msg);
}

void TeleopWidget::start_auto()
{
  ui->btn->setText("Auto mode");
  ui->btn->setEnabled(false);
  ui->btn->setStyleSheet("");
  ui->status_line->setText(QString::fromStdString("Working properly"));
  touch_->setEnabled(false);
  target_du_name_ = "";

  if (mode_ == Mode::INACTIVE)
  {
    auto msg_task = std::make_shared<dump_messages::msg::TaskPostingReq>();
    msg_task->target_ids = {};  // Broadcast
    msg_task->deadlines = deadlines_;
    msg_task->amount_of_sands = amount_of_sands_;
    msg_task->dump_sites = disposal_sites_;
    msg_task->load_sites = loading_sites_;
    task_publisher_->publish(*msg_task);
    ui->status_line->setText(QString::fromStdString("Task request"));
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