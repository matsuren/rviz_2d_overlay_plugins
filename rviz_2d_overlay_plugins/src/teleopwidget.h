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

QT_BEGIN_NAMESPACE
namespace Ui { class TeleopWidget; }
QT_END_NAMESPACE

namespace rviz_2d_overlay_plugins
{
class TeleopWidget : public rviz_common::Panel
{
    Q_OBJECT

public:
    TeleopWidget(QWidget *parent = nullptr);
    ~TeleopWidget();

    virtual void onInitialize();

public Q_SLOTS:
    void tick();

private:
    Ui::TeleopWidget *ui;
    TouchWidget* touch_;

    // The ROS node handle.
    // ros::NodeHandle nh_;
    rclcpp::Node::SharedPtr nh_;

    // The ROS publisher for the command velocity.
    // ros::Publisher twist_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};
}
#endif // TELEOPWIDGET_H
