// Modified from https://github.com/Kotakku/sample_rviz_plugins/blob/main/src/twist_panel.cpp
#include "teleopwidget.h"
#include "ui_teleopwidget.h"
#include "touchwidget.h"
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <QLabel>
#include <QTimer>
namespace rviz_2d_overlay_plugins
{
TeleopWidget::TeleopWidget(QWidget *parent)
    : rviz_common::Panel(parent)
    , ui(new Ui::TeleopWidget)
{
    ui->setupUi(this);
    touch_ = new TouchWidget();
    ui->layout->addWidget(touch_);

    QTimer* output_timer = new QTimer(this);
    connect(output_timer, SIGNAL(timeout()), this, SLOT(tick()));
    output_timer->start(100);
}

TeleopWidget::~TeleopWidget()
{
    delete ui;
}


void TeleopWidget::onInitialize()
{
    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void TeleopWidget::tick()
{
    if (ui->enable_check->isChecked()){
        //
        ui->topic_edit->setEnabled(false);
        ui->stamped_check->setEnabled(false);
        ui->frame_edit->setEnabled(false);
        touch_->setEnabled(true);
    }
    else
    {
        // gray to not process
        ui->topic_edit->setEnabled(true);
        ui->stamped_check->setEnabled(true);
        ui->frame_edit->setEnabled(true);
        touch_->setEnabled(false);
    }
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::TeleopWidget, rviz_common::Panel)