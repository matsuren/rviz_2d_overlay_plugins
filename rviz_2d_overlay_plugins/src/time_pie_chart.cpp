// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include "time_pie_chart.h"
#include <iomanip>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_rendering/render_system.hpp>
#include <QPainter>

namespace rviz_2d_overlay_plugins
{
TimePieChart::TimePieChart() : data_(0.0), update_required_(false), first_time_(true), current_time_(0.0)
{
  size_property_ =
      new rviz_common::properties::IntProperty("size", 150, "size of the plotter window", this, SLOT(updateSize()));
  left_property_ =
      new rviz_common::properties::IntProperty("left", 128, "left of the plotter window", this, SLOT(updateLeft()));
  top_property_ =
      new rviz_common::properties::IntProperty("top", 128, "top of the plotter window", this, SLOT(updateTop()));
  fg_color_property_ = new rviz_common::properties::ColorProperty("foreground color", QColor(25, 255, 240),
                                                                  "color to draw line", this, SLOT(updateFGColor()));
  fg_alpha_property_ = new rviz_common::properties::FloatProperty(
      "foreground alpha", 0.7, "alpha belnding value for foreground", this, SLOT(updateFGAlpha()));
  fg_alpha2_property_ = new rviz_common::properties::FloatProperty(
      "foreground alpha 2", 0.4, "alpha belnding value for foreground for indicator", this, SLOT(updateFGAlpha2()));
  bg_color_property_ = new rviz_common::properties::ColorProperty("background color", QColor(0, 0, 0),
                                                                  "background color", this, SLOT(updateBGColor()));
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
      "backround alpha", 0.0, "alpha belnding value for background", this, SLOT(updateBGAlpha()));
  text_size_property_ =
      new rviz_common::properties::IntProperty("text size", 22, "text size", this, SLOT(updateTextSize()));
  show_caption_property_ =
      new rviz_common::properties::BoolProperty("show caption", true, "show caption", this, SLOT(updateShowCaption()));
  max_value_property_ = new rviz_common::properties::FloatProperty("max value", 400.0, "max value of pie chart", this,
                                                                   SLOT(updateMaxValue()));
  min_value_property_ = new rviz_common::properties::FloatProperty("min value", 0.0, "min value of pie chart", this,
                                                                   SLOT(updateMinValue()));
  auto_color_change_property_ = new rviz_common::properties::BoolProperty(
      "auto color change", true, "change the color automatically", this, SLOT(updateAutoColorChange()));
  max_color_property_ = new rviz_common::properties::ColorProperty(
      "max color", QColor(220, 10, 10), "only used if auto color change is set to True.", this, SLOT(updateMaxColor()));

  med_color_property_ = new rviz_common::properties::ColorProperty(
      "med color", QColor(220, 10, 10), "only used if auto color change is set to True.", this, SLOT(updateMedColor()));

  max_color_threshold_property_ = new rviz_common::properties::FloatProperty(
      "max color change threthold", 400, "change the max color at threshold", this, SLOT(updateMaxColorThreshold()));

  med_color_threshold_property_ = new rviz_common::properties::FloatProperty(
      "med color change threthold", 0, "change the med color at threshold ", this, SLOT(updateMedColorThreshold()));

  clockwise_rotate_property_ = new rviz_common::properties::BoolProperty(
      "clockwise rotate direction", false, "change the rotate direction", this, SLOT(updateClockwiseRotate()));
  caption_property_ = new rviz_common::properties::StringProperty("caption", "Put caption", "caption for visualization",
                                                                  this, SLOT(updateCaption()));
}

TimePieChart::~TimePieChart()
{
  if (overlay_->isVisible())
  {
    overlay_->hide();
  }
}

void TimePieChart::onInitialize()
{
  RTDClass::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "TimePieChart" << count++;
  overlay_.reset(new rviz_2d_overlay_plugins::OverlayObject(ss.str()));
  onEnable();
  updateSize();
  updateLeft();
  updateTop();
  updateFGColor();
  updateBGColor();
  updateFGAlpha();
  updateFGAlpha2();
  updateBGAlpha();
  updateMinValue();
  updateMaxValue();
  updateTextSize();
  updateShowCaption();
  updateAutoColorChange();
  updateMaxColor();
  updateMedColor();
  updateMaxColorThreshold();
  updateMedColorThreshold();
  updateClockwiseRotate();
  updateCaption();
  overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
  overlay_->hide();
  caption_text_ = ss.str();

  // Create subscriber
  current_time_subscription_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<std_msgs::msg::Float32>(
      "/monitor/current_time", rclcpp::QoS(1).best_effort(), [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
        current_time_ = msg->data;
        update_required_ = true;
      });
}

void TimePieChart::update(float /* wall_dt */, float /* ros_dt */)
{
  if (update_required_)
  {
    update_required_ = false;
    overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
    overlay_->setPosition(left_, top_);
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    drawPlot(data_);
  }
}

void TimePieChart::processMessage(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!overlay_->isVisible())
  {
    return;
  }
  if (data_ != msg->data || first_time_)
  {
    first_time_ = false;
    data_ = msg->data;
    update_required_ = true;
  }
}

void TimePieChart::drawPlot(double val)
{
  QColor fg_color(fg_color_);

  if (auto_color_change_)
  {
    double r = std::min(1.0, fabs((val - min_value_) / (max_value_ - min_value_)));
    if (r > 0.9)
    {
      double r2 = (r - 0.9) / 0.1;
      fg_color.setRed((max_color_.red() - fg_color_.red()) * r2 + fg_color_.red());
      fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2 + fg_color_.green());
      fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2 + fg_color_.blue());
    }
    if (max_color_threshold_ != 0)
    {
      if (r > max_color_threshold_)
      {
        fg_color.setRed(max_color_.red());
        fg_color.setGreen(max_color_.green());
        fg_color.setBlue(max_color_.blue());
      }
    }
    if (med_color_threshold_ != 0)
    {
      if (max_color_threshold_ > r and r > med_color_threshold_)
      {
        fg_color.setRed(med_color_.red());
        fg_color.setGreen(med_color_.green());
        fg_color.setBlue(med_color_.blue());
      }
    }
  }

  QColor fg_color2(fg_color);
  QColor bg_color(bg_color_);
  fg_color.setAlpha(fg_alpha_);
  fg_color2.setAlpha(fg_alpha2_);
  bg_color.setAlpha(bg_alpha_);
  int width = overlay_->getTextureWidth();
  int height = overlay_->getTextureHeight();
  {
    rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color);
    QPainter painter(&Hud);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int outer_line_width = 5;
    const int value_line_width = 10;
    const int value_indicator_line_width = 2;
    const int value_padding = 10;

    const int value_aabb_offset = outer_line_width + value_padding + value_line_width / 2;

    painter.setPen(QPen(fg_color, outer_line_width, Qt::SolidLine));

    painter.drawEllipse(outer_line_width / 2, outer_line_width / 2, width - outer_line_width,
                        height - outer_line_width - caption_offset_);

    painter.setPen(QPen(fg_color2, value_indicator_line_width, Qt::SolidLine));
    painter.drawEllipse(value_aabb_offset, value_aabb_offset, width - value_aabb_offset * 2,
                        height - value_aabb_offset * 2 - caption_offset_);

    // Draw estimated time
    const double ratio = (val - min_value_) / (max_value_ - min_value_);
    const double rotate_direction = clockwise_rotate_ ? -1.0 : 1.0;
    const double ratio_angle = ratio * 360.0 * rotate_direction;
    const double start_angle_offset = -90;
    painter.setPen(QPen(fg_color, 2, Qt::SolidLine));
    painter.setBrush(QBrush(fg_color));
    painter.drawPie(QRectF(value_aabb_offset, value_aabb_offset, width - value_aabb_offset * 2,
                           height - value_aabb_offset * 2 - caption_offset_),
                    start_angle_offset * 16, ratio_angle * 16);

    // Draw current time
    painter.setPen(QPen(fg_color, value_line_width + 1, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(fg_color));
    const double ratio2 = (current_time_ - min_value_) / (max_value_ - min_value_);
    const double ratio_angle2 = ratio2 * 360.0 * rotate_direction;
    painter.drawArc(QRectF(value_aabb_offset - value_padding / 2, value_aabb_offset - value_padding / 2,
                           width - value_aabb_offset * 2 + value_padding,
                           height - value_aabb_offset * 2 - caption_offset_ + value_padding),
                    start_angle_offset * 16, ratio_angle2 * 16);

    QFont font = painter.font();
    font.setPointSize(text_size_);
    font.setBold(true);
    painter.setFont(font);
    painter.setPen(QPen(fg_color, value_line_width, Qt::SolidLine));
    // caption
    if (show_caption_)
    {
      painter.drawText(0, height - caption_offset_, width, caption_offset_, Qt::AlignCenter | Qt::AlignVCenter,
                       QString::fromStdString(caption_text_));
    }

    // done
    painter.end();
    // Unlock the pixel buffer
  }
}

void TimePieChart::onEnable()
{
  subscribe();
  overlay_->show();
  first_time_ = true;
}

void TimePieChart::onDisable()
{
  unsubscribe();
  overlay_->hide();
}

void TimePieChart::updateSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  texture_size_ = size_property_->getInt();
  update_required_ = true;
}

void TimePieChart::updateTop()
{
  top_ = top_property_->getInt();
  update_required_ = true;
}

void TimePieChart::updateLeft()
{
  left_ = left_property_->getInt();
  update_required_ = true;
}

void TimePieChart::updateBGColor()
{
  bg_color_ = bg_color_property_->getColor();
  update_required_ = true;
}

void TimePieChart::updateFGColor()
{
  fg_color_ = fg_color_property_->getColor();
  update_required_ = true;
}

void TimePieChart::updateFGAlpha()
{
  fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  update_required_ = true;
}

void TimePieChart::updateFGAlpha2()
{
  fg_alpha2_ = fg_alpha2_property_->getFloat() * 255.0;
  update_required_ = true;
}

void TimePieChart::updateBGAlpha()
{
  bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  update_required_ = true;
}

void TimePieChart::updateMinValue()
{
  min_value_ = min_value_property_->getFloat();
  update_required_ = true;
}

void TimePieChart::updateMaxValue()
{
  max_value_ = max_value_property_->getFloat();
  update_required_ = true;
}

void TimePieChart::updateTextSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  text_size_ = text_size_property_->getInt();
  QFont font;
  font.setPointSize(text_size_);
  caption_offset_ = QFontMetrics(font).height();
  update_required_ = true;
}

void TimePieChart::updateShowCaption()
{
  show_caption_ = show_caption_property_->getBool();
  update_required_ = true;
}

void TimePieChart::updateAutoColorChange()
{
  auto_color_change_ = auto_color_change_property_->getBool();
  if (auto_color_change_)
  {
    max_color_property_->show();
    med_color_property_->show();
    max_color_threshold_property_->show();
    med_color_threshold_property_->show();
  }
  else
  {
    max_color_property_->hide();
    med_color_property_->hide();
    max_color_threshold_property_->hide();
    med_color_threshold_property_->hide();
  }
  update_required_ = true;
}

void TimePieChart::updateMaxColor()
{
  max_color_ = max_color_property_->getColor();
  update_required_ = true;
}

void TimePieChart::updateMedColor()
{
  med_color_ = med_color_property_->getColor();
  update_required_ = true;
}

void TimePieChart::updateMaxColorThreshold()
{
  max_color_threshold_ = max_color_threshold_property_->getFloat();
  update_required_ = true;
}

void TimePieChart::updateMedColorThreshold()
{
  med_color_threshold_ = med_color_threshold_property_->getFloat();
  update_required_ = true;
}

void TimePieChart::updateClockwiseRotate()
{
  clockwise_rotate_ = clockwise_rotate_property_->getBool();
  update_required_ = true;
}

void TimePieChart::updateCaption()
{
  caption_text_ = caption_property_->getString().toStdString();
  update_required_ = true;
}

bool TimePieChart::isInRegion(int x, int y)
{
  return (top_ < y && top_ + texture_size_ > y && left_ < x && left_ + texture_size_ > x);
}

void TimePieChart::movePosition(int x, int y)
{
  top_ = y;
  left_ = x;
}

void TimePieChart::setPosition(int x, int y)
{
  top_property_->setValue(y);
  left_property_->setValue(x);
}
}  // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::TimePieChart, rviz_common::Display)
