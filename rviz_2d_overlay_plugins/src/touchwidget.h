// From https://github.com/project-srs/ros_lecture/tree/master/plugin_lecture/src/rviz/panel
#ifndef TOUCHWIDGET_H
#define TOUCHWIDGET_H

#include <QWidget>

class TouchWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TouchWidget(QWidget *parent = nullptr);

    // property
    bool grayout;
    float x_value;
    float y_value;
    int hcen;
    int vcen;
    int rsize;

    // event
    virtual void setEnabled(bool enable);
    virtual void paintEvent(QPaintEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void leaveEvent(QEvent* event);
    void set_value(float x, float y);
};

#endif // TOUCHWIDGET_H
