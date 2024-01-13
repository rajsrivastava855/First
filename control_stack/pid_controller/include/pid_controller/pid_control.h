////////////////////////////////////////////////////////////////////////////////
//
// MIT License
//
// Copyright (c) 2017 Tiburon, NIT Rourkela
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// AUTHORS: Sonali Agrawal
//
////////////////////////////////////////////////////////////////////////////////


#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QMainWindow>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#include <ui_pid_control.h>
#include <pid_controller/PID.h>
#include <pid_controller/Setpoint.h>

#include <hammerhead/hammerhead.h>


namespace Ui {
class PIDController;
}

class PIDController : public QMainWindow
{
    Q_OBJECT

public:
    explicit PIDController(ros::NodeHandle _nh,QWidget *parent = 0);
    ~PIDController();

private slots:
    void publish_pid(void);
    void publish_sp(void);
    void on_save_clicked();

private:
    Ui::PIDController *ui;
    ros::NodeHandle nh;
    pid_controller::PID pidMsg;
    pid_controller::Setpoint spMsg;
    ros::Publisher pidPub;
    ros::Publisher spPub;
    void setLabels();

};

#endif // PIDCONTROLLER_H
