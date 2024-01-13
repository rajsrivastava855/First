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
// AUTHORS: Prabin Rath, Rohit Suri
//
////////////////////////////////////////////////////////////////////////////////

#ifndef THRUSTERCONTOL_H
#define THRUSTERCONTOL_H

#include <QMainWindow>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#include <thruster_controller/ThrusterSpeeds.h>
#include <ui_thruster_control.h>
#include <hammerhead/hammerhead.h>

namespace Ui {
class ThrusterControl;
}

class ThrusterControl : public QMainWindow
{
    Q_OBJECT

public:
    explicit ThrusterControl(ros::NodeHandle _nh,QWidget *parent = 0);
    ~ThrusterControl();

private slots:
    void publish(void);
    void on_stop_1_clicked();
    void on_stop_2_clicked();
    void on_stop_3_clicked();
    void on_stop_4_clicked();
    void on_stop_5_clicked();
    void on_stop_6_clicked();
    void on_stop_all_clicked();
    void on_save_clicked();

private:
    Ui::ThrusterControl *ui;
    ros::NodeHandle nh;
    thruster_controller::ThrusterSpeeds msg;
    ros::Publisher pub;
};

#endif // THRUSTERCONTOL_H
