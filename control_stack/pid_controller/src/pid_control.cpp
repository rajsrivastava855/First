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

////////////////////////////////////////////////////////////////////////////////
//
// This file contains functions for the class ThrusterControl. This is the UI
// which can be used to control the thrusters manually. It reads the config file
// and reverses thrusters accordingly. On exit, it saves reverse status to file.
//
////////////////////////////////////////////////////////////////////////////////


//#include "mainwindow.h"
//#include "ui_mainwindow.h"

#include <pid_controller/pid_control.h>

PIDController::PIDController(ros::NodeHandle _nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PIDController), nh(_nh)
{
    ui->setupUi(this);

    pidPub = nh.advertise<pid_controller::PID>("/pid_params", 1, true);
    spPub = nh.advertise<pid_controller::Setpoint>("/setpoints", 1, true);

    std::string confFileName = ros::package::getPath("pid_controller") + "/config/pid.conf";
    std::ifstream confFile(confFileName.c_str());

    float val;

    confFile>>val;
    ui->kp_surge_slider->setValue(int(val));
	ui->kp_surge_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_surge_slider->setValue(int(val));
	ui->ki_surge_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_surge_slider->setValue(int(val));
	ui->kd_surge_slider_2->setValue((val - (int(val))) * 1000);
    //confFile>>val;
    //ui->sp_surge_slider->setValue(int(val));

    confFile>>val;
    ui->kp_sway_slider->setValue(int(val));
	ui->kp_sway_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_sway_slider->setValue(int(val));
	ui->ki_sway_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_sway_slider->setValue(int(val));
	ui->kd_sway_slider_2->setValue((val - (int(val))) * 1000);
    //confFile>>val;
    //ui->sp_sway_slider->setValue(int(val));

    confFile>>val;
    ui->kp_heave_slider->setValue(int(val));
	ui->kp_heave_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_heave_slider->setValue(int(val));
	ui->ki_heave_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_heave_slider->setValue(int(val));
	ui->kd_heave_slider_2->setValue((val - (int(val))) * 1000);
    //confFile>>val;
    //ui->sp_heave_slider->setValue(int(val));

    confFile>>val;
    ui->kp_roll_slider->setValue(int(val));
	ui->kp_roll_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_roll_slider->setValue(int(val));
	ui->ki_roll_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_roll_slider->setValue(int(val));
	ui->kd_roll_slider_2->setValue((val - (int(val))) * 1000);
    //confFile>>val;
    //ui->sp_roll_slider->setValue(int(val));

    confFile>>val;
    ui->kp_pitch_slider->setValue(int(val));
	ui->kp_pitch_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_pitch_slider->setValue(int(val));
	ui->ki_pitch_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_pitch_slider->setValue(int(val));
	ui->kd_pitch_slider_2->setValue((val - (int(val))) * 1000);

    confFile>>val;
    ui->kp_yaw_slider->setValue(int(val));
	ui->kp_yaw_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->ki_yaw_slider->setValue(int(val));
	ui->ki_yaw_slider_2->setValue((val - (int(val))) * 1000);
    confFile>>val;
    ui->kd_yaw_slider->setValue(int(val));
	ui->kd_yaw_slider_2->setValue((val - (int(val))) * 1000);

    confFile.close();
    AUV_LOG("Sliders initialized with config values");

    for(int i=0;i<6;i++)
    {
        pidMsg.kp.push_back(0);
        pidMsg.ki.push_back(0);
        pidMsg.kd.push_back(0);
        spMsg.setpoints.push_back(0);
    }

    connect(ui->kp_heave_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_heave_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_heave_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_heave_slider, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_sway_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_sway_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_sway_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_sway_slider, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_surge_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_surge_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_surge_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_surge_slider, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_yaw_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_yaw_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_yaw_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_yaw_slider, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_pitch_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_pitch_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_pitch_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kp_roll_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_roll_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_roll_slider, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));

    connect(ui->kp_heave_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_heave_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_heave_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_heave_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_sway_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_sway_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_sway_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_sway_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_surge_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_surge_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_surge_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_surge_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_yaw_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_yaw_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_yaw_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->sp_yaw_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_sp(void)));
    connect(ui->kp_pitch_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_pitch_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_pitch_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kp_roll_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->ki_roll_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    connect(ui->kd_roll_slider_2, SIGNAL(valueChanged(int)), SLOT(publish_pid(void)));
    setLabels();
    publish_sp();
    publish_pid();
}

PIDController::~PIDController()
{
    delete ui;
}

void PIDController::setLabels()
{
    ui->kp_heave_val->setText(QString::number(ui->kp_heave_slider->value() + (ui->kp_heave_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_heave_val->setText(QString::number(ui->ki_heave_slider->value() + (ui->ki_heave_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_heave_val->setText(QString::number(ui->kd_heave_slider->value() + (ui->kd_heave_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kp_sway_val->setText(QString::number(ui->kp_sway_slider->value() + (ui->kp_sway_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_sway_val->setText(QString::number(ui->ki_sway_slider->value() + (ui->ki_sway_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_sway_val->setText(QString::number(ui->kd_sway_slider->value() + (ui->kd_sway_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kp_surge_val->setText(QString::number(ui->kp_surge_slider->value() + (ui->kp_surge_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_surge_val->setText(QString::number(ui->ki_surge_slider->value() + (ui->ki_surge_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_surge_val->setText(QString::number(ui->kd_surge_slider->value() + (ui->kd_surge_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kp_yaw_val->setText(QString::number(ui->kp_yaw_slider->value() + (ui->kp_yaw_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_yaw_val->setText(QString::number(ui->ki_yaw_slider->value() + (ui->ki_yaw_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_yaw_val->setText(QString::number(ui->kd_yaw_slider->value() + (ui->kd_yaw_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kp_pitch_val->setText(QString::number(ui->kp_pitch_slider->value() + (ui->kp_pitch_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_pitch_val->setText(QString::number(ui->ki_pitch_slider->value() + (ui->ki_pitch_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_pitch_val->setText(QString::number(ui->kd_pitch_slider->value() + (ui->kd_pitch_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kp_roll_val->setText(QString::number(ui->kp_roll_slider->value() + (ui->kp_roll_slider_2->value()*1.0)/1000.0,'g',4));
    ui->ki_roll_val->setText(QString::number(ui->ki_roll_slider->value() + (ui->ki_roll_slider_2->value()*1.0)/1000.0,'g',4));
    ui->kd_roll_val->setText(QString::number(ui->kd_roll_slider->value() + (ui->kd_roll_slider_2->value()*1.0)/1000.0,'g',4));
    ui->sp_heave_val->setText(QString::number(ui->sp_heave_slider->value() + (ui->sp_heave_slider_2->value()*1.0)/1000.0,'g',4));
    ui->sp_sway_val->setText(QString::number(ui->sp_sway_slider->value() + (ui->sp_sway_slider_2->value()*1.0)/1000.0,'g',4));
    ui->sp_surge_val->setText(QString::number(ui->sp_surge_slider->value() + (ui->sp_surge_slider_2->value()*1.0)/1000.0,'g',4));
    ui->sp_yaw_val->setText(QString::number(ui->sp_yaw_slider->value() + (ui->sp_yaw_slider_2->value()*1.0)/1000.0,'g',4));
}

void PIDController::publish_pid(void)
{
    //msg data order: kp,ki,kd
    pidMsg.kp[2] = ui->kp_heave_slider->value() + (ui->kp_heave_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[2] = ui->ki_heave_slider->value() + (ui->ki_heave_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[2] = ui->kd_heave_slider->value() + (ui->kd_heave_slider_2->value()*1.0)/1000.0;
    pidMsg.kp[1] = ui->kp_sway_slider->value() + (ui->kp_sway_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[1] = ui->ki_sway_slider->value() + (ui->ki_sway_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[1] = ui->kd_sway_slider->value() + (ui->kd_sway_slider_2->value()*1.0)/1000.0;
    pidMsg.kp[0] = ui->kp_surge_slider->value() + (ui->kp_surge_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[0] = ui->ki_surge_slider->value() + (ui->ki_surge_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[0] = ui->kd_surge_slider->value() + (ui->kd_surge_slider_2->value()*1.0)/1000.0;
    pidMsg.kp[5] = ui->kp_yaw_slider->value() + (ui->kp_yaw_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[5] = ui->ki_yaw_slider->value() + (ui->ki_yaw_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[5] = ui->kd_yaw_slider->value() + (ui->kd_yaw_slider_2->value()*1.0)/1000.0;
    pidMsg.kp[4] = ui->kp_pitch_slider->value() + (ui->kp_pitch_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[4] = ui->ki_pitch_slider->value() + (ui->ki_pitch_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[4] = ui->kd_pitch_slider->value() + (ui->kd_pitch_slider_2->value()*1.0)/1000.0;
    pidMsg.kp[3] = ui->kp_roll_slider->value() + (ui->kp_roll_slider_2->value()*1.0)/1000.0;
    pidMsg.ki[3] = ui->ki_roll_slider->value() + (ui->ki_roll_slider_2->value()*1.0)/1000.0;
    pidMsg.kd[3] = ui->kd_roll_slider->value() + (ui->kd_roll_slider_2->value()*1.0)/1000.0;

    //setting labels:
    setLabels();
    pidPub.publish(pidMsg);
}

void PIDController::publish_sp(void)
{
    //setpoints msg data order: surge, sway, heave, yaw
    spMsg.setpoints[0] = ui->sp_surge_slider->value() + (ui->sp_surge_slider_2->value()/100.0);
    spMsg.setpoints[1] = ui->sp_sway_slider->value() + (ui->sp_sway_slider_2->value()/100.0);
    spMsg.setpoints[2] = ui->sp_heave_slider->value() + (ui->sp_heave_slider_2->value()/100.0);
    spMsg.setpoints[3] = 0.0;
    spMsg.setpoints[4] = 0.0;
    spMsg.setpoints[5] = ui->sp_yaw_slider->value() + (ui->sp_yaw_slider_2->value()/100.0);

    //setting labels
    setLabels();

    spPub.publish(spMsg);
}

void PIDController::on_save_clicked()
{
    std::string confFileName = ros::package::getPath("pid_controller") + "/config/pid.conf";
    std::ofstream confFile(confFileName.c_str());

    confFile<<float(ui->kp_surge_slider->value() + (ui->kp_surge_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_surge_slider->value() + (ui->ki_surge_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_surge_slider->value() + (ui->kd_surge_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile<<float(ui->kp_sway_slider->value() + (ui->kp_sway_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_sway_slider->value() + (ui->ki_sway_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_sway_slider->value() + (ui->kd_sway_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile<<float(ui->kp_heave_slider->value() + (ui->kp_heave_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_heave_slider->value() + (ui->ki_heave_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_heave_slider->value() + (ui->kd_heave_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile<<float(ui->kp_roll_slider->value() + (ui->kp_roll_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_roll_slider->value() + (ui->ki_roll_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_roll_slider->value() + (ui->kd_roll_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile<<float(ui->kp_pitch_slider->value() + (ui->kp_pitch_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_pitch_slider->value() + (ui->ki_pitch_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_pitch_slider->value() + (ui->kd_pitch_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile<<float(ui->kp_yaw_slider->value() + (ui->kp_yaw_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->ki_yaw_slider->value() + (ui->ki_yaw_slider_2->value()*1.0)/1000.0)<<'\n';
    confFile<<float(ui->kd_yaw_slider->value() + (ui->kd_yaw_slider_2->value()*1.0)/1000.0)<<'\n';

    confFile.close();

    AUV_LOG("Configuration file successfully written!");
}
