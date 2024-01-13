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

////////////////////////////////////////////////////////////////////////////////
//
// This file contains functions for the class ThrusterControl. This is the UI
// which can be used to control the thrusters manually. It reads the config file
// and reverses thrusters accordingly. On exit, it saves reverse status to file.
//
////////////////////////////////////////////////////////////////////////////////

#include <thruster_controller/thruster_control.h>

ThrusterControl::ThrusterControl(ros::NodeHandle _nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ThrusterControl), nh(_nh)
{
    ui->setupUi(this);
    pub = nh.advertise<thruster_controller::ThrusterSpeeds>("/thruster_speeds", 1);
    connect(ui->slider_1, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->slider_2, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->slider_3, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->slider_4, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->slider_5, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->slider_6, SIGNAL(valueChanged(int)), SLOT(publish(void)));
    connect(ui->cb_1, SIGNAL(toggled(bool)), SLOT(publish(void)));
    connect(ui->cb_2, SIGNAL(toggled(bool)), SLOT(publish(void)));
    connect(ui->cb_3, SIGNAL(toggled(bool)), SLOT(publish(void)));
    connect(ui->cb_4, SIGNAL(toggled(bool)), SLOT(publish(void)));
    connect(ui->cb_5, SIGNAL(toggled(bool)), SLOT(publish(void)));
    connect(ui->cb_6, SIGNAL(toggled(bool)), SLOT(publish(void)));

    for(int i=0;i<6;i++)
    {
        msg.data.push_back(1500);
        msg.reverse.push_back(false);
    }

    std::string confFileName = ros::package::getPath("thruster_controller") + "/config/reverse.conf";
    std::ifstream confFile(confFileName.c_str());

    bool val;
    confFile>>val;
    ui->cb_1->setChecked(val);
    confFile>>val;
    ui->cb_2->setChecked(val);
    confFile>>val;
    ui->cb_3->setChecked(val);
    confFile>>val;
    ui->cb_4->setChecked(val);
    confFile>>val;
    ui->cb_5->setChecked(val);
    confFile>>val;
    ui->cb_6->setChecked(val);

    confFile.close();
    AUV_LOG("Parameters loaded!");
}


ThrusterControl::~ThrusterControl()
{
    delete ui;
}

void ThrusterControl::publish(void)
{
    msg.data[0] = ui->slider_1->value();
    msg.data[1] = ui->slider_2->value();
    msg.data[2] = ui->slider_3->value();
    msg.data[3] = ui->slider_4->value();
    msg.data[4] = ui->slider_5->value();
    msg.data[5] = ui->slider_6->value();
    if(ui->latchDepth->isChecked())
        msg.data[1] = msg.data[0];
    ui->th_1->setText(QString::number(ui->slider_1->value()));
    ui->th_2->setText(QString::number(ui->slider_2->value()));
    ui->th_3->setText(QString::number(ui->slider_3->value()));
    ui->th_4->setText(QString::number(ui->slider_4->value()));
    ui->th_5->setText(QString::number(ui->slider_5->value()));
    ui->th_6->setText(QString::number(ui->slider_6->value()));
    msg.reverse[0] = ui->cb_1->isChecked();
    msg.reverse[1] = ui->cb_2->isChecked();
    msg.reverse[2] = ui->cb_3->isChecked();
    msg.reverse[3] = ui->cb_4->isChecked();
    msg.reverse[4] = ui->cb_5->isChecked();
    msg.reverse[5] = ui->cb_6->isChecked();
    pub.publish(msg);
}

void ThrusterControl::on_stop_1_clicked()
{
    ui->slider_1->setValue(1500);
}

void ThrusterControl::on_stop_2_clicked()
{
    ui->slider_2->setValue(1500);
}

void ThrusterControl::on_stop_3_clicked()
{
    ui->slider_3->setValue(1500);
}

void ThrusterControl::on_stop_4_clicked()
{
    ui->slider_4->setValue(1500);
}

void ThrusterControl::on_stop_5_clicked()
{
    ui->slider_5->setValue(1500);
}

void ThrusterControl::on_stop_6_clicked()
{
    ui->slider_6->setValue(1500);
}

void ThrusterControl::on_stop_all_clicked()
{
    ui->slider_1->setValue(1500);
    ui->slider_2->setValue(1500);
    ui->slider_3->setValue(1500);
    ui->slider_4->setValue(1500);
    ui->slider_5->setValue(1500);
    ui->slider_6->setValue(1500);
    publish();
}

void ThrusterControl::on_save_clicked()
{
    std::string confFileName = ros::package::getPath("thruster_controller") + "/config/reverse.conf";
    std::ofstream confFile(confFileName.c_str());
    confFile<<int(ui->cb_1->isChecked())<<'\n';
    confFile<<int(ui->cb_2->isChecked())<<'\n';
    confFile<<int(ui->cb_3->isChecked())<<'\n';
    confFile<<int(ui->cb_4->isChecked())<<'\n';
    confFile<<int(ui->cb_5->isChecked())<<'\n';
    confFile<<int(ui->cb_6->isChecked())<<'\n';
    confFile.close();
    AUV_LOG("Configuration file successfully written!");
}
