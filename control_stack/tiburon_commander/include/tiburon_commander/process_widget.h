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
// AUTHORS: Rohit Suri
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PROCESS_WIDGET_H
#define PROCESS_WIDGET_H

#include <QWidget>
#include <QProcess>
#include <QDebug>

#include <boost/tokenizer.hpp>

#include <tiburon_commander/output_widget.h>
#include <ui_process_widget.h>

namespace Ui
{
class ProcessWidget;
}

class ProcessWidget : public QWidget
{
    Q_OBJECT
  public:
    ProcessWidget(QWidget *parent = 0);
    ~ProcessWidget();
    QString pName, pRunSetting, pTabName, pHost, pCommand;

    bool processRunning;
    void setDetails(std::string);
    void watchProcess();
    bool runCbIsChecked();

  public slots:
    void updateText();
    void updateError();
    void runBtnClicked();
    void viewBtnClicked();

  private:
    QProcess *process = NULL;
    OutputWidget *out = NULL;
    Ui::ProcessWidget *ui;
    QProcess::ProcessState state = QProcess::NotRunning, lastState = QProcess::NotRunning;
    void runProcess();
    void stopProcess();
};

#endif  // PROCESS_WIDGET_H
