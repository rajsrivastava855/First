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

////////////////////////////////////////////////////////////////////////////////
//
// This file contains functions for the class ProcessWidget. Each process in the
// configuration file is associated with a widget which can be used to start,
// stop, and view the status of the process.
//
////////////////////////////////////////////////////////////////////////////////

#include <tiburon_commander/process_widget.h>

ProcessWidget::ProcessWidget(QWidget *parent) : QWidget(parent), ui(new Ui::ProcessWidget)
{
    ui->setupUi(this);
    ui->runLbl->setStyleSheet("QLabel { background-color : red;}");
    connect(ui->runBtn, SIGNAL(clicked()), SLOT(runBtnClicked()));
    connect(ui->viewBtn, SIGNAL(clicked()), SLOT(viewBtnClicked()));
    processRunning = false;
    out = new OutputWidget();
}

ProcessWidget::~ProcessWidget()
{
}

void ProcessWidget::setDetails(std::string details)
{
    // Accepts the string containing process name, command, etc., and sets the
    // variables accordingly.
    // Link below describes the tokenizer
    // http://www.boost.org/doc/libs/1_36_0/libs/tokenizer/escaped_list_separator.htm
    typedef boost::tokenizer<boost::escaped_list_separator<char>> TokenSeperator;
    TokenSeperator tok(details, boost::escaped_list_separator<char>('\\', ' ', '\"'));
    TokenSeperator::iterator it = tok.begin();
    pName = QString::fromStdString(*it++);
    pRunSetting = QString::fromStdString(*it++);
    pTabName = QString::fromStdString(*it++);
    pHost = QString::fromStdString(*it++);
    pCommand = QString::fromStdString(*it++);

    ui->nameLbl->setText(pName);
    if (!QString::compare(pRunSetting, "ON", Qt::CaseInsensitive))
        ui->runCb->setChecked(true);

    out->setWindowTitle(pName);
}

void ProcessWidget::runBtnClicked()
{
    if (!processRunning)
    {
        runProcess();
        ui->runBtn->setText("Stop");
    }
    else
    {
        stopProcess();
        ui->runBtn->setText("Run");
    }
    processRunning = !processRunning;
}

void ProcessWidget::updateError()
{
    // Redirecting stderr to OutputWidget
    QByteArray data = process->readAllStandardError();
    out->displayOutput(data);
}

void ProcessWidget::updateText()
{
    // Redirecting stdout to OutputWidget
    QByteArray data = process->readAllStandardOutput();
    out->displayOutput(data);
}

void ProcessWidget::runProcess()
{
    // Read documentation of QProcess for more details
    process = new QProcess();
    //process->setProcessChannelMode(QProcess::MergedChannels);
    connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
    connect(process, SIGNAL(readyReadStandardError()), this, SLOT(updateError()));
    QString processCode = pCommand;
    /*if(pHost == "localhost")
        processCode = pCommand;
    else
        processCode = "ssh -t "+pHost+" \""+pCommand+"\"";*/
    process->start(processCode);
    qDebug() << "Started: " << pCommand;
}

void ProcessWidget::stopProcess()
{
    process->terminate();
}

void ProcessWidget::watchProcess()
{
    // This function tracks the current status of the QProcess variable
    // qDebug() << "Night gathers and now my watch begins";
    if(process == NULL)
        return;
    state = process->state();

    if (lastState != state)
    {
        switch (state)
        {
            case QProcess::Running:
                ui->runLbl->setText(" Y ");
                ui->runLbl->setStyleSheet("QLabel { background-color : green; }");
                break;
            case QProcess::NotRunning:
                ui->runLbl->setText(" N ");
                ui->runLbl->setStyleSheet("QLabel { background-color : red; }");
                break;
            case QProcess::Starting:
                ui->runLbl->setText(" S ");
                ui->runLbl->setStyleSheet("QLabel { background-color : yellow; }");
                break;
            default:
                break;
        }
    }
    lastState = state;
    if (state == QProcess::NotRunning && processRunning)
        if (ui->runCb->isChecked())
            runProcess();
        else
            runBtnClicked();
    // qDebug() << "Now my watch is ended";
}

bool ProcessWidget::runCbIsChecked()
{
    return ui->runCb->isChecked();
}

void ProcessWidget::viewBtnClicked()
{
    out->clearBox();
    out->show();
    out->activateWindow();
}
