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
// This file contains functions for the class CommanderWindow. This is the main
// UI which has different tabs for categories of processes according to the
// configuration file.
//
////////////////////////////////////////////////////////////////////////////////

#include <tiburon_commander/commander_window.h>

CommanderWindow::CommanderWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::CommanderWindow)
{
    ui->setupUi(this);
    // Remove the first tab which appears by default
    ui->tabWidget->removeTab(ui->tabWidget->indexOf(ui->tabWidget->widget(0)));

    connect(ui->runAllBtn, SIGNAL(clicked()), SLOT(runAllBtnClicked()));
    connect(ui->stopAllBtn, SIGNAL(clicked()), SLOT(stopAllBtnClicked()));
    connect(ui->quitBtn, SIGNAL(clicked()), SLOT(quitBtnClicked()));
}

CommanderWindow::~CommanderWindow()
{
    delete ui;
}

bool CommanderWindow::setupTabs(std::string confFileName)
{
    std::string line;
    std::ifstream confFile(confFileName.c_str());
    if (!confFile.is_open())
    {
        qDebug() << "Unable to open config file. Exiting!";
        return false;
    }

    std::map<QString, int> tabNames; // Each tab is associated with a name and id
    int numOfTabs = 0;
    // Finding list of all processes in config file
    while (std::getline(confFile, line))
    {
        if (line[0] == '#' || line.size() == 0)
            continue;
        ProcessWidget* newProcess = new ProcessWidget();
        newProcess->setDetails(line);
        processList.push_back(newProcess);
        // Finding if tab exists else create new
        if (tabNames.find(newProcess->pTabName) == tabNames.end())
            tabNames.insert(std::pair<QString, int>(newProcess->pTabName, numOfTabs++));
    }

    // Making a list of processes according to the tab name
    std::vector<ProcessVector> processTabs(tabNames.size());
    for (ProcessVector::iterator it = processList.begin(); it != processList.end(); it++)
        processTabs[tabNames[(*it)->pTabName]].push_back(*it);

    confFile.close();
    for (std::vector<ProcessVector>::iterator it = processTabs.begin(); it != processTabs.end(); it++)
        addTabForSection(*it, (*it)[0]->pTabName);

    return true;
}

void CommanderWindow::addTabForSection(ProcessVector processList, QString tabName)
{
    QFrame* group = new QFrame();
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    group->setLayout(vbox);
    QScrollArea* scrollArea = new QScrollArea();
    for (size_t i = 0; i < processList.size(); ++i)
    {
        vbox->addWidget(processList[i]);
    }
    scrollArea->setWidget(group);
    ui->tabWidget->addTab(scrollArea, tabName);
}

void CommanderWindow::watchProcesses()
{
    for (ProcessVector::iterator it = processList.begin(); it != processList.end(); it++)
        (*it)->watchProcess();
}

void CommanderWindow::runAllBtnClicked()
{
    for (ProcessVector::iterator it = processList.begin(); it != processList.end(); it++)
        if (!(*it)->processRunning and (*it)->runCbIsChecked())
            (*it)->runBtnClicked();
}

void CommanderWindow::stopAllBtnClicked()
{
    for (ProcessVector::iterator it = processList.begin(); it != processList.end(); it++)
        if ((*it)->processRunning)
            (*it)->runBtnClicked();
}

void CommanderWindow::quitBtnClicked()
{
    QMessageBox::StandardButton resBtn =
        QMessageBox::question(this, "Tiburon Commander", tr("Kill all processes and exit?"),
                              QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);
    if (resBtn == QMessageBox::Yes)
    {
        stopAllBtnClicked();
        exit(0);
    }
}

void CommanderWindow::closeEvent(QCloseEvent* event)
{
    QMessageBox::StandardButton resBtn =
        QMessageBox::question(this, "Tiburon Commander", tr("Kill all processes and exit?"),
                              QMessageBox::No | QMessageBox::Yes, QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes)
    {
        event->ignore();
    }
    else
    {
        stopAllBtnClicked();
        event->accept();
    }
}
