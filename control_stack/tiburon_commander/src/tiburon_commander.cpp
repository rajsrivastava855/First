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
// This file is used to deploy the Commander UI of Tiburon, which is used to
// switch all nodes ON/OFF or visualize the data.
//
// CommanderWindow is the main UI with different tabs. Each tab has a certain
// number of processes depending on the configuration file. Each process is an
// object of the class ProcessWidget. OutputWidget is the class which provides a
// recent log for each process.
//
////////////////////////////////////////////////////////////////////////////////

#include <ros/package.h>
#include <QApplication>

#include <tiburon_commander/commander_window.h>

static bool quitSignal = false;

int main(int argc, char** argv)
{
    QApplication a(argc, argv);
    // TODO: Pass config file as an argument
    std::string confFileName = ros::package::getPath("tiburon_commander") + "/config/hammerhead.conf";
    CommanderWindow w;
    w.setWindowTitle("Tiburon Commander");
    if (!w.setupTabs(confFileName))
        return 1;
    w.show();
    while (w.isVisible())
    {
        usleep(50000);
        a.processEvents();
        w.watchProcesses();
    }
}
