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
// This file contains functions for the class OutputWidget which is used to view
// the log of the associated process.
//
////////////////////////////////////////////////////////////////////////////////

#include <tiburon_commander/output_widget.h>

OutputWidget::OutputWidget(QWidget *parent) : QWidget(parent), ui(new Ui::OutputWidget)
{
    ui->setupUi(this);
}

OutputWidget::~OutputWidget()
{
}

void OutputWidget::displayOutput(QByteArray data)
{
    // Accepts the input QByteArray and formats it for display in OutputWidget
    QString str = QString(data);
    QStringList lines = str.split(QRegExp("[\r\n]"),QString::SkipEmptyParts);
    for(int i = 0; i < lines.size() ; i++)
    {
        if(lines[i].startsWith("[ERROR]"))
            ui->outputBox->setTextColor(QColor("red"));
        else if(lines[i].startsWith("[LOG]"))
            ui->outputBox->setTextColor(QColor("green"));
        else
        {
            if(!ui->debugCb->isChecked())
                continue;
            ui->outputBox->setTextColor(QColor("black"));
        }
        ui->outputBox->append(lines[i]);
    }
}

void OutputWidget::clearBox()
{
    ui->outputBox->clear();
}
