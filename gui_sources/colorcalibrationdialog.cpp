//=========================================================================
// MultiCamViewer - Viewer and algorithms for multicamera system
//
// Copyright 2018 Pierre Guilbert
// Author: Pierre Guilbert (spguilbert@gmail.com)
// Data: 02-11-2018
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//=========================================================================

#include "colorcalibrationdialog.h"
#include "ui_colorcalibrationdialog.h"

ColorCalibrationDialog::ColorCalibrationDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ColorCalibrationDialog)
{
  ui->setupUi(this);
  this->ui->LeftOnRightRatio->setChecked(true);
  this->ui->saturationSpinBox->setDecimals(5);
  this->ui->saturationSpinBox->setValue(0.035);
}

ColorCalibrationDialog::~ColorCalibrationDialog()
{
  delete ui;
}

//---------------------------------------------------------------------------
int ColorCalibrationDialog::GetWindowSise()
{
  return this->ui->WindowSizeSpinBox->value();
}

//---------------------------------------------------------------------------
bool ColorCalibrationDialog::GetEnableWhiteBalance()
{
  return this->ui->WhiteBalanceCheckBox->isChecked();
}

//---------------------------------------------------------------------------
double ColorCalibrationDialog::GetWhiteSaturation()
{
  return this->ui->saturationSpinBox->value();
}

//---------------------------------------------------------------------------
int ColorCalibrationDialog::GetMethodChoice()
{
  int ret = 0;
  if (this->ui->LeftOnRightRatio->isChecked())
  {
    std::cout << "Left On right" << std::endl;
    ret = 1;
  }
  else if (this->ui->LeftOnRightRatio->isChecked())
  {
    std::cout << "Right On Left" << std::endl;
    ret = 2;
  }

  return ret;
}
