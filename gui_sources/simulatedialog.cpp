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

#include "simulatedialog.h"
#include "ui_simulatedialog.h"

SimulateDialog::SimulateDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::SimulateDialog)
{
  ui->setupUi(this);
  this->ui->unitedCameraRadio->setChecked(true);
}

SimulateDialog::~SimulateDialog()
{
  delete ui;
}

//---------------------------------------------------------------------------
double SimulateDialog::GetLeftPhi()
{
  return this->ui->LeftPhiSPinBox->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetLeftTheta()
{
  return this->ui->LeftThetaSpinBox->value();
}

//---------------------------------------------------------------------------
QPushButton* SimulateDialog::GetLoadButton()
{
  return this->ui->loadCalibration;
}

//---------------------------------------------------------------------------
double SimulateDialog::GetLeftPsi()
{
  return this->ui->LeftImagePsi->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRightPhi()
{
  return this->ui->RightPhiSpinBox->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRightTheta()
{
  return this->ui->RightThetaSpinBox->value();
}

//---------------------------------------------------------------------------
void SimulateDialog::SetRelativePhi(double input)
{
  this->ui->relPhiSpinBox->setValue(input);
}

//---------------------------------------------------------------------------
void SimulateDialog::SetRelativeTheta(double input)
{
  this->ui->relThetaSpinBox->setValue(input);
}

//---------------------------------------------------------------------------
void SimulateDialog::SetRelativePsi(double input)
{
  this->ui->relPsiSpinBox->setValue(input);
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRightPsi()
{
  return this->ui->RightPsiSpinBox->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRelPhi()
{
  return this->ui->relPhiSpinBox->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRelTheta()
{
  return this->ui->relThetaSpinBox->value();
}

//---------------------------------------------------------------------------
double SimulateDialog::GetRelPsi()
{
  return this->ui->relPsiSpinBox->value();
}

//---------------------------------------------------------------------------
int SimulateDialog::GetMethod()
{
  int res = 0;
  if (this->ui->separateCameraRadio->isChecked())
  {
    res = 1;
  }
  else if (this->ui->unitedCameraRadio->isChecked())
  {
    res = 2;
  }

  return res;
}