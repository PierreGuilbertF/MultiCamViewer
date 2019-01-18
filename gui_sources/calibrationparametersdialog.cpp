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

#include "calibrationparametersdialog.h"
#include "ui_calibrationparametersdialog.h"

CalibrationParametersDialog::CalibrationParametersDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::CalibrationParametersDialog)
{
  ui->setupUi(this);
}

CalibrationParametersDialog::~CalibrationParametersDialog()
{
  delete ui;
}

//---------------------------------------------------------------------------
int CalibrationParametersDialog::GetNFeatures()
{
  return this->ui->nFeaturesSpinBox->value();
}

//---------------------------------------------------------------------------
int CalibrationParametersDialog::GetNOctaves()
{
  return this->ui->nOctavesSpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetEdgeThreshold()
{
  return this->ui->edgeThresholdSpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetContrastThreshold()
{
  return this->ui->contrastThresholdSpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetSigma()
{
  return this->ui->sigmaSpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetRatioMatchingThresh()
{
  return this->ui->ratioMatchingThreshSpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetParam1()
{
  return this->ui->param1SpinBox->value();
}

//---------------------------------------------------------------------------
double CalibrationParametersDialog::GetParam2()
{
  return this->ui->param2SpinBox->value();
}

//---------------------------------------------------------------------------
int CalibrationParametersDialog::GetNbrImToUse()
{
  return this->ui->nbrImSpinBox->value();
}

//---------------------------------------------------------------------------
bool CalibrationParametersDialog::GetExportImageResult()
{
  return this->ui->exportImageResultCheckBox->isChecked();
}
