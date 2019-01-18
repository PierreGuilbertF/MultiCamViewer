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

#ifndef SIMULATEDIALOG_H
#define SIMULATEDIALOG_H

// STD
#include <iostream>
#include <fstream>

// OPENCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// QT
#include <QtWidgets/QDialog>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QtWidgets>

// LOCAL
#include "AudioExtractor.h"
#include "Calibrator.h"
#include "ManualMatcher.h"
#include "NumericalAnalysisTools.h"
#include "StereoVideoManager.h"
#include "Toolkit.h"

namespace Ui {
class SimulateDialog;
}

class SimulateDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SimulateDialog(QWidget *parent = 0);
  ~SimulateDialog();

  // Get the spin box values
  double GetLeftPhi();
  double GetLeftTheta();
  double GetLeftPsi();
  double GetRightPhi();
  double GetRightTheta();
  double GetRightPsi();
  double GetRelPhi();
  double GetRelTheta();
  double GetRelPsi();
  void SetRelativePhi(double input);
  void SetRelativeTheta(double input);
  void SetRelativePsi(double input);
  QPushButton* GetLoadButton();
  int GetMethod();

private slots:

private:
  Ui::SimulateDialog *ui;
};

#endif // DISTANCEINFODIALOG_H
