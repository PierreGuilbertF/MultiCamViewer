#ifndef SYNCHRONIZATIONDIALOG_H
#define SYNCHRONIZATIONDIALOG_H

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
class SynchronizationDialog;
}

class SynchronizationDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SynchronizationDialog(QWidget *parent = 0);
  ~SynchronizationDialog();

  double GetStartPoint();
  double GetSequenceLength();

private slots:

private:
  Ui::SynchronizationDialog *ui;
};

#endif // DISTANCEINFODIALOG_H
