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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// STD
#include <iostream>
#include <fstream>
#include <ctime>

// OPENCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/xphoto/white_balance.hpp"

// QT
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QtWidgets>
#include <QtWidgets/qslider.h>

// LOCAL
#include "AudioExtractor.h"
#include "Calibrator.h"
#include "ColorCalibrator.h"
#include "ManualMatcher.h"
#include "NumericalAnalysisTools.h"
#include "StereoVideoManager.h"
#include "Toolkit.h"
#include "calibrationparametersdialog.h"
#include "distanceinfodialog.h"
#include "simulatedialog.h"
#include "colorcalibrationdialog.h"
#include "exportvideodialog.h"
#include "synchronizationdialog.h";

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  // Refresh the qpixmap image with the new one
  void ShowImageLeft();
  void ShowImageRight();
  void ShowImageLastResult();
  void ShowImageLastResultLeft();

private slots:
  // method called when the button to change the left image is clicked
  // we prompt the user to select an image, and we display it
  void onOpenLeftVideo();

  // method called when the button to change the right image is clicked
  // we prompt the user to select an image, and we display it
  void onOpenRightVideo();

  // Make the calibration of the current cameras
  void OnCalibrate();

  // Load a calibration to simulate the same pose
  void OnLoadCalibrationFromSimulation();

  // Launch distance oinformation dialog
  void OnLaunchDistanceInfoDialog();

  // launch the simulation dialog box
  void OnLaunchSimulationDialog();

  // Simulate a new acquisition
  void OnSimulateAcquisition();

  // Indicate distance information
  void OnDistanceInfo();

  // Import a calibration
  void OnImportCalibration();

  // Export a calibration
  void OnExportCalibration();

  // Too busy to explain
  void OnLaunchDialogForCalibration();

  // Export the images rectified
  void OnSaveRectifiedImages();

  // Export the video
  void OnSaveVideo();

  // Calibrate the color of the videos
  void OnCalibrateColor();

  // Play the video
  void OnPlay();

  // Next frame at the end of the timer
  void OnTimerPlayOut();

  //
  void OnTimerCalibrationOut();

  // Launch time synchronization algorithm
  void OnSynchronization();

  // Launch image based synchronization algorithm
  void OnSynchronizationImageBased();

  // Set the parameters of progression spin box and slider
  void SetProgressionParameters();

  // Update camera stream pos when slider is changed
  void OnSliderChanged();

  // Update camera stream pos when spin box is changed
  void OnSpinBoxChanged();

  // Launch the dialog to set parameters for color calibration
  void OnLaunchDialogForColorCalibration();

  // Launch the dialog to set the parameters of the export video
  void OnLaunchExportVideoDialog();

  // Launch the dialog to set the parameters of synchronization
  void OnLaunchSyncDialog();

  // Decide to align or not the basleine
  void OnShouldAlignBaseline();

  // Import the time synchronisation
  void OnImportTimeSync();

  // Export the time synchronisation
  void OnExportTimeSync();

  // Interrupt the export video
  void OnStopExportVideo();

  // Method called when the windows is resized
  void resizeEvent(QResizeEvent* event);

  // Method called when a key is pressed
  void keyPressEvent(QKeyEvent* event);

private:
  // the UI object, to access the UI elements created with Qt Designer
  Ui::MainWindow *ui;

  // the left and right pictures, converted to OpenCV Mat format
  cv::Mat LeftImage;
  cv::Mat RightImage;

  // Store the name of the video file
  std::string FilenameLeft;
  std::string FilenameRight;

  // The StereoVideo manager
  StereoVideoManager StereoVideo;

  // The color calibrator
  ColorCalibrator LeftColorCalibrator;
  ColorCalibrator RighttColorCalibrator;

  // Results image of the last operation
  std::vector<cv::Mat> LastOperationResults;
  cv::Mat LeftResult;
  cv::Mat RightResult;
  int indexResultToShow;

  // Dialog to select calibration parameters
  CalibrationParametersDialog* ParamDialog;

  // Dialog to select distance information parameters
  DistanceInfoDialog* distanceInfoDialog;

  // Dialog to choose the parameters of the simulation
  SimulateDialog* simulateDialog;

  // Dialog to choose the color calibration parameters
  ColorCalibrationDialog* colorDialog;

  // Dialog to choose the computation options before exporting
  ExportVideoDialog* exportDialog;

  // Dialog to choose the parameters of the synchronization computation
  SynchronizationDialog* syncDialog;

  // Timers to play video
  QTimer* PlayTimer;
  QTimer* CalibrationProgressTimer;

  // ProgressBar
  QProgressBar* Progress;

  // save management
  bool IsSaving;
  bool IsWriterOpen;
  bool IsColorCalibrated;
  int ColorCalibrationMethod;
  double WhiteSaturation;
  bool ShouldWhiteBalance;
  bool ShouldInterruptExport;
  cv::VideoWriter vidWriter1;
  cv::VideoWriter vidWriter2;

  std::string exportFilenameLeft;
  std::string exportFilenameRight;
  double TimeOffset;
};

// Convert a cv::Mat image to QImage
QImage  cvMatToQImage(const cv::Mat &inMat);

#endif // MAINWINDOW_H
