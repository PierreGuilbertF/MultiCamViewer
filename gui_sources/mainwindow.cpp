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

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  this->setWindowModality(Qt::ApplicationModal);

  this->ParamDialog = new CalibrationParametersDialog(this);
  this->distanceInfoDialog = new DistanceInfoDialog(this);
  this->simulateDialog = new SimulateDialog(this);
  this->colorDialog = new ColorCalibrationDialog(this);
  this->exportDialog = new ExportVideoDialog(this);
  this->syncDialog = new SynchronizationDialog(this);

  this->ui->actionCropResult->setChecked(true);

  this->PlayTimer = new QTimer(this);
  this->CalibrationProgressTimer = new QTimer(this);

  this->Progress = new QProgressBar(this->statusBar());
  this->Progress->setMinimum(0);
  this->Progress->setMaximum(100);
  this->Progress->resize(this->width(), this->Progress->height());

  // connect the left and right buttons to the corresponding slot
  connect(ui->pushButtonLeft, SIGNAL(clicked(bool)), this, SLOT(onOpenLeftVideo()));
  connect(ui->pushButtonRight, SIGNAL(clicked(bool)), this, SLOT(onOpenRightVideo()));

  // connect the elements of the status bar
  connect(ui->actionExportCalibration, SIGNAL(triggered(bool)), this, SLOT(OnExportCalibration()));
  connect(ui->actionImportCalibration, SIGNAL(triggered(bool)), this, SLOT(OnImportCalibration()));
  connect(ui->actionCalibrateVideo, SIGNAL(triggered(bool)), this, SLOT(OnLaunchDialogForCalibration()));
  connect(ui->actionSaveRectifiedImages, SIGNAL(triggered(bool)), this, SLOT(OnSaveRectifiedImages()));
  connect(ui->actionConfigureAndExportVideo, SIGNAL(triggered(bool)), this, SLOT(OnSaveVideo()));
  connect(ui->actionCalibrateColor, SIGNAL(triggered(bool)), this, SLOT(OnLaunchDialogForColorCalibration()));
  connect(ui->actionPlay, SIGNAL(triggered(bool)), this, SLOT(OnPlay()));
  connect(ui->actionSynchronization, SIGNAL(triggered(bool)), this, SLOT(OnLaunchSyncDialog()));
  connect(ui->actionDistanceInformation, SIGNAL(triggered(bool)), this, SLOT(OnLaunchDistanceInfoDialog()));
  connect(ui->actionImportTimeSync, SIGNAL(triggered(bool)), this, SLOT(OnImportTimeSync()));
  connect(ui->actionExportTimeSync, SIGNAL(triggered(bool)), this, SLOT(OnExportTimeSync()));
  connect(ui->actionStopExportVideo, SIGNAL(triggered(bool)), this, SLOT(OnStopExportVideo()));
  connect(ui->actionCropResult, SIGNAL(triggered(bool)), this, SLOT(OnTimerPlayOut()));
  connect(ui->actionAlignBaseline, SIGNAL(triggered(bool)), this, SLOT(OnShouldAlignBaseline()));
  connect(this->simulateDialog->GetLoadButton(), SIGNAL(clicked()), this, SLOT(OnLoadCalibrationFromSimulation()));
  connect(this->ui->actionSimulate, SIGNAL(triggered(bool)), this, SLOT(OnLaunchSimulationDialog()));
  connect(this->PlayTimer, SIGNAL(timeout()), this, SLOT(OnTimerPlayOut()));
  connect(this->CalibrationProgressTimer, SIGNAL(timeout()), this, SLOT(OnTimerCalibrationOut()));
  connect(this->ParamDialog, SIGNAL(accepted()), this, SLOT(OnCalibrate()));
  connect(this->distanceInfoDialog, SIGNAL(accepted()), this, SLOT(OnDistanceInfo()));
  connect(this->simulateDialog, SIGNAL(accepted()), this, SLOT(OnSimulateAcquisition()));
  connect(this->colorDialog, SIGNAL(accepted()), this, SLOT(OnCalibrateColor()));
  connect(this->syncDialog, SIGNAL(accepted()), this, SLOT(OnSynchronizationImageBased()));
  connect(this->ui->progressionSlider, SIGNAL(sliderReleased()), this, SLOT(OnSliderChanged()));
  connect(this->ui->progressionSpinBox, SIGNAL(valueChanged(int)), this, SLOT(OnSpinBoxChanged()));

  this->ui->actionStopExportVideo->setVisible(false);

  this->IsWriterOpen = false;
  this->IsSaving = false;
  this->IsColorCalibrated = false;
  this->WhiteSaturation = 0.035;
  this->StereoVideo.SetProgressBar(this->Progress);
  this->TimeOffset = 0;
  this->ShouldInterruptExport = false;
}

//---------------------------------------------------------------------------
MainWindow::~MainWindow()
{
  delete ui;
}

//---------------------------------------------------------------------------
void MainWindow::ShowImageLeft()
{
  if (!this->LeftImage.data)
  {
    return;
  }
  // Qt display
  // we load the picture from the file, to display it in a QLabel in the GUI
  QImage leftPicture = cvMatToQImage(this->LeftImage);
  //leftPicture.load(filename);

  // some computation to resize the image if it is too big to fit in the GUI
  QPixmap leftPixmap = QPixmap::fromImage(leftPicture);
  int max_width  = std::min(ui->label_image_left->width(), leftPicture.width());
  int max_height = std::min(ui->label_image_left->height(), leftPicture.height());
  ui->label_image_left->setPixmap(leftPixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
}

//---------------------------------------------------------------------------
void MainWindow::OnStopExportVideo()
{
  this->ShouldInterruptExport = true;
}

//---------------------------------------------------------------------------
void MainWindow::ShowImageRight()
{
  if (!this->RightImage.data)
  {
    return;
  }
  ///// Qt display stuff
  // we load the picture from the file, to display it in a QLabel in the GUI
  QImage rightPicture = cvMatToQImage(this->RightImage);

  // some computation to resize the image if it is too big to fit in the GUI
  QPixmap rightPixmap = QPixmap::fromImage(rightPicture);
  int max_width  = std::min(ui->label_image_right->width(), rightPicture.width());
  int max_height = std::min(ui->label_image_right->height(), rightPicture.height());
  ui->label_image_right->setPixmap(rightPixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
}

//---------------------------------------------------------------------------
void MainWindow::ShowImageLastResult()
{
  if (!this->RightResult.data)
  {
    return;
  }
  ///// Qt display stuff
  // we load the picture from the file, to display it in a QLabel in the GUI
  QImage opPicture = cvMatToQImage(this->RightResult);

  // some computation to resize the image if it is too big to fit in the GUI
  QPixmap opPixmap = QPixmap::fromImage(opPicture);
  int max_width  = std::min(ui->label_image_right->width(), opPicture.width());
  int max_height = std::min(ui->label_image_right->height(), opPicture.height());
  ui->resultLabel->setPixmap(opPixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
}

//---------------------------------------------------------------------------
void MainWindow::ShowImageLastResultLeft()
{
  if (!this->LeftResult.data)
  {
    return;
  }
  ///// Qt display stuff
  // we load the picture from the file, to display it in a QLabel in the GUI
  QImage opPicture = cvMatToQImage(this->LeftResult);

  // some computation to resize the image if it is too big to fit in the GUI
  QPixmap opPixmap = QPixmap::fromImage(opPicture);
  int max_width  = std::min(ui->label_image_left->width(), opPicture.width());
  int max_height = std::min(ui->label_image_left->height(), opPicture.height());
  ui->resultLabelLeft->setPixmap(opPixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
}

//---------------------------------------------------------------------------
void MainWindow::onOpenLeftVideo()
{
  // we prompt the user with a file dialog,
  // to select the picture file from the left camera
  QString filename = QFileDialog::getOpenFileName(this, "Select left picture file", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
      return;
  }

  // we convert filename from QString to std::string (needed by OpenCV)
  std::string filename_s = filename.toUtf8().constData();
  this->FilenameLeft = filename_s;

  this->StereoVideo.OpenVideo1(filename_s);

  // we load the picture in the OpenCV Mat format, to compute depth map
  //this->LeftImage = cv::imread(filename_s, CV_LOAD_IMAGE_COLOR);
  this->StereoVideo.GetVideo(0)->read(this->LeftImage);

  this->ShowImageLeft();

  if (this->StereoVideo.GetIsStereoReady())
  {
    this->SetProgressionParameters();
  }

  return;
}

//---------------------------------------------------------------------------
void MainWindow::onOpenRightVideo()
{
  // we prompt the user with a file dialog,
  // to select the picture file from the left camera
  QString filename = QFileDialog::getOpenFileName(this, "Select right picture file", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
      return;
  }

  // we convert filename from QString to std::string (needed by OpenCV)
  std::string filename_s = filename.toUtf8().constData();
  this->FilenameRight = filename_s;

  this->StereoVideo.OpenVideo2(filename_s);

  // we load the picture in the OpenCV Mat format, to compute depth map
  //this->RightImage = cv::imread(filename_s, CV_LOAD_IMAGE_COLOR);
  this->StereoVideo.GetVideo(1)->read(this->RightImage);

  this->ShowImageRight();

  if (this->StereoVideo.GetIsStereoReady())
  {
    this->SetProgressionParameters();
  }

  return;
}

//---------------------------------------------------------------------------
void MainWindow::OnCalibrate()
{
  bool shouldExportResults = this->ParamDialog->GetExportImageResult();
  std::string filename_s;
  // Choose the filename export
  if (shouldExportResults)
  {
    QString filename = QFileDialog::getSaveFileName(this, "Select export result file name", QDir::homePath(), NULL);
    if (filename.isNull() || filename.isEmpty())
    {
      std::cout << "No file selected" << std::endl;
      return;
    }

    filename_s = filename.toStdString();
  }

  QMessageBox* msgBox = new QMessageBox(this);
  msgBox->show();
  msgBox->setAttribute(Qt::WA_DeleteOnClose);
  msgBox->setStandardButtons(QMessageBox::Ok);
  msgBox->setText(tr("Please wait while geometrical calibration is processing. This operation can take severals minutes depending on the nbr of frames used"));
  msgBox->setWindowTitle("Geometrical Calibration...");
  msgBox->setModal(true);
  msgBox->update();

  int NbrFrameToUse = this->ParamDialog->GetNbrImToUse();
  CalibratorParams paramCurrent;
  paramCurrent.contrastThreshold = this->ParamDialog->GetContrastThreshold();
  paramCurrent.edgeThreshold = this->ParamDialog->GetEdgeThreshold();
  paramCurrent.nFeatures = this->ParamDialog->GetNFeatures();
  paramCurrent.nOctaves = this->ParamDialog->GetNOctaves();
  paramCurrent.param1 = this->ParamDialog->GetParam1();
  paramCurrent.param2 = this->ParamDialog->GetParam2();
  paramCurrent.ratioMatchingThresh = this->ParamDialog->GetRatioMatchingThresh();
  paramCurrent.sigma = this->ParamDialog->GetSigma();

  // Set parameters
  this->StereoVideo.SetUsePrior(true);
  this->StereoVideo.SetCalibratorParameters(paramCurrent);

  // Show timer
  /*QProgressDialog progress("Computing calibration...", "Cancel", 0, 100, this);
  progress.setWindowModality(Qt::WindowModal);
  progress.setValue(0);
  progress.show();
  this->CalibrationProgressTimer->start(1);*/

  if (shouldExportResults)
  {
    this->StereoVideo.SetShouldExport(true, filename_s);
  }

  // disable the mainwindow while the calibration is computing
  this->setEnabled(false);

  // calibration computation
  this->StereoVideo.SolidCameraCalibration(NbrFrameToUse, paramCurrent);

  // enable the mainwindow
  this->setEnabled(true);

  if (shouldExportResults)
  {
    this->StereoVideo.SetShouldExport(false, filename_s);
  }
  //this->CalibrationProgressTimer->stop();
  msgBox->close();
}

//---------------------------------------------------------------------------
QImage  cvMatToQImage( const cv::Mat &inMat )
{
  switch ( inMat.type() )
  {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
      //std::cout << "CV_8UC4 type" << std::endl;
      //std::cout << CV_8UC4 << " ; " << inMat.type() << std::endl;
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_ARGB32 );

      return image;
    }

    // 8-bit, 3 channel
    case CV_8UC3:
    {
      //std::cout << "CV_8UC3 type" << std::endl;
      //std::cout << CV_8UC3 << " ; " << inMat.type() << std::endl;
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_RGB888 );

      return image.rgbSwapped();
    }

    // 8-bit, 1 channel
    case CV_8UC1:
    {
      //std::cout << "CV_8UC1 type" << std::endl;
      //std::cout << CV_8UC1 << " ; " << inMat.type() << std::endl;
      QImage image( inMat.data,
                    inMat.cols, inMat.rows,
                    static_cast<int>(inMat.step),
                    QImage::Format_Grayscale8 );

      return image;
    }

    default:
      std::cout << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type() << std::endl;
      break;
  }

  return QImage();
}

//---------------------------------------------------------------------------
void MainWindow::resizeEvent(QResizeEvent* event)
{
  /*double ratio = 2.0;
  QSize newSize = this->size();
  ui->label_image_left->resize(newSize.width() / ratio, newSize.height() / ratio);
  ui->label_image_right->resize(newSize.width() / ratio, newSize.height() / ratio);
  ui->resultLabel->resize(newSize.width() / ratio, newSize.height() / ratio);*/

  this->ShowImageLeft();
  this->ShowImageRight();
  this->ShowImageLastResult();
}

//---------------------------------------------------------------------------
void MainWindow::keyPressEvent(QKeyEvent* event)
{
  /*if (event->key() == Qt::Key_Q)
  {
    int maxN = this->LastOperationResults.size();
    if (maxN < 2)
    {
      return;
    }

    this->indexResultToShow = (this->indexResultToShow - 1) % maxN;
    this->ShowImageLastResult();
  }

  if (event->key() == Qt::Key_D)
  {
    int maxN = this->LastOperationResults.size();
    if (maxN < 2)
    {
      return;
    }

    this->indexResultToShow = (this->indexResultToShow + 1) % maxN;
    this->ShowImageLastResult();
  }*/
}

//---------------------------------------------------------------------------
void MainWindow::OnImportCalibration()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video before"));
    return;
  }

  QString filename = QFileDialog::getOpenFileName(this, "Select calibration import", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  std::string filename_s = filename.toStdString();
  std::ifstream file;
  file.open(filename_s);
  file.precision(20);
  if (!file.is_open())
  {
    std::cout << "Error opening file : " << filename_s << std::endl;
  }

  // double Mat (6 = double)
  cv::Mat currF = cv::Mat::zeros(3, 3, 6);

  std::vector<cv::KeyPoint> leftK;
  std::vector<cv::KeyPoint> rightK;

  // read the file
  std::string line;
  int tokenEncountered = 0;
  int matrixLine = 0;
  while (std::getline(file, line))
  {
    if (line[0] == '!')
    {
      tokenEncountered++;
      continue;
    }

    // Fundamental matrix data
    if (tokenEncountered == 1)
    {
      std::istringstream iss(line);
      double a, b, c;
      iss >> a >> b >> c;
      currF.at<double>(matrixLine, 0) = a;
      currF.at<double>(matrixLine, 1) = b;
      currF.at<double>(matrixLine, 2) = c;
      matrixLine++;
    }
    // Pair matches data
    else if (tokenEncountered == 3)
    {
      std::istringstream iss(line);
      double xl, yl, xr, yr;
      iss >> xl >> yl >> xr >> yr;
      cv::KeyPoint L, R;
      L.pt.x = xl;
      L.pt.y = yl;
      R.pt.x = xr;
      R.pt.y = yr;
      leftK.push_back(L);
      rightK.push_back(R);
    }
    else
    {
      std::cout << "Oups ! Error while parsing calib data" << std::endl;
    }
  }

  this->StereoVideo.SetF(currF, leftK, rightK);

  this->OnTimerPlayOut();
}

//---------------------------------------------------------------------------
void MainWindow::OnExportCalibration()
{
  if (!this->StereoVideo.GetIsCalibrated())
  {
    QMessageBox::information(this, tr("Warning"), tr("No calibration has been made, please run a calibration before exporting"));
    return;
  }

  QString filename = QFileDialog::getSaveFileName(this, "Select calibration export", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  std::string filename_s = filename.toStdString();
  std::ofstream file;
  file.open(filename_s);
  if (!file.is_open())
  {
    std::cout << "Error opening file : " << filename_s << std::endl;
  }

  file.precision(20);
  file << "!#!  Fundamental matrix !#!" << std::endl;
  cv::Mat F = this->StereoVideo.GetF();
  for (int i = 0; i < F.rows; ++i)
  {
    for (int j = 0; j < F.cols; ++j)
    {
      file << F.at<double>(i, j) << " ";
    }
    file << std::endl;
  }

  file << "!#! Matches : point1 <-> point2 !#!" << std::endl;
  file << "!#! Xl, Yl, Xr, Yr !#!" << std::endl;

  std::vector<cv::KeyPoint> leftMatches = this->StereoVideo.GetLeftMatched();
  std::vector<cv::KeyPoint> rightMatches = this->StereoVideo.GetRightMatched();

  for (int i = 0; i < leftMatches.size(); ++i)
  {
    cv::Point2f L = leftMatches[i].pt;
    cv::Point2f R = rightMatches[i].pt;

    file << L.x << " " << L.y << " " << R.x << " " << R.y << std::endl;
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnImportTimeSync()
{
  QString filename = QFileDialog::getOpenFileName(this, "Select time sync import", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  std::string filename_s = filename.toStdString();
  std::ifstream file;
  file.open(filename_s);
  file.precision(20);
  if (!file.is_open())
  {
    std::cout << "Error opening file : " << filename_s << std::endl;
  }

  std::string line1, line2;
  std::getline(file, line1);
  std::getline(file, line2);

  std::istringstream iss(line2);
  iss >> this->TimeOffset;

  if (this->TimeOffset >= 0)
  {
    std::cout << "Signal left is in advance and right is late" << std::endl;
    double finalTimeSync = this->TimeOffset;
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(0.0, finalTimeSync);
  }
  else
  {
    std::cout << "Signal right is in advance and left is late" << std::endl;
    double finalTimeSync = -this->TimeOffset;
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(finalTimeSync, 0.0);
    this->TimeOffset = -finalTimeSync;
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnExportTimeSync()
{
  QString filename = QFileDialog::getSaveFileName(this, "Select time sync export", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  std::string filename_s = filename.toStdString();
  std::ofstream file;
  file.open(filename_s);
  if (!file.is_open())
  {
    std::cout << "Error opening file : " << filename_s << std::endl;
  }

  file.precision(20);
  file << "!#!  Time synchronisation !#!" << std::endl;
  file << this->TimeOffset << std::endl;
  file.close();
}

//---------------------------------------------------------------------------
void MainWindow::OnLaunchDialogForCalibration()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video"));
    return;
  }

  this->ParamDialog->show();
}

//---------------------------------------------------------------------------
void MainWindow::OnLaunchExportVideoDialog()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video"));
    return;
  }

  this->exportDialog->show();
}

void MainWindow::OnLaunchSyncDialog()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video"));
    return;
  }

  this->syncDialog->show();
}

//---------------------------------------------------------------------------
void MainWindow::OnSaveVideo()
{
  // select export files
  QString filename = QFileDialog::getSaveFileName(this, "Select Video Export", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  // get output filename information
  QFileInfo saveFilenameInfo(filename);
  QString exportFolder = saveFilenameInfo.absoluteDir().absolutePath();
  QString exportFilename = saveFilenameInfo.fileName();

  // get video extension
  QFileInfo internFileInfoLeft(QString(this->FilenameLeft.c_str()));
  QFileInfo internFileInfoRight(QString(this->FilenameLeft.c_str()));
  QString extensionLeft = internFileInfoLeft.suffix();
  QString extensionRight = internFileInfoRight.suffix();

  std::string filename_s = filename.toStdString();
  QString exportFilenameLeft = exportFolder + "/" + exportFilename + "Left" + "." + extensionLeft;
  QString exportFilenameRight = exportFolder + "/" + exportFilename + "Right" + "." + extensionLeft;

  // find output frames format
  cv::Mat frame1, frame2;
  this->StereoVideo.GetVideo(1)->read(frame2);
  this->StereoVideo.GetVideo(0)->read(frame1);
  cv::Size s1 = frame1.size();
  cv::Size s2 = frame2.size();
  if (this->StereoVideo.GetIsCalibrated())
  {
    cv::Mat frameTemp1, frameTemp2;
    
    // warp using computed homography
    std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > > bounds = this->StereoVideo.WarpCalibration(frame1, frame2, this->LeftResult, this->RightResult);

    std::cout << "Corners of left image: " << std::endl;
    for (unsigned int k = 0; k < bounds.first.size(); ++k)
    {
      std::cout << "[" << bounds.first[k].x << "," << bounds.first[k].y << "]" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Corners of right image: " << std::endl;
    for (unsigned int k = 0; k < bounds.second.size(); ++k)
    {
      std::cout << "[" << bounds.second[k].x << "," << bounds.second[k].y << "]" << std::endl;
    }
    std::cout << std::endl;

    double aspectRatioL = 1.0 * frame1.cols / frame1.rows;
    double aspectRatioR = 1.0 * frame2.cols / frame2.rows;

    double y1 = DiagLengthRectangleInsidePolygone(bounds.first, aspectRatioL);
    double y2 = DiagLengthRectangleInsidePolygone(bounds.second, aspectRatioR);

    std::pair<cv::Rect, cv::Rect> cropRectangle = KeepSmallerRectangle(bounds.first, bounds.second, y1, y2, aspectRatioL, aspectRatioR);

    if (this->ui->actionCropResult->isChecked())
    {
      // Crop the images according to the rectangles
      cv::Mat croppedLeft(this->LeftResult, cropRectangle.first);
      cv::Mat croppedRight(this->RightResult, cropRectangle.second);

      // Copy result
      frameTemp1 = croppedLeft.clone();
      frameTemp2 = croppedRight.clone();
    }
    else
    {
      cv::Rect cropRectangleLeft = CenteredRectangle(bounds.first, frame1.rows, frame1.cols);
      cv::Rect cropRectangleRight = CenteredRectangle(bounds.second, frame2.rows, frame2.cols);

      // Crop the images according to the rectangles
      cv::Mat croppedLeft(this->LeftResult, cropRectangleLeft);
      cv::Mat croppedRight(this->RightResult, cropRectangleRight);

      // Resize the images to fit the original images sizes
      frameTemp1 = croppedLeft.clone();
      frameTemp2 = croppedRight.clone();
    }

    s1 = frameTemp1.size();
    s2 = frameTemp2.size();
  }

  // create the video streams
  this->vidWriter1.open(exportFilenameLeft.toStdString().c_str(), CV_FOURCC('M', 'J', 'P', 'G'), this->StereoVideo.GetVideo(0)->get(CV_CAP_PROP_FPS), s1);
  this->vidWriter2.open(exportFilenameRight.toStdString().c_str(), CV_FOURCC('M', 'J', 'P', 'G'), this->StereoVideo.GetVideo(1)->get(CV_CAP_PROP_FPS), s2);

  // reset the StereoVideo position
  this->StereoVideo.ResetStreamPosition();

  int nbrFrame1 = this->StereoVideo.GetVideoInfo(0).NbrFrame;
  int nbrFrame2 = this->StereoVideo.GetVideoInfo(1).NbrFrame;

  clock_t loopBegin, loopEnd;
  clock_t cloneBegin, cloneEnd;
  clock_t colorBegin, colorEnd;
  clock_t warpBegin, warpEnd;
  clock_t writeBegin, writeEnd;
  double loopT, cloneT, colorT, warpT, writeT;

  loopBegin = clock();
  int count = 0;
  // Save the videos
  while (this->StereoVideo.GetVideo(1)->read(this->RightImage) && this->StereoVideo.GetVideo(0)->read(this->LeftImage))
  {
    if (this->ShouldInterruptExport)
    {
      this->ShouldInterruptExport = false;
      break;
    }

    loopEnd = clock();
    loopT = double(loopEnd - loopBegin) / CLOCKS_PER_SEC;
    loopBegin = clock();

    // Update GUI progress things
    int pos1 = this->StereoVideo.GetVideo(0)->get(CV_CAP_PROP_POS_FRAMES);
    int pos2 = this->StereoVideo.GetVideo(1)->get(CV_CAP_PROP_POS_FRAMES);
    int nbrFrame1 = this->StereoVideo.GetVideoInfo(0).NbrFrame;
    int nbrFrame2 = this->StereoVideo.GetVideoInfo(1).NbrFrame;
    double rel1 = 1.0 * pos1 / nbrFrame1;
    double rel2 = 1.0 * pos2 / nbrFrame2;
    this->Progress->setValue(100.0 * std::max(rel1, rel2));
    this->ui->progressionSlider->setValue(std::min(pos1, pos2));
    this->ui->progressionSpinBox->setValue(std::min(pos1, pos2));

    if (count % 150 == 0)
    {
      this->update();
      this->Progress->update();
    }

    cloneBegin = clock();
    cv::Mat temp2 = this->RightImage.clone();
    cv::Mat temp1 = this->LeftImage.clone();

    cv::Mat I0 = this->LeftImage.clone();
    cv::Mat I1 = this->RightImage.clone();
    cloneEnd = clock();

    colorBegin = clock();
    if (this->ui->actionCalibrateColor->isChecked())
    {
        if (this->ShouldWhiteBalance)
        {
            WhiteBalanceGreyAssumption(this->LeftImage, I0, this->WhiteSaturation);
            WhiteBalanceGreyAssumption(this->RightImage, I1, this->WhiteSaturation);
        }

        LeftColorCalibrator.AddImage(I0);
        RighttColorCalibrator.AddImage(I1);

        if (this->ColorCalibrationMethod == 1)
        {
            double moyTarget = RighttColorCalibrator.GetMoy();
            double sigmaTarget = RighttColorCalibrator.GetSigma();
            LeftColorCalibrator.MatchTargetCalibration(I0, temp1, moyTarget, sigmaTarget);

            temp2 = I1.clone();
        }
        else
        {
            double moyTarget = LeftColorCalibrator.GetMoy();
            double sigmaTarget = LeftColorCalibrator.GetSigma();
            RighttColorCalibrator.MatchTargetCalibration(I1, temp2, moyTarget, sigmaTarget);

            temp1 = I1.clone();
        }
    }
    colorEnd = clock();

    warpBegin = clock();
    if (this->StereoVideo.GetIsCalibrated())
    {
      // warp using computed homography
      std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > > bounds = this->StereoVideo.WarpCalibration(temp1, temp2, this->LeftResult, this->RightResult);

      std::cout << "Corners of left image: " << std::endl;
      for (unsigned int k = 0; k < bounds.first.size(); ++k)
      {
        std::cout << "[" << bounds.first[k].x << "," << bounds.first[k].y << "]" << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Corners of right image: " << std::endl;
      for (unsigned int k = 0; k < bounds.second.size(); ++k)
      {
        std::cout << "[" << bounds.second[k].x << "," << bounds.second[k].y << "]" << std::endl;
      }
      std::cout << std::endl;

      double aspectRatioL = 1.0 * this->LeftImage.cols / this->LeftImage.rows;
      double aspectRatioR = 1.0 * this->RightImage.cols / this->RightImage.rows;

      double y1 = DiagLengthRectangleInsidePolygone(bounds.first, aspectRatioL);
      double y2 = DiagLengthRectangleInsidePolygone(bounds.second, aspectRatioR);

      std::pair<cv::Rect, cv::Rect> cropRectangle = KeepSmallerRectangle(bounds.first, bounds.second, y1, y2, aspectRatioL, aspectRatioR);

      if (this->ui->actionCropResult->isChecked())
      {
        // Crop the images according to the rectangles
        cv::Mat croppedLeft(this->LeftResult, cropRectangle.first);
        cv::Mat croppedRight(this->RightResult, cropRectangle.second);

        // Copy result
        this->LeftResult = croppedLeft.clone();
        this->RightResult = croppedRight.clone();
      }
      else
      {
        cv::Rect cropRectangleLeft = CenteredRectangle(bounds.first, I0.rows, I0.cols);
        cv::Rect cropRectangleRight = CenteredRectangle(bounds.second, I1.rows, I1.cols);
        
        // Crop the images according to the rectangles
        cv::Mat croppedLeft(this->LeftResult, cropRectangleLeft);
        cv::Mat croppedRight(this->RightResult, cropRectangleRight);

        // Resize the images to fit the original images sizes
        this->LeftResult = croppedLeft.clone();
        this->RightResult = croppedRight.clone();
      }
    }
    else
    {
      this->LeftResult = temp1;
      this->RightResult = temp2;
    }
    warpEnd = clock();

    // export images
    std::cout << "Size Given1 : " << this->LeftResult.size() << std::endl;
    std::cout << "Size Given2 : " << this->RightResult.size() << std::endl;

    writeBegin = clock();
    this->vidWriter1.write(this->LeftResult);
    this->vidWriter2.write(this->RightResult);
    writeEnd = clock();

    // Time processing information
    colorT = double(colorEnd - colorBegin) / CLOCKS_PER_SEC;
    writeT = double(writeEnd - writeBegin) / CLOCKS_PER_SEC;
    warpT = double(warpEnd - warpBegin) / CLOCKS_PER_SEC;
    cloneT = double(cloneEnd - cloneBegin) / CLOCKS_PER_SEC;

    std::cout << "-------TIME INFO---------" << std::endl;
    std::cout << "Current processing FPS : " << 1.0 / loopT << std::endl;
    std::cout << "processing frame number : " << count << std::endl;
    std::cout << "Advencement in percentage : " << 1.0 * count / (1.0 * nbrFrame1) * 100 << std::endl;
    std::cout << "loop time processing : " << loopT << "s => " << loopT / loopT * 100 << std::endl;
    std::cout << "color time processing : " << colorT << "s => " << colorT / loopT * 100 << std::endl;
    std::cout << "warp time processing : " << warpT << "s => " << warpT / loopT * 100 << std::endl;
    std::cout << "clone time processing : " << cloneT << "s => " << cloneT / loopT * 100 << std::endl;
    std::cout << "write time processing : " << writeT << "s => " << writeT / loopT * 100 << std::endl;
    std::cout << "--------------------------" << std::endl;
    count++;
  }

  // close the streams
  this->vidWriter1.release();
  this->vidWriter2.release();

  // check if the export song are required
  bool shouldExportSongLeft = true;
  bool shouldExportSongRight = true;
  if (std::abs(this->TimeOffset) > 1e-3)
  {
    if (this->TimeOffset >= 0)
    {
      // left image
      shouldExportSongLeft = true;
      shouldExportSongRight = false;
    }
    else
    {
      // right image
      shouldExportSongLeft = false;
      shouldExportSongRight = true;
    }
  }
  // enforce .mov filetype for temporary song fileformat for garanteed codec compatibility
  QString songFileExt = (extensionLeft.toLower()=="mov")?extensionLeft:"aac";
  QString leftSong = exportFolder + "/" + "LeftSongExtracted." + songFileExt;
  QString rightSong = exportFolder + "/" + "RightSongExtracted." + songFileExt;

  // Extract the songs
  if (shouldExportSongLeft)
  {
    QString commandExtractSong = "./ffmpeg.exe -i " + QString(this->FilenameLeft.c_str()) + " -vn -acodec copy " + leftSong;
    QString delPrevious = "del " + leftSong;
    commandExtractSong.replace("/", "\\");
    delPrevious.replace("/", "\\");
    system(delPrevious.toStdString().c_str()); // Erase the file if already exist
    system(commandExtractSong.toStdString().c_str());
  }
  if (shouldExportSongRight)
  {
    QString commandExtractSong = "./ffmpeg.exe -i " + QString(this->FilenameRight.c_str()) + " -vn -acodec copy " + rightSong;
    QString delPrevious = "del " + rightSong;
    commandExtractSong.replace("/", "\\");
    delPrevious.replace("/", "\\");
    system(delPrevious.toStdString().c_str()); // Erase the file if already exist
    system(commandExtractSong.toStdString().c_str());
  }

  // temporary files to add song
  QString leftVideoWithSong = exportFolder + "/" + exportFilename + "LeftWithSong." + extensionLeft;
  QString rightVideoWithSong = exportFolder + "/" + exportFilename + "RightWithSong." + extensionRight;

  if (shouldExportSongLeft)
  {
    QString commandLinkStreams = "./ffmpeg.exe -i " + exportFilenameLeft + " -i " + leftSong + " -c copy -map 0:v:0 -map 1:a:0 " + leftVideoWithSong + " -shortest";
    QString delCom = "del " + leftVideoWithSong;
    commandLinkStreams.replace("/", "\\");
    delCom.replace("/", "\\");
    system(delCom.toStdString().c_str()); // delete old file if already exist
    system(commandLinkStreams.toStdString().c_str());
  }
  if (shouldExportSongRight)
  {
    QString commandLinkStreams = "./ffmpeg.exe -i " + exportFilenameRight + " -i " + rightSong + " -c copy -map 0:v:0 -map 1:a:0 " + rightVideoWithSong + " -shortest";
    QString delCom = "del " + rightVideoWithSong;
    commandLinkStreams.replace("/", "\\");
    delCom.replace("/", "\\");
    system(delCom.toStdString().c_str()); // delete old file if already exist
    system(commandLinkStreams.toStdString().c_str());
  }

  // now rename temp file
  //QString outputNameQ(outputName.c_str());
  //outputNameQ.replace('/', '\\');
  //std::string renameCommand = "ren " + outputNameQ.toStdString() + " " + newName;
  //std::cout << "rename command is : " << renameCommand << std::endl;
  //system(renameCommand.c_str());
 
  // Enable the mainwindow
  this->setEnabled(true);
}

//---------------------------------------------------------------------------
void MainWindow::OnSaveRectifiedImages()
{
  if (this->IsSaving)
  {
    this->IsSaving = false;
    this->IsWriterOpen = false;

    // close the streams
    this->vidWriter1.release();
    this->vidWriter2.release();

    std::string videoToAddSong;
    std::string videoToExtractSong;
    if (this->TimeOffset >= 0)
    {
      // left image
      videoToAddSong = this->exportFilenameLeft;
      videoToExtractSong = this->FilenameLeft;
    }
    else
    {
      videoToAddSong = this->exportFilenameRight;
      videoToExtractSong = this->FilenameRight;
    }

    // Extract the song
    std::string commandExtractSong = "./ffmpeg.exe -i " + videoToExtractSong + " -vn -acodec copy songExtracted.aac";
    system("del songExtracted.aac"); // Erase the file if already exist
    system(commandExtractSong.c_str());

    // Output filename
    std::string outputName = videoToAddSong;
    int L1 = outputName.length() - 1;
    while (L1 > 0)
    {
      if (outputName[L1] == '.')
      {
        break;
      }
      L1--;
    }
    outputName[L1 - 1] = 'A';

    // Link the song with the exported video
    std::string commandLinkStreams = "./ffmpeg.exe -i " + videoToAddSong + " -i songExtracted.aac -c copy -map 0:v:0 -map 1:a:0 " + outputName + " -shortest";
    std::string delCom = "del " + outputName;
    //system(delCom.c_str()); // delete old file if already exist
    system(commandLinkStreams.c_str());

    // delete temporary files
    system("del songExtracted.aac");
    QString videoToAddSongQ(videoToAddSong.c_str());
    videoToAddSongQ.replace('/', '\\');
    delCom = "del " + videoToAddSongQ.toStdString();
    std::cout << "delete command is : " << delCom << std::endl;
    //system(delCom.c_str());

    L1 = videoToAddSong.length() - 1;
    while (L1 > 0)
    {
      if (videoToAddSong[L1] == '/')
      {
        break;
      }
      L1--;
    }
    std::string newName = videoToAddSong.substr(L1 + 1);

    // now rename temp file
    QString outputNameQ(outputName.c_str());
    outputNameQ.replace('/', '\\');
    std::string renameCommand = "ren " + outputNameQ.toStdString() + " " + newName;
    std::cout << "rename command is : " << renameCommand << std::endl;
    system(renameCommand.c_str());

    return;
  }

  QString filename = QFileDialog::getSaveFileName(this, "Select Video Export", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  // find video extension
  std::string extensionLeft;
  int L1 = this->FilenameLeft.length() - 1;
  while (L1 > 0)
  {
    if (this->FilenameLeft[L1] == '.')
    {
      break;
    }
    L1--;
  }

  extensionLeft = this->FilenameLeft.substr(L1 + 1);

  std::string filename_s = filename.toStdString();
  std::string filenameLeft = filename_s + "Left." + extensionLeft;
  std::string filenameRight = filename_s + "Right." + extensionLeft;
  this->exportFilenameRight = filenameRight;
  this->exportFilenameLeft = filenameLeft;

  cv::Mat frame1, frame2;
  this->StereoVideo.GetVideo(1)->read(frame2);
  this->StereoVideo.GetVideo(0)->read(frame1);

  cv::Size s1 = frame1.size();
  cv::Size s2 = frame2.size();
  std::cout << "Size Initial Expected1 : " << s1 << std::endl;
  std::cout << "Size Initial Expected2 : " << s2 << std::endl;

  if (this->StereoVideo.GetIsCalibrated())
  {
    cv::Mat frameTemp1, frameTemp2;
    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f> > bounds = this->StereoVideo.WarpCalibration(frame1, frame2, frameTemp1, frameTemp2);

    if (this->ui->actionCropResult->isChecked())
    {
      double aspectRatioL = 1.0 * frame1.cols / frame1.rows;
      double aspectRatioR = 1.0 * frame2.cols / frame2.rows;

      // Get the rectangle inside boundaries rectangle which keep the aspect ratio
      cv::Rect LeftRect = InteriorRectangleOfWarpedImage(bounds.first, aspectRatioL);
      cv::Rect RightRect = InteriorRectangleOfWarpedImage(bounds.second, aspectRatioR);

      // Crop the images according to the rectangles
      cv::Mat croppedLeft(frameTemp1, LeftRect);
      cv::Mat croppedRight(frameTemp2, RightRect);

      // Resize the images to fit the original images sizes
      cv::resize(croppedLeft, frameTemp1, frame1.size(), 0, 0, cv::INTER_LINEAR);
      cv::resize(croppedRight, frameTemp2, frame2.size(), 0, 0, cv::INTER_LINEAR);
    }

    s1 = frameTemp1.size();
    s2 = frameTemp2.size();
  }

  std::cout << "Size Expected1 : " << s1 << std::endl;
  std::cout << "Size Expected2 : " << s2 << std::endl;
  
  this->vidWriter1.open(filenameLeft.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), this->StereoVideo.GetVideo(0)->get(CV_CAP_PROP_FPS), s1);
  this->vidWriter2.open(filenameRight.c_str(), CV_FOURCC('M', 'J', 'P', 'G'), this->StereoVideo.GetVideo(1)->get(CV_CAP_PROP_FPS), s2);

  this->StereoVideo.ResetStreamPosition();

  this->IsSaving = true;
  this->IsWriterOpen = true;
}

//---------------------------------------------------------------------------
void MainWindow::OnCalibrateColor()
{
  std::cout << "Color calibration ! " << std::endl;
  this->LeftColorCalibrator.ResetCalibrator();
  this->RighttColorCalibrator.ResetCalibrator();
  this->LeftColorCalibrator.SetWindowSize(this->colorDialog->GetWindowSise());
  this->RighttColorCalibrator.SetWindowSize(this->colorDialog->GetWindowSise());
  this->ColorCalibrationMethod = this->colorDialog->GetMethodChoice();
this->ShouldWhiteBalance = this->colorDialog->GetEnableWhiteBalance();
this->WhiteSaturation = this->colorDialog->GetWhiteSaturation();
this->IsColorCalibrated = true;
}

//---------------------------------------------------------------------------
void MainWindow::OnPlay()
{
  if (!this->PlayTimer->isActive())
  {
    if (this->StereoVideo.GetIsStereoReady())
    {
      double fps = 0.5 * (this->StereoVideo.GetFps(0) + this->StereoVideo.GetFps(1));
      double timeToWaitMs = 1 / fps * 1000;
      this->PlayTimer->start(timeToWaitMs);
    }
    else
    {
      QMessageBox::information(this, tr("Warning"), tr("Stereo Video is not ready yet, please open a left and right video"));
    }
  }
  else
  {
    this->PlayTimer->stop();
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnTimerPlayOut()
{
  if (this->StereoVideo.GetVideo(1)->read(this->RightImage) && this->StereoVideo.GetVideo(0)->read(this->LeftImage))
  {
    // Update GUI progress things
    int pos1 = this->StereoVideo.GetVideo(0)->get(CV_CAP_PROP_POS_FRAMES);
    int pos2 = this->StereoVideo.GetVideo(1)->get(CV_CAP_PROP_POS_FRAMES);
    int nbrFrame1 = this->StereoVideo.GetVideoInfo(0).NbrFrame;
    int nbrFrame2 = this->StereoVideo.GetVideoInfo(1).NbrFrame;
    double rel1 = 1.0 * pos1 / nbrFrame1;
    double rel2 = 1.0 * pos2 / nbrFrame2;
    this->Progress->setValue(100.0 * std::max(rel1, rel2));
    this->ui->progressionSlider->setValue(std::min(pos1, pos2));
    this->ui->progressionSpinBox->setValue(std::min(pos1, pos2));

    cv::Mat temp2 = this->RightImage.clone();
    cv::Mat temp1 = this->LeftImage.clone();

    cv::Mat I0 = this->LeftImage.clone();
    cv::Mat I1 = this->RightImage.clone();

    if (this->ui->actionCalibrateColor->isChecked())
    {
      if (this->ShouldWhiteBalance)
      {
        WhiteBalanceGreyAssumption(this->LeftImage, I0, this->WhiteSaturation);
        WhiteBalanceGreyAssumption(this->RightImage, I1, this->WhiteSaturation);
      }

      LeftColorCalibrator.AddImage(I0);
      RighttColorCalibrator.AddImage(I1);

      if (this->ColorCalibrationMethod == 1)
      {
        double moyTarget = RighttColorCalibrator.GetMoy();
        double sigmaTarget = RighttColorCalibrator.GetSigma();
        LeftColorCalibrator.MatchTargetCalibration(I0, temp1, moyTarget, sigmaTarget);

        temp2 = I1.clone();
      }
      else
      {
        double moyTarget = LeftColorCalibrator.GetMoy();
        double sigmaTarget = LeftColorCalibrator.GetSigma();
        RighttColorCalibrator.MatchTargetCalibration(I1, temp2, moyTarget, sigmaTarget);

        temp1 = I1.clone();
      }
    }

    if (this->StereoVideo.GetIsCalibrated())
    {
      // warp using computed homography
      std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > > bounds = this->StereoVideo.WarpCalibration(temp1, temp2, this->LeftResult, this->RightResult);

      std::cout << "Corners of left image: " << std::endl;
      for (unsigned int k = 0; k < bounds.first.size(); ++k)
      {
        std::cout << "[" << bounds.first[k].x << "," << bounds.first[k].y << "]" << std::endl;
      }
      std::cout << std::endl;

      std::cout << "Corners of right image: " << std::endl;
      for (unsigned int k = 0; k < bounds.second.size(); ++k)
      {
        std::cout << "[" << bounds.second[k].x << "," << bounds.second[k].y << "]" << std::endl;
      }
      std::cout << std::endl;

      double aspectRatioL = 1.0 * this->LeftImage.cols / this->LeftImage.rows;
      double aspectRatioR = 1.0 * this->RightImage.cols / this->RightImage.rows;

      double y1 = DiagLengthRectangleInsidePolygone(bounds.first, aspectRatioL);
      double y2 = DiagLengthRectangleInsidePolygone(bounds.second, aspectRatioR);

      std::pair<cv::Rect, cv::Rect> cropRectangle = KeepSmallerRectangle(bounds.first, bounds.second, y1, y2, aspectRatioL, aspectRatioR);

      if (this->ui->actionCropResult->isChecked())
      {
        // Crop the images according to the rectangles
        cv::Mat croppedLeft(this->LeftResult, cropRectangle.first);
        cv::Mat croppedRight(this->RightResult, cropRectangle.second);

        // Resize the images to fit the original images sizes
        cv::resize(croppedLeft, this->LeftResult, this->LeftImage.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(croppedRight, this->RightResult, this->RightImage.size(), 0, 0, cv::INTER_LINEAR);
      }
      else
      {
        cv::Rect cropRectangleLeft = CenteredRectangle(bounds.first, I0.rows, I0.cols);
        cv::Rect cropRectangleRight = CenteredRectangle(bounds.second, I1.rows, I1.cols);

        // Show the cropping area
        cv::rectangle(this->LeftResult, cropRectangle.first, cv::Scalar(0.0, 255.0, 0.0), 20);
        cv::rectangle(this->RightResult, cropRectangle.second, cv::Scalar(0.0, 255.0, 0.0), 20);

        // show the non-cropping area
        cv::rectangle(this->LeftResult, cropRectangleLeft, cv::Scalar(255.0, 0.0, 0.0), 20);
        cv::rectangle(this->RightResult, cropRectangleRight, cv::Scalar(255.0, 0.0, 0.0), 20);
      }
    }
    else
    {
      this->LeftResult = temp1;
      this->RightResult = temp2;
    }

    if (this->IsSaving && this->IsWriterOpen)
    {
      std::cout << "Size Given1 : " << this->LeftResult.size() << std::endl;
      std::cout << "Size Given2 : " << this->RightResult.size() << std::endl;
      this->vidWriter1.write(this->LeftResult);
      this->vidWriter2.write(this->RightResult);
    }

    //this->StereoVideo.UndistordImage(this->LeftImage, this->LeftResult, 0);
    //this->StereoVideo.UndistordImage(this->RightImage, this->RightResult, 1);

    // refresh the results
    this->ShowImageLeft();
    this->ShowImageRight();
    this->ShowImageLastResult();
    this->ShowImageLastResultLeft();
  }
  else
  {
    this->StereoVideo.ResetStreamPosition();
    if (this->IsSaving && this->IsWriterOpen)
    {
      this->IsSaving = false;
      this->IsWriterOpen = false;
      this->vidWriter1.release();
      this->vidWriter2.release();
    }
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnTimerCalibrationOut()
{
  double u = this->StereoVideo.GetCalibrationProgression();
  //this->ProgressBar->setValue(u);
}

//---------------------------------------------------------------------------
void MainWindow::OnSynchronizationImageBased()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video before synchronization"));
    return;
  }

  QMessageBox* msgBox = new QMessageBox(this);
  msgBox->show();
  msgBox->setAttribute(Qt::WA_DeleteOnClose);
  msgBox->setStandardButtons(QMessageBox::Ok);
  msgBox->setText(tr("Please wait while time synchronization is processing. This operation can take severals seconds"));
  msgBox->setWindowTitle("Time Synchronization...");
  msgBox->setModal(true);
  msgBox->update();

  // disable the mainwindow while the computation is running
  this->setEnabled(false);

  int maxAllocation = this->syncDialog->GetSequenceLength() * this->StereoVideo.GetFps(0); // two minute of recording at 30 FPS
  int size1, size2;
  double* data1 = new double[maxAllocation];
  double* data2 = new double[maxAllocation];

  std::cout << "Hello ! " << std::endl;
  this->StereoVideo.ComputeSyncSignal(data1, data2, &size1, &size2, maxAllocation, this->syncDialog->GetStartPoint());
  std::cout << "Bye ! " << std::endl;

  std::cout << "size1 : " << size1 << std::endl;
  std::cout << "size2 : " << size2 << std::endl;

  double maxSample = std::min(size1, size2);
  double sample_rate = 30;
  double duration = (1.0 * maxSample) / sample_rate;

  std::pair<double, double> u1 = TimeSynchronization(data1, data2, maxSample, maxSample, sample_rate);
  std::pair<double, double> u2 = TimeSynchronization(data2, data1, maxSample, maxSample, sample_rate);

  std::cout << "timesync 1 : " << u1.first << std::endl;
  std::cout << "timesync 2 : " << u2.first << std::endl;

  if (u1.first >= (duration / 2) && u2.first <= (duration / 2))
  {
    std::cout << "Signal left is in advance and right is late" << std::endl;
    double finalTimeSync = u2.first + 1.0 / this->StereoVideo.GetFps(0);
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(0.0, finalTimeSync);
    this->TimeOffset = finalTimeSync;
  }
  else if (u2.first >= (duration / 2) && u1.first <= (duration / 2))
  {
    std::cout << "Signal right is in advance and left is late" << std::endl;
    double finalTimeSync = u1.first + 1.0 / this->StereoVideo.GetFps(0);
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(finalTimeSync, 0.0);
    this->TimeOffset = -finalTimeSync;
  }
  else
  {
    std::cout << "Weird case" << std::endl;
    this->TimeOffset = 0;
  }

  // refresh the results
  this->ShowImageLeft();
  this->ShowImageRight();
  this->ShowImageLastResult();
  this->ShowImageLastResultLeft();

  delete[] data1;
  delete[] data2;

  // Enable the mainwindow while the computation is over
  this->setEnabled(true);

  msgBox->close();
}

//---------------------------------------------------------------------------
void MainWindow::OnSynchronization()
{
  /*if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video before synchronization"));
    return;
  }

  int sample_rate = 44100;
  double* data1;
  double* data2;
  int size1, size2;

  QMessageBox* msgBox = new QMessageBox(this);
  msgBox->show();
  msgBox->setAttribute(Qt::WA_DeleteOnClose);
  msgBox->setStandardButtons(QMessageBox::Ok);
  msgBox->setText(tr("Please wait while time synchronization is processing. This operation can take severals seconds"));
  msgBox->setWindowTitle("Time Synchronization...");
  msgBox->setModal(true);
  msgBox->update();

  // disable the mainwindow while the computation is running
  this->setEnabled(false);

  int maxAllocation = 100000000;

  data1 = new double[maxAllocation];
  data2 = new double[maxAllocation];

  // first, extract the audio using ffmped
  std::string command1, command2;
  command1 = "./ffmpeg.exe -i " + this->FilenameLeft + " -vn -acodec copy left.aac";
  command2 = "./ffmpeg.exe -i " + this->FilenameRight + " -vn -acodec copy right.aac";
  system("del left.aac");
  system("del right.aac");
  system(command1.c_str());
  system(command2.c_str());
  

  AudioExtract("left.aac", sample_rate, data1, &size1, maxAllocation);
  AudioExtract("right.aac", sample_rate, data2, &size2, maxAllocation);

  std::cout << "Data1 length : " << size1 << std::endl;
  std::cout << "Data2 length : " << size2 << std::endl;

  int minSize = std::min(size1, size2);
  int maxSample = minSize / 10;
  double duration = (1.0 * maxSample) / sample_rate;
  std::cout << "maxSample : " << maxSample << std::endl;
  std::cout << "Analysing by sequences of : " << duration << std::endl;
  // cut the all sequence in small pieces
  int nbrSeq = minSize / maxSample;
  double* sound1 = new double[maxSample];
  double* sound2 = new double[maxSample];

  std::vector<double> timeSync1;
  std::vector<double> timeSync2;
  std::vector<double> score1;
  std::vector<double> score2;
  double totalScore1 = 0;
  double totalScore2 = 0;

  for (int k = 0; k < nbrSeq; ++k)
  {
    for (int i = 0; i < maxSample; ++i)
    {
      sound1[i] = data1[k * maxSample + i];
      sound2[i] = data2[k * maxSample + i];
    }

    std::pair<double, double> u = TimeSynchronization(sound1, sound2, maxSample, maxSample, sample_rate);
    timeSync1.push_back(u.first);
    score1.push_back(u.second);
    totalScore1 += u.second;

    u = TimeSynchronization(sound2, sound1, maxSample, maxSample, sample_rate);
    timeSync2.push_back(u.first);
    score2.push_back(u.second);
    totalScore2 += u.second;
  }

  //std::sort(timeSync.begin(), timeSync.end());

  double moy1 = 0;
  double moy2 = 0;
  for (int k = 0; k < timeSync1.size(); ++k)
  {
    moy1 = moy1 + timeSync1[k] * score1[k] / totalScore1;
    moy2 = moy2 + timeSync2[k] * score2[k] / totalScore2;
  }

  double var1 = 0;
  double var2 = 0;
  for (int k = 0; k < timeSync1.size(); ++k)
  {
    var1 = var1 + std::pow(timeSync1[k] - moy1, 2) * score1[k] / totalScore1;
    var2 = var2 + std::pow(timeSync2[k] - moy2, 2) * score2[k] / totalScore2;
  }
  var1 = std::sqrt(var1);
  var2 = std::sqrt(var2);

  std::cout << "moy1 : " << moy1 << std::endl;
  std::cout << "var1 : " << var1 << std::endl;
  std::cout << std::endl;
  std::cout << "moy2 : " << moy2 << std::endl;
  std::cout << "var2 : " << var2 << std::endl;

  double totalVar = (var1 + var2) / 2;

  // bad results, let's take the median
  if (totalVar > 0.10)
  {
    std::sort(timeSync1.begin(), timeSync1.end());
    std::sort(timeSync2.begin(), timeSync2.end());
    int halfSize = timeSync1.size() / 2;
    if (timeSync1.size() % 2 == 0)
    {
      moy1 = (timeSync1[halfSize] + timeSync1[halfSize - 1]) / 2;
      moy2 = (timeSync2[halfSize] + timeSync2[halfSize - 1]) / 2;
    }
    else
    {
      moy1 = timeSync1[halfSize];
      moy2 = timeSync2[halfSize];
    }
  }

  if (moy1 >= (duration / 2) && moy2 <= (duration / 2))
  {
    std::cout << "Signal left is in advance and right is late" << std::endl;
    double finalTimeSync = moy2;
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(0.0, finalTimeSync);
  }
  else if (moy2 >= (duration / 2) && moy1 <= (duration / 2))
  {
    std::cout << "Signal right is in advance and left is late" << std::endl;
    double finalTimeSync = moy1;
    std::cout << "finalTimeSync : " << finalTimeSync << std::endl;
    this->StereoVideo.SetSyncs(finalTimeSync, 0.0);
  }
  else
  {
    std::cout << "Weird case" << std::endl;
  }

  // refresh the results
  this->ShowImageLeft();
  this->ShowImageRight();
  this->ShowImageLastResult();
  this->ShowImageLastResultLeft();

  delete[] data1;
  delete[] data2;

  delete[] sound1;
  delete[] sound2;

  system("del left.aac");
  system("del right.aac");

  // Enable the mainwindow while the computation is over
  this->setEnabled(true);

  msgBox->close();*/
}

//---------------------------------------------------------------------------
void MainWindow::OnLaunchDistanceInfoDialog()
{
  if (!this->StereoVideo.GetIsCalibrated())
  {
    QMessageBox::information(this, tr("Warning"), tr("Calibration has not been done, please calibrate the camera or import a calibration"));
    return;
  }

  this->distanceInfoDialog->show();
}

//---------------------------------------------------------------------------
void MainWindow::SetProgressionParameters()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    std::cout << "Stereo not ready !" << std::endl;
    return;
  }
  
  int maxFrame = std::min(this->StereoVideo.GetVideoInfo(0).NbrFrame, this->StereoVideo.GetVideoInfo(1).NbrFrame);
  this->ui->progressionSlider->setMinimum(0);
  this->ui->progressionSlider->setMaximum(maxFrame);

  this->ui->progressionSpinBox->setMinimum(0);
  this->ui->progressionSpinBox->setMaximum(maxFrame);
}

//---------------------------------------------------------------------------
void MainWindow::OnSliderChanged()
{
  // Set the value of the spin box
  this->ui->progressionSpinBox->setValue(this->ui->progressionSlider->value());

  // Set the pos of the StereoVideo
  this->StereoVideo.SetStreamPos(this->ui->progressionSlider->value());
}

//---------------------------------------------------------------------------
void MainWindow::OnSpinBoxChanged()
{

}

//---------------------------------------------------------------------------
void MainWindow::OnDistanceInfo()
{
  if (this->distanceInfoDialog->GetMethodChoice() == 1)
  {
    this->StereoVideo.SetCameraDistance(this->distanceInfoDialog->GetDistanceValue());
  }
  else
  {
    cv::Mat frame1, frame2;
    this->StereoVideo.GetVideo(0)->read(frame1);
    this->StereoVideo.GetVideo(1)->read(frame2);

    // We will get two 3D points reconstructed from a match
    // between the left and the right images of the stereo
    ManualMatcher matcher(frame1, frame2);
    matcher.LaunchMatchingPipeline(2);
    std::vector<std::pair<cv::Point2f, cv::Point2f> > manualyMatched = matcher.GetMatchedPoints();

    this->StereoVideo.RecoveringCamerasDistance(this->distanceInfoDialog->GetDistanceValue(), manualyMatched);
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnLaunchSimulationDialog()
{
  if (!this->StereoVideo.GetIsCalibrated())
  {
    QMessageBox::information(this, tr("Warning"), tr("Calibration has not been done, please calibrate the camera or import a calibration"));
    return;
  }
  this->simulateDialog->show();
}

//---------------------------------------------------------------------------
void MainWindow::OnSimulateAcquisition()
{
  if (this->simulateDialog->GetMethod() == 1)
  {
    // Get parameters of left image
    double lPhi = this->simulateDialog->GetLeftPhi();
    double lTheta = this->simulateDialog->GetLeftTheta();
    double lPsi = this->simulateDialog->GetLeftPsi();

    // Get parameters of right image
    double rPhi = this->simulateDialog->GetRightPhi();
    double rTheta = this->simulateDialog->GetRightTheta();
    double rPsi = this->simulateDialog->GetRightPsi();

    double leftParameters[3] = { lPhi, lTheta, lPsi };
    double rightParameters[3] = { rPhi, rTheta, rPsi };

    this->StereoVideo.SimulateNewPose(leftParameters, rightParameters);
  }
  else
  {
    double phi = this->simulateDialog->GetRelPhi();
    double theta = this->simulateDialog->GetRelTheta();
    double psi = this->simulateDialog->GetRelPsi();

    double relParameters[3] = { phi, theta, psi };

    this->StereoVideo.SimulateNewPose(relParameters);
  }
}

//---------------------------------------------------------------------------
void MainWindow::OnShouldAlignBaseline()
{
  this->StereoVideo.SetShouldAlignBaseline(this->ui->actionAlignBaseline->isChecked());
  this->OnTimerPlayOut();
}

//---------------------------------------------------------------------------
void MainWindow::OnLaunchDialogForColorCalibration()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("Stereo video is not ready, please open left and right video first"));
    return;
  }

  if (this->IsColorCalibrated)
  {
    this->IsColorCalibrated = false;
    return;
  }
  
  this->colorDialog->show();
}

//---------------------------------------------------------------------------
void MainWindow::OnLoadCalibrationFromSimulation()
{
  if (!this->StereoVideo.GetIsStereoReady())
  {
    QMessageBox::information(this, tr("Warning"), tr("No video launched, please load left and right video before"));
    return;
  }

  QString filename = QFileDialog::getOpenFileName(this, "Select calibration to match", QDir::homePath(), NULL);
  if (filename.isNull() || filename.isEmpty())
  {
    std::cout << "No file selected" << std::endl;
    return;
  }

  std::string filename_s = filename.toStdString();
  std::ifstream file;
  file.open(filename_s);
  file.precision(20);
  if (!file.is_open())
  {
    std::cout << "Error opening file : " << filename_s << std::endl;
  }

  // double Mat (6 = double)
  cv::Mat currF = cv::Mat::zeros(3, 3, 6);
  std::vector<cv::KeyPoint> leftK;
  std::vector<cv::KeyPoint> rightK;
  Eigen::Matrix<double, 3, 3> Fund;

  // read the file
  std::string line;
  int tokenEncountered = 0;
  int matrixLine = 0;
  while (std::getline(file, line))
  {
    if (line[0] == '!')
    {
      tokenEncountered++;
      continue;
    }

    // Fundamental matrix data
    if (tokenEncountered == 1)
    {
      std::istringstream iss(line);
      double a, b, c;
      iss >> a >> b >> c;
      currF.at<double>(matrixLine, 0) = a;
      currF.at<double>(matrixLine, 1) = b;
      currF.at<double>(matrixLine, 2) = c;
      Fund(matrixLine, 0) = a;
      Fund(matrixLine, 1) = b;
      Fund(matrixLine, 2) = c;
      matrixLine++;
    }
    // Pair matches data
    else if (tokenEncountered == 3)
    {
      std::istringstream iss(line);
      double xl, yl, xr, yr;
      iss >> xl >> yl >> xr >> yr;
      cv::KeyPoint L, R;
      L.pt.x = xl;
      L.pt.y = yl;
      R.pt.x = xr;
      R.pt.y = yr;
      leftK.push_back(L);
      rightK.push_back(R);
    }
    else
    {
      std::cout << "Oups ! Error while parsing calib data" << std::endl;
    }
  }

  // Prior intrinsec parameters
  Eigen::Matrix<double, 3, 3> Kprior1;
  Kprior1 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  double phi, psi;
  double theta = 0.95993108859; // 55 degre
  double w = static_cast<double>(this->LeftImage.cols);
  double h = static_cast<double>(this->LeftImage.rows);
  double f = w / (2 * std::tan(theta / 2.0));
  Kprior1(0, 0) = f;
  Kprior1(1, 1) = f;
  Kprior1(2, 2) = 1;
  Kprior1(0, 2) = w / 2.0;
  Kprior1(1, 2) = h / 2.0;
  // K2
  Eigen::Matrix<double, 3, 3> Kprior2;
  Kprior2 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  w = static_cast<double>(this->RightImage.cols);
  h = static_cast<double>(this->RightImage.rows);
  f = w / (2 * std::tan(theta / 2.0));
  Kprior2(0, 0) = f;
  Kprior2(1, 1) = f;
  Kprior2(2, 2) = 1;
  Kprior2(0, 2) = w / 2.0;
  Kprior2(1, 2) = h / 2.0;

  Eigen::Matrix<double, 3, 3> R = GetPositionInformationFromFund(Fund, Kprior1, Kprior2);
  this->StereoVideo.SimulateNewPose(R);
  this->simulateDialog->hide();
}