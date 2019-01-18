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
/**
 * @class   StereoVideoManager
 * @brief
 *
 *
*/

// STD
#include <ctime>

// LOCAL
#include "StereoVideoManager.h"

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

//---------------------------------------------------------------------------
VideoInfo::VideoInfo()
{
  this->CurrentPos = -1;
  this->Fps = -1;
  this->Height = 0;
  this->NbrFrame = 0;
  this->RelativeCurrentPos = -1;
  this->Width = 0;
  this->IndexFrame = 0;
}

//---------------------------------------------------------------------------
StereoVideoManager::StereoVideoManager()
{
  this->VideoReady[0] = false;
  this->VideoReady[1] = false;
  this->IsStereoReady = false;
  this->sync1 = 0;
  this->sync2 = 0;
  this->shouldExportResults = false;
  this->IsCalibrated = false;
  this->ComputationProgression = new QProgressBar();

  // standard parameters from GoPro
  // fish-eyes cameras.
  this->DistortionCoeff.resize(5, 0);
  this->DistortionCoeff[0] = -0.04133383;
  this->DistortionCoeff[1] = 0.13164492;
  this->DistortionCoeff[2] = -0.07380571;
  this->DistortionCoeff[3] = 0.01821631;

  // 120° Field of View, most
  // commonly used field of view
  this->FOV = 120;

  // distortions mapping
  this->distMappingXYLeft.resize(2);
  this->distMappingXYRight.resize(2);

  // do not align by default
  this->ShouldAlignBaseline = false;
}

//---------------------------------------------------------------------------
void StereoVideoManager::OpenVideo1(std::string filename)
{
  this->Video1.open(filename);

  if (!this->Video1.isOpened())
  {
    std::cout << "first video : Cannot open video!\n" << std::endl;
  }
  else
  {
    this->VideoReady[0] = true;
  }

  this->IsStereoReady = this->VideoReady[0] && this->VideoReady[1];

  if (this->IsStereoReady)
  {
    this->UpdateInfo();
    this->UpdateIntrinsicParams();
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::OpenVideo2(std::string filename)
{
  this->Video2.open(filename);

  if (!this->Video2.isOpened())
  {
    std::cout << "second video : Cannot open video!\n" << std::endl;
  }
  else
  {
    this->VideoReady[1] = true;
  }

  this->IsStereoReady = this->VideoReady[0] && this->VideoReady[1];

  if (this->IsStereoReady)
  {
    this->UpdateInfo();
    this->UpdateIntrinsicParams();
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::OpenVideo(std::string filename1, std::string filename2)
{
  this->Video1.open(filename1);
  this->Video2.open(filename2);

  if (!this->Video1.isOpened())
  {
    std::cout << "first video : Cannot open video!\n" << std::endl;
  }
  else
  {
    this->VideoReady[0] = true;
  }

  if (!this->Video2.isOpened())
  {
    std::cout << "second video : Cannot open video!\n" << std::endl;
  }
  else
  {
    this->VideoReady[1] = true;
  }

  this->IsStereoReady = this->VideoReady[0] && this->VideoReady[1];

  if (this->IsStereoReady)
  {
    this->UpdateInfo();
    this->UpdateIntrinsicParams();
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::UpdateInfo()
{
  if (this->IsStereoReady)
  {
    this->Info[0].CurrentPos = this->Video1.get(CV_CAP_PROP_POS_MSEC);
    this->Info[0].RelativeCurrentPos = this->Video1.get(CV_CAP_PROP_POS_AVI_RATIO);
    this->Info[0].Fps = this->Video1.get(CV_CAP_PROP_FPS);
    this->Info[0].Height = (unsigned int)this->Video1.get(CV_CAP_PROP_FRAME_HEIGHT);
    this->Info[0].Width = (unsigned int)this->Video1.get(CV_CAP_PROP_FRAME_WIDTH);
    this->Info[0].NbrFrame = (unsigned int)this->Video1.get(CV_CAP_PROP_FRAME_COUNT);
    this->Info[0].IndexFrame = (unsigned int)this->Video1.get(CV_CAP_PROP_POS_FRAMES);

    this->Info[1].CurrentPos = this->Video2.get(CV_CAP_PROP_POS_MSEC);
    this->Info[1].RelativeCurrentPos = this->Video2.get(CV_CAP_PROP_POS_AVI_RATIO);
    this->Info[1].Fps = this->Video2.get(CV_CAP_PROP_FPS);
    this->Info[1].Height = (unsigned int)this->Video2.get(CV_CAP_PROP_FRAME_HEIGHT);
    this->Info[1].Width = (unsigned int)this->Video2.get(CV_CAP_PROP_FRAME_WIDTH);
    this->Info[1].NbrFrame = (unsigned int)this->Video2.get(CV_CAP_PROP_FRAME_COUNT);
    this->Info[1].IndexFrame = (unsigned int)this->Video2.get(CV_CAP_PROP_POS_FRAMES);
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::ShowInfo()
{
  std::cout << "Video1 information : " << std::endl;
  std::cout << "Current position : " << this->Info[0].CurrentPos << std::endl;
  std::cout << "Relative current position : " << this->Info[0].RelativeCurrentPos << std::endl;
  std::cout << "FPS : " << this->Info[0].Fps << std::endl;
  std::cout << "Height : " << this->Info[0].Height << std::endl;
  std::cout << "Width : " << this->Info[0].Width << std::endl;
  std::cout << "NbrFrame : " << this->Info[0].NbrFrame << std::endl;
  std::cout << "Next Frame : " << this->Info[0].IndexFrame << std::endl;
  std::cout << std::endl;

  std::cout << "Video2 information : " << std::endl;
  std::cout << "Current position : " << this->Info[1].CurrentPos << std::endl;
  std::cout << "Relative current position : " << this->Info[1].RelativeCurrentPos << std::endl;
  std::cout << "FPS : " << this->Info[1].Fps << std::endl;
  std::cout << "Height : " << this->Info[1].Height << std::endl;
  std::cout << "Width : " << this->Info[1].Width << std::endl;
  std::cout << "NbrFrame : " << this->Info[1].NbrFrame << std::endl;
  std::cout << "Index Frame : " << this->Info[1].IndexFrame << std::endl;
  std::cout << std::endl;
}

//---------------------------------------------------------------------------
cv::VideoCapture* StereoVideoManager::GetVideo(int idxVideo)
{
  if(idxVideo == 0)
  {
    return &this->Video1;
  }
  else
  {
    return &this->Video2;
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetProgressBar(QProgressBar* input)
{
  if (input != NULL)
  {
    delete this->ComputationProgression;
  }

  this->ComputationProgression = input;
}

//---------------------------------------------------------------------------
bool StereoVideoManager::GetIsStereoReady()
{
  return this->IsStereoReady;
}

//---------------------------------------------------------------------------
double StereoVideoManager::GetFps(int indexCamera)
{
  return this->Info[indexCamera].Fps;
}

//---------------------------------------------------------------------------
Calibrator* StereoVideoManager::GetCalibrator()
{
  return &this->Calibr;
}

//---------------------------------------------------------------------------
VideoInfo StereoVideoManager::GetVideoInfo(int idxVideo)
{
  return this->Info[idxVideo];
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetUsePrior(bool value)
{
  this->Calibr.SetUsePrior(value);
  this->UsePrior = value;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetCalibratorParameters(CalibratorParams value)
{
  this->paramC = value;
  this->Calibr.params = value;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SolidCameraCalibration(int nFrame, CalibratorParams parameters)
{
  int maxFrame = (int)(std::min((double)(this->Info[0].NbrFrame) - this->sync1, (double)(this->Info[1].NbrFrame) - this->sync2));
  int step = static_cast<int>(std::floor(static_cast<double>(maxFrame) / static_cast<double>(nFrame)));

  std::vector<cv::Point2f> leftAccumulatedMatches;
  std::vector<cv::Point2f> rightAccumulatedMatches;

  std::vector<cv::KeyPoint> leftAccumulatedMatchesK;
  std::vector<cv::KeyPoint> rightAccumulatedMatchesK;

  this->CalibrationProgression = 0;

  for (int k = 0; k < maxFrame; k = k + step)
  {
    int index1 = k + static_cast<int>(this->sync1);
    int index2 = k + static_cast<int>(this->sync2);

    std::cout << "index: " << k << std::endl;
    std::cout << "reading frames: [" << index1 << "; " << index2 << "]" << std::endl;

    this->Video1.set(CV_CAP_PROP_POS_FRAMES, index1);
    this->Video2.set(CV_CAP_PROP_POS_FRAMES, index2);

    cv::Mat frame1, frame2;
    this->Video1.read(frame1);
    this->Video2.read(frame2);

    if (frame1.cols == 0 || frame1.rows == 0 || frame2.cols == 0 || frame2.rows == 0)
    {
      std::cout << "one image is badly defined, skip it" << std::endl;
      continue;
    }

    this->Calibr = Calibrator();
    this->Calibr.SetImage(frame1, 0);
    this->Calibr.SetImage(frame2, 1);
    this->Calibr.params = parameters;
    this->Calibr.SetUsePrior(this->UsePrior);

    this->Calibr.Calibrate();

    std::vector<cv::KeyPoint> leftCurrentMatches = this->Calibr.GetMatchedKeyPoint(0);
    std::vector<cv::KeyPoint> rightCurrentMatches = this->Calibr.GetMatchedKeyPoint(1);

    if (leftCurrentMatches.size() == 0 || rightCurrentMatches.size() == 0)
    {
      std::cout << "One image has 0 keypoints; skip it" << std::endl;
      continue;
    }

    if (this->shouldExportResults)
    {
      std::vector<cv::Mat> results = this->Calibr.GetCalibrationResults();
      std::string exportFi = this->exportFilename + std::to_string(k) + ".png";
      cv::imwrite(exportFi, results[3]);
    }


    for (int i = 0; i < leftCurrentMatches.size(); ++i)
    {
      leftAccumulatedMatches.push_back(leftCurrentMatches[i].pt);
      rightAccumulatedMatches.push_back(rightCurrentMatches[i].pt);

      leftAccumulatedMatchesK.push_back(leftCurrentMatches[i]);
      rightAccumulatedMatchesK.push_back(rightCurrentMatches[i]);
    }
    std::cout << "Added : " << leftCurrentMatches.size() << " points, total : " << leftAccumulatedMatches.size() << " points" << std::endl;

    this->CalibrationProgression = static_cast<double>(k) / static_cast<double>(maxFrame) * 100;
    this->ComputationProgression->setValue(this->CalibrationProgression);
  }

  std::cout << "Computing fundamental matrix using : " << leftAccumulatedMatches.size() << " points" << std::endl;
  // Compute the RANSAC to find the fundamental matrix
  cv::Mat MaskF; // Mask of ransac computing
  this->F = cv::findFundamentalMat(leftAccumulatedMatches, rightAccumulatedMatches, MaskF, cv::FM_RANSAC, 2, 0.99);

  int inlier = 0;
  for (int k = 0; k < MaskF.rows; ++k)
  {
    int value = static_cast<int>(MaskF.at<uchar>(k, 0));
    if (value == 1)
    {
      inlier++;
      this->leftMatched.push_back(leftAccumulatedMatchesK[k]);
      this->rightMatched.push_back(rightAccumulatedMatchesK[k]);
    }
  }

  std::cout << "TOTAL INLIERS : " << inlier << std::endl;

  this->Calibr.SetF(this->F, this->leftMatched, this->rightMatched);

  /*if (this->UsePrior)
  {
    this->F = this->Calibr.GetF();
  }*/

  std::cout << "Fundamental matrix value : " << std::endl;
  std::cout << this->F << std::endl;

  // Now compute essential matrix and pose using prior values
  this->ComputeEssential();

  this->Video1.set(CV_CAP_PROP_POS_FRAMES, static_cast<int>(this->sync1));
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, static_cast<int>(this->sync2));
}

void StereoVideoManager::SetSyncs(double s1, double s2)
{
  this->sync1 = static_cast<int>(this->Info[0].Fps * s1);
  this->sync2 = static_cast<int>(this->Info[1].Fps * s2);

  this->Video1.set(CV_CAP_PROP_POS_FRAMES, static_cast<int>(this->sync1));
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, static_cast<int>(this->sync2));
}

//---------------------------------------------------------------------------
std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > > StereoVideoManager::WarpCalibration(cv::Mat& src1, cv::Mat& src2, cv::Mat& dst1, cv::Mat& dst2)
{
  // Initialize images
  dst1 = cv::Mat(src1.size(), src1.type());
  dst2 = cv::Mat(src2.size(), src2.type());

  std::vector< cv::Point2f > BoundsLeft = homography_warp(src1, this->Hcv1, dst1);
  std::vector< cv::Point2f > BoundsRight = homography_warp(src2, this->Hcv2, dst2);

  return std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > >(BoundsLeft, BoundsRight);
}

//---------------------------------------------------------------------------
void StereoVideoManager::ResetStreamPosition()
{
  // Set the videos pointers to 0
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, this->sync1);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, this->sync2);
}

//---------------------------------------------------------------------------
void StereoVideoManager::ExportVideoRectified(std::string filename, int idxCamera)
{
  // Set the videos pointers to 0
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, this->sync1);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, this->sync2);

  cv::VideoCapture* video;
  if (idxCamera == 0)
  {
    video = &this->Video1;
  }
  else
  {
    video = &this->Video2;
  }

  cv::Mat frame;
  this->Video1.read(frame);
  this->Video2.read(frame);

  // Rectified the images
  cv::Mat H1, H2, H3;
  std::cout << "left size : " << this->leftMatched.size() << std::endl;
  std::cout << "right size : " << this->rightMatched.size() << std::endl;
  std::cout << "F : " << this->F << std::endl;
  cv::stereoRectifyUncalibrated(this->leftMatched, this->rightMatched, this->F,
                                frame.size(), H1, H2, 5);
  std::cout << "End" << std::endl;
  std::cout.flush();

  /*cv::Mat F_t = cv::Mat(this->F.cols, this->F.rows, CV_32F);
  cv::transpose(this->F, F_t);
  cv::stereoRectifyUncalibrated(this->leftMatched, this->rightMatched, F_t,
                                frame.size(), H2, H3, 5);*/


  cv::Mat H;
  if (idxCamera == 0)
  {
    H = H1;
  }
  else
  {
    H = H2;
  }

  std::cout << "H1 : " << H1 << std::endl;
  std::cout << "H2 : " << H2 << std::endl;
  std::cout << "H : " << H << std::endl;

  cv::Size s;
  {
    cv::Mat rectified(frame.size(), frame.type());
    homography_warp(frame, H, rectified);
    s = cv::Size(rectified.cols, rectified.rows);
  }

  cv::VideoWriter vidWriter;
  vidWriter.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), video->get(CV_CAP_PROP_FPS), s, true);

  if (!vidWriter.isOpened())
  {
    std::cout << "Could not open the output video for write: " << filename << std::endl;
    return;
  }

  cv::Mat frame1, frame2;
  this->Video1.read(frame1);
  this->Video2.read(frame2);
  cv::Mat rectified1(frame1.size(), frame1.type());
  cv::Mat rectified2(frame2.size(), frame2.type());
  homography_warp(frame1, H1, rectified1);
  homography_warp(frame2, H2, rectified2);

  for(int x = 0; x < rectified1.rows; x = x + 100)
  {
    double t = static_cast<double>(x) / static_cast<double>(rectified1.rows);
    int r = static_cast<int>(255 * t);
    int g = static_cast<int>(255 - 255 * t);
    int b = static_cast<int>(125 + (255 - 125) * t);
    for (int j = 0; j < rectified1.cols; ++j)
    {
      // get pixel
      cv::Vec3b color = rectified1.at<cv::Vec3b>(cv::Point(j,x));
      color[0] = r;
      color[1] = g;
      color[2] = b;
      // set pixel
      rectified1.at<cv::Vec3b>(cv::Point(j,x)) = color;
    }
    for (int j = 0; j < rectified2.cols; ++j)
    {
      // get pixel
      cv::Vec3b color = rectified2.at<cv::Vec3b>(cv::Point(j,x));
      color[0] = r;
      color[1] = g;
      color[2] = b;
      // set pixel
      rectified2.at<cv::Vec3b>(cv::Point(j,x)) = color;
    }
  }

  for(int x = 0; x < frame1.rows; x = x + 100)
  {
    double t = static_cast<double>(x) / static_cast<double>(frame1.rows);
    int r = static_cast<int>(255 * t);
    int g = static_cast<int>(255 - 255 * t);
    int b = static_cast<int>(125 + (255 - 125) * t);
    for (int j = 0; j < frame1.cols; ++j)
    {
      // get pixel
      cv::Vec3b color = frame1.at<cv::Vec3b>(cv::Point(j,x));
      color[0] = r;
      color[1] = g;
      color[2] = b;
// set pixel
frame1.at<cv::Vec3b>(cv::Point(j, x)) = color;
    }
    for (int j = 0; j < frame2.cols; ++j)
    {
      // get pixel
      cv::Vec3b color = frame2.at<cv::Vec3b>(cv::Point(j, x));
      color[0] = r;
      color[1] = g;
      color[2] = b;
      // set pixel
      frame2.at<cv::Vec3b>(cv::Point(j, x)) = color;
    }
  }

  /*DisplayImage(rectified1, "rectified1");
  DisplayImage(rectified2, "rectified2");
  DisplayImage(frame1, "frame1");
  DisplayImage(frame2, "frame2");*/

  while (video->read(frame))
  {
    cv::Mat rectified(frame.size(), frame.type());
    homography_warp(frame, H, rectified);

    /*cv::namedWindow("rectified", cv::WINDOW_NORMAL);
    cv::imshow("rectified", rectified);
    cv::waitKey(1);*/

    vidWriter.write(rectified);
  }

  // Set the videos pointers to 0
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, this->sync1);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, this->sync2);

  return;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetF(cv::Mat Farg)
{
  this->F = Farg;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetLeftMatched(std::vector<cv::KeyPoint> K)
{
  this->leftMatched = K;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetRightMatched(std::vector<cv::KeyPoint> K)
{
  this->rightMatched = K;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetF(cv::Mat Farg, std::vector<cv::KeyPoint> K1, std::vector<cv::KeyPoint> K2)
{
  this->F = Farg;
  this->leftMatched = K1;
  this->rightMatched = K2;

  this->Calibr.SetF(this->F, this->leftMatched, this->rightMatched);

  this->ComputeEssential();

  this->IsCalibrated = true;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetShouldExport(bool should, std::string filename)
{
  this->shouldExportResults = should;
  this->exportFilename = filename;
}

//---------------------------------------------------------------------------
void StereoVideoManager::UndistordImage(cv::Mat Input, cv::Mat& Output, unsigned int camIndex)
{
  if (camIndex == 0)
  {
    std::cout << "size of Input is : " << std::endl << Input.size() << std::endl;

    cv::remap(Input, Output, this->distMappingXYLeft[0], this->distMappingXYLeft[1], CV_INTER_LINEAR);
    std::cout << "size of Output is : " << std::endl << Output.size() << std::endl;
  }
  else
  {
    cv::remap(Input, Output, this->distMappingXYRight[0], this->distMappingXYRight[1], CV_INTER_LINEAR);
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::UpdateIntrinsicParams()
{
  // dimensions of the image
  double w = static_cast<double>(this->Info[0].Width);
  double h = static_cast<double>(this->Info[0].Height);
  this->FOV = 50;
  // focal depth in pixel
  double f = w / (2 * std::tan(this->FOV / 180.0 * 3.14159265359 / 2.0));

  // prior intrinsic parameters of left image
  Eigen::Matrix<double, 3, 3> Kprior = Eigen::Matrix<double, 3, 3>::Zero();
  Kprior(0, 0) = f;
  Kprior(1, 1) = f;
  Kprior(2, 2) = 1;
  Kprior(0, 2) = w / 2.0;
  Kprior(1, 2) = h / 2.0;

  this->K1 = Kprior;
  this->K2 = Kprior;

  // Fill the intrinsic parameters matrix
  cv::Mat KLeft(cv::Size(3, 3), 6);
  cv::Mat KRight(cv::Size(3, 3), 6);
  for (int i = 0; i < KLeft.rows; ++i)
  {
    for (int j = 0; j < KLeft.cols; ++j)
    {
      KLeft.at<double>(i, j) = this->K1(i, j);
      KRight.at<double>(i, j) = this->K2(i, j);
    }
  }

  // Fill the distortion coeffs
  cv::Mat distCoeff(cv::Size(1, 4), 6);
  for (unsigned int k = 0; k < this->DistortionCoeff.size(); ++k)
  {
    distCoeff.at<double>(0, k) = 0.0;//this->DistortionCoeff[k] / 100.0;
  }

  // Fill the distortion coeffs
 

  std::cout << "Init distortion mapping using D: " << distCoeff << std::endl;

  int outWidth = this->Info[0].Width;
  int outHeight = this->Info[0].Height;
  cv::Size oldsz1 = cv::Size(this->Info[0].Width, this->Info[0].Height);
  cv::Size oldsz2 = cv::Size(this->Info[1].Width, this->Info[1].Height);
  double scale = 1;
  cv::Size sz1 = cv::Size(scale * outWidth, scale * outHeight);
  cv::Size sz2 = cv::Size(scale * outWidth, scale * outHeight);

  //this->K1New = cv::getOptimalNewCameraMatrix(KLeft, distCoeff2, oldsz1, 1.0, sz1);
  //this->K2New = cv::getOptimalNewCameraMatrix(KRight, distCoeff2, oldsz2, 1.0, sz2);
  
  this->K1New = KLeft.clone();
  this->K2New = KRight.clone();

  this->K1New.at<double>(2 ,2) = 1.0/scale;

  std::cout << "K1 was: " << std::endl << KLeft << std::endl;
  std::cout << "K1 is : " << std::endl << this->K1New << std::endl;
  std::cout << "oldsz1 is : " << std::endl << oldsz1 << std::endl;
  std::cout << "sz1 is : " << std::endl << sz1 << std::endl;

  std::cout << "K2 was: " << std::endl << KRight << std::endl;
  std::cout << "K2 is : " << std::endl << this->K2New << std::endl;
  std::cout << "oldsz2 is : " << std::endl << oldsz2 << std::endl;
  std::cout << "sz2 is : " << std::endl << sz2 << std::endl;

  // Create the distortion mappings
  cv::Mat R = cv::Mat::eye(cv::Size(3, 3), 6);
  std::cout << "R is : " << std::endl << R << std::endl;
  cv::fisheye::initUndistortRectifyMap(KLeft, distCoeff, R, KLeft, sz1, CV_32FC1, this->distMappingXYLeft[0], this->distMappingXYLeft[1]);
  cv::fisheye::initUndistortRectifyMap(KRight, distCoeff, R, this->K2New, sz2, CV_32FC1, this->distMappingXYRight[0], this->distMappingXYRight[1]);

  for (unsigned int x = 0; x < sz1.width; x += 100)
  {
    for (unsigned int y = 0; y < sz1.height; y += 100)
    {
      std::cout << " ("<< x << "," << y << ") taken from ("<< this->distMappingXYLeft[0].at<float>(y,x) << ", " << this->distMappingXYLeft[1].at<float>(y, x) << ")" << std::endl;
    }
  }
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetShouldAlignBaseline(bool input)
{
  this->ShouldAlignBaseline = input;
  this->ComputeEssential();
}

//---------------------------------------------------------------------------
void StereoVideoManager::ComputeEssential()
{
  if (this->leftMatched.size() == 0)
  {
    return;
  }

  // convert cv matrix to eigen matrix
  for (int i = 0; i < this->F.rows; ++i)
  {
    for (int j = 0; j < this->F.cols; ++j)
    {
      this->Fund(i, j) = this->F.at<double>(i, j);
    }
  }

  std::cout << "Fundamental determinant : " << this->Fund.determinant() << std::endl;


  std::cout << "Using K1 : " << std::endl << this->K1 << std::endl;
  std::cout << "Using K2 : " << std::endl << this->K2 << std::endl;

  // Compute E from fundamental and Kpriors
  this->E = this->K2.transpose() * this->Fund * this->K1;
  std::cout << "Essential matrix : " << std::endl << this->E << std::endl;

  // now compute R and T from Essential matrix
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(this->E, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Matrix<double, 3, 3> V = svd.matrixV();
  Eigen::Matrix<double, 3, 3> D;
  double singularValue = (svd.singularValues()(0) + svd.singularValues()(1)) / 2.0;
  D << svd.singularValues()(0), 0, 0, 0, svd.singularValues()(1), 0, 0, 0, svd.singularValues()(2);

  std::cout << "Essential singular values are:" << std::endl << D << std::endl;
  std::cout << "Essential left singular vectors are the columns of the thin U matrix:" << std::endl << U << std::endl;
  std::cout << "Essential right singular vectors are the columns of the thin V matrix:" << std::endl << V << std::endl;

  Eigen::Matrix<double, 3, 3> W;
  W << 0, -1, 0,
       1,  0, 0,
       0,  0, 1;

  // Compute R and rescale R so that det(R) = 1
  Eigen::Matrix<double, 3, 3> R = U * W * V.transpose();
  R = std::cbrt(1.0 / R.determinant()) * R;

  double PI = 3.14159265359;
  // Euler angle parametrization of SO(3)
  double phi, theta, psi;
  phi = std::atan2(R(2, 1), R(2, 2)) * 180 / PI;
  theta = -std::sin(R(2, 0)) * 180 / PI;
  psi = -std::atan2(R(1, 0), R(0, 0)) * 180 / PI;

  // We should not have a big angle between the two views. If it is the case
  // we assume that it is because we chose the wrong solution among the 4
  if (std::abs(phi) > 100 || std::abs(theta) > 100 || std::abs(psi) > 100)
  {
    R = U * W.transpose() * V.transpose();
    R = std::cbrt(1.0 / R.determinant()) * R;

    phi = std::atan2(R(2, 1), R(2, 2)) * 180 / PI;
    theta = -std::sin(R(2, 0)) * 180 / PI;
    psi = -std::atan2(R(1, 0), R(0, 0)) * 180 / PI;
  }

  this->RelR = R;
  std::cout << "R : " << R << std::endl;
  std::cout << "Rotation angles : [" << phi << ";" << theta << ";" << psi << "]" << std::endl;

  // Compute T
  Eigen::Matrix<double, 3, 1> T;
  T << U(0, 2), U(1, 2), U(2, 2);
  T.normalize();

  Eigen::Matrix<double, 3, 3> Id;
  Id << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

  this->T1 << 0, 0, 0;
  this->T2 << T(0), T(1), T(2);
  this->RelT = T;

  // Aligne the axes of the cameras. To do that we decided
  // to change the axis of both camera so that the homography
  // distortion will be balanced between the two images. The
  // new axis of the cameras expressed in the referential of
  // the left camera will be [ex|ey|ez] = Rdemi
  Eigen::Quaternion<double> q2(R);
  Eigen::Quaternion<double> q1(Id);
  Eigen::Quaternion<double> qdemi = q2.slerp(0.5, q1);
  Eigen::Matrix<double, 3, 3> Rdemi = qdemi.toRotationMatrix();
  this->R1 = Rdemi;
  this->R2 = Rdemi.transpose(); // R1 = R2 * R
  std::cout << "R1 : " << std::endl << R1 << std::endl;
  std::cout << "R2 : " << std::endl << R2 << std::endl;

  // Now we will aligne the new x axis of the cameras with
  // the baseline
  Eigen::Matrix<double, 3, 1> ex, ey, ez, ex2, ey2, ez2, axeRot;
  ex = Rdemi.col(0);
  ey = Rdemi.col(1);
  ez = Rdemi.col(2);

  // We want the direction of the baseline
  // into the image plane. Since we will
  // normalized this direction we can use a orthogonal
  // projection instead of a central one
  Eigen::Matrix<double, 2, 1> Tprojected2D;
  Tprojected2D << T.dot(ex), T.dot(ey);
  Tprojected2D.normalize();
  if (Tprojected2D(0) < 0)
  {
    Tprojected2D = -Tprojected2D;
  }

  // angle between ex and TprojectedXY
  double angle1 = std::atan2(Tprojected2D(1), Tprojected2D(0));
  std::cout << "angle between baseline correction: " << angle1 / PI * 180.0 << std::endl;

  // Add an extra rotation matrix around
  // Z-axis to set the baseline horizontal
  Eigen::Matrix<double, 3, 3> Rz;
  Rz << std::cos(angle1), -std::sin(angle1), 0,
        std::sin(angle1),  std::cos(angle1), 0,
                       0,                 0, 1;

  if (this->ShouldAlignBaseline)
  {
    // Now aligne the axes with the baseline
    this->R1 = Rz.transpose() * this->R1; // first align between cameras
    this->R2 = Rz.transpose() * this->R2; // then align with baseline
  }

  // Now, compute the homographies between image and simulated pose
  this->H1 = this->K1 * this->R1 * Id.transpose() * this->K1.inverse();
  this->H2 = this->K2 * this->R2 * this->K2.inverse();

  this->Hcv1 = cv::Mat(this->F.size(), this->F.type());
  this->Hcv2 = cv::Mat(this->F.size(), this->F.type());

  for (int i = 0; i < this->F.rows; ++i)
  {
    for (int j = 0; j < this->F.cols; ++j)
    {
      this->Hcv1.at<double>(i, j) = this->H1(i, j);
      this->Hcv2.at<double>(i, j) = this->H2(i, j);
    }
  }

  std::cout << "H1 : " << std::endl << this->Hcv1 << std::endl;
  std::cout << "H2 : " << std::endl << this->Hcv2 << std::endl;

  // And now the cameras matrix
  this->P1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;

  this->P2 << R(0, 0), R(0, 1), R(0, 2), T(0),
              R(1, 0), R(1, 1), R(1, 2), T(1),
              R(2, 0), R(2, 1), R(2, 2), T(2);

  this->P1 = this->K1 * this->P1;
  this->P2 = this->K2 * this->P2;

  this->Camera1 = cv::Mat(3, 4, this->F.type());
  this->Camera2 = cv::Mat(3, 4, this->F.type());
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      this->Camera1.at<double>(i, j) = this->P1(i, j);
      this->Camera2.at<double>(i, j) = this->P2(i, j);
    }
  }

  std::cout << "P1 : " << std::endl << this->P1 << std::endl;
  std::cout << "P2 : " << std::endl << this->P2 << std::endl;

  this->IsCalibrated = true;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SimulateNewPose(double relativeParameters[3])
{
  std::cout << std::endl << std::endl << std::endl;
  double phi = relativeParameters[0] * 3.14159265359 / 180.0;
  double theta = relativeParameters[1] * 3.14159265359 / 180.0;
  double psi = relativeParameters[2] * 3.14159265359 / 180.0;

  Eigen::Matrix<double, 3, 3> Rtot;
  Eigen::Matrix<double, 3, 3> Rx, Ry, Rz;

  Rz << std::cos(psi), -std::sin(psi), 0,
    std::sin(psi), std::cos(psi), 0,
    0, 0, 1;

  Rx << 1, 0, 0,
    0, std::cos(phi), -std::sin(phi),
    0, std::sin(phi), std::cos(phi);

  Ry << std::cos(theta), 0, std::sin(theta),
    0, 1, 0,
    -std::sin(theta), 0, std::cos(theta);

  Rtot = Rz * Ry * Rx;

  std::cout << "Rrec : " << this->RelR << std::endl;
  std::cout << "R : " << Rtot << std::endl;

  Eigen::Matrix<double, 3, 3> Id;
  Id << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;

  Eigen::Quaternion<double> q2(Rtot);
  Eigen::Quaternion<double> q1(Id);

  Eigen::Quaternion<double> qdemi = q2.slerp(0.5, q1);
  Eigen::Matrix<double, 3, 3> Rdemi = qdemi.toRotationMatrix();

  std::cout << "R1 : " << this->R1 << std::endl;
  std::cout << "R1rec : " << Rdemi << std::endl;
  std::cout << "prod : " << this->R1 * Rdemi << std::endl;

  this->H1 = this->K1 * Rdemi * this->R1 * this->K1.inverse();
  this->H2 = this->K2 * Rdemi.transpose() * this->R2 * this->K2.inverse();

  this->Hcv1 = cv::Mat(this->F.size(), this->F.type());
  this->Hcv2 = cv::Mat(this->F.size(), this->F.type());

  for (int i = 0; i < this->F.rows; ++i)
  {
    for (int j = 0; j < this->F.cols; ++j)
    {
      this->Hcv1.at<double>(i, j) = this->H1(i, j);
      this->Hcv2.at<double>(i, j) = this->H2(i, j);
    }
  }

  //std::cout << "H1 : " << std::endl << this->Hcv1 << std::endl;
  //std::cout << "H2 : " << std::endl << this->Hcv2 << std::endl;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SimulateNewPose(double leftParameters[3], double rightParameters[3])
{
  double phi = leftParameters[0] * 3.14159265359 / 180.0;
  double theta = leftParameters[1] * 3.14159265359 / 180.0;
  double psi = leftParameters[2] * 3.14159265359 / 180.0;

  Eigen::Matrix<double, 3, 3> Rl, Rr;
  Eigen::Matrix<double, 3, 3> Rx, Ry, Rz;

  Rz << std::cos(psi), -std::sin(psi), 0,
        std::sin(psi),  std::cos(psi), 0,
                    0,              0, 1;

  Rx << 1,             0,              0,
        0, std::cos(phi), -std::sin(phi),
        0, std::sin(phi),  std::cos(phi);

  Ry << std::cos(theta), 0, std::sin(theta),
        0,               1,               0,
        -std::sin(theta), 0, std::cos(theta);

  Rl = Rz * Ry * Rx;

  phi = rightParameters[0] * 3.14159265359 / 180.0;
  theta = rightParameters[1] * 3.14159265359 / 180.0;
  psi = rightParameters[2] * 3.14159265359 / 180.0;

  Rz << std::cos(psi), -std::sin(psi), 0,
        std::sin(psi), std::cos(psi), 0,
        0, 0, 1;

  Rx << 1, 0, 0,
        0, std::cos(phi), -std::sin(phi),
        0, std::sin(phi), std::cos(phi);

  Ry << std::cos(theta), 0, std::sin(theta),
        0, 1, 0,
       -std::sin(theta), 0, std::cos(theta);

  Rr = Rz * Ry * Rx;

  this->H1 = this->K1 * Rl * this->R1 * this->K1.inverse();
  this->H2 = this->K2 * Rr * this->R2 * this->K2.inverse();

  this->Hcv1 = cv::Mat(this->F.size(), this->F.type());
  this->Hcv2 = cv::Mat(this->F.size(), this->F.type());

  for (int i = 0; i < this->F.rows; ++i)
  {
    for (int j = 0; j < this->F.cols; ++j)
    {
      this->Hcv1.at<double>(i, j) = this->H1(i, j);
      this->Hcv2.at<double>(i, j) = this->H2(i, j);
    }
  }

  std::cout << "H1 : " << std::endl << this->Hcv1 << std::endl;
  std::cout << "H2 : " << std::endl << this->Hcv2 << std::endl;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SimulateNewPose(Eigen::Matrix<double, 3, 3> R)
{
  Eigen::Matrix<double, 3, 3> Id;
  Id << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;

  Eigen::Quaternion<double> q2(R);
  Eigen::Quaternion<double> q1(Id);

  Eigen::Quaternion<double> qdemi = q2.slerp(0.5, q1);
  Eigen::Matrix<double, 3, 3> Rdemi = qdemi.toRotationMatrix();

  this->H1 = this->K1 * Rdemi * this->R1 * this->K1.inverse();
  this->H2 = this->K2 * Rdemi.transpose() * this->R2 * this->K2.inverse();

  this->Hcv1 = cv::Mat(this->F.size(), this->F.type());
  this->Hcv2 = cv::Mat(this->F.size(), this->F.type());

  for (int i = 0; i < this->F.rows; ++i)
  {
    for (int j = 0; j < this->F.cols; ++j)
    {
      this->Hcv1.at<double>(i, j) = this->H1(i, j);
      this->Hcv2.at<double>(i, j) = this->H2(i, j);
    }
  }
}

//---------------------------------------------------------------------------
bool StereoVideoManager::GetIsCalibrated()
{
  return this->IsCalibrated;
}

//---------------------------------------------------------------------------
cv::Mat StereoVideoManager::GetH(unsigned int index)
{
  if (index == 0)
  {
    this->Hcv1;
  }
  else
  {
    return this->Hcv2;
  }
}

//---------------------------------------------------------------------------
double StereoVideoManager::GetCalibrationProgression()
{
  return this->CalibrationProgression;
}

//---------------------------------------------------------------------------
cv::Mat StereoVideoManager::GetF()
{
  return this->F;
}

//---------------------------------------------------------------------------
std::vector<cv::KeyPoint> StereoVideoManager::GetLeftMatched()
{
  return this->leftMatched;
}

//---------------------------------------------------------------------------
std::vector<cv::KeyPoint> StereoVideoManager::GetRightMatched()
{
  return this->rightMatched;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetStreamPos(int pos)
{
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, this->sync1 + pos);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, this->sync2 + pos);
}

//---------------------------------------------------------------------------
void StereoVideoManager::RecoveringCamerasDistance(double distancePoints, std::vector<std::pair<cv::Point2f, cv::Point2f> > points)
{
  std::vector<cv::Point2f> u, v;
  for (int k = 0; k < points.size(); ++k)
  {
    u.push_back(points[k].first);
    v.push_back(points[k].second);
  }

  std::vector<Eigen::Matrix<float, 3, 1> > X = this->Triangulate(u, v);

  std::cout << "Point 1 : " << std::endl << X[0] << std::endl;
  std::cout << "Point 2 : " << std::endl << X[1] << std::endl;

  Eigen::Matrix<float, 3, 1> X1X2 = X[1] - X[0];
  double length = X1X2.norm();
  double scale = distancePoints / length;
  double distUn = this->RelT.norm();
  this->RelT = this->RelT * scale;
  double distCam = this->RelT.norm();

  std::cout << "old length : " << distUn << std::endl;
  std::cout << "camera length : " << distCam << std::endl;

  // recompute camera matrix with the scale known
  // And now the cameras matrix
  this->P1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;

  this->P2 << this->RelR(0, 0), this->RelR(0, 1), this->RelR(0, 2), this->RelT(0),
              this->RelR(1, 0), this->RelR(1, 1), this->RelR(1, 2), this->RelT(1),
              this->RelR(2, 0), this->RelR(2, 1), this->RelR(2, 2), this->RelT(2);

  this->P1 = this->K1 * this->P1;
  this->P2 = this->K2 * this->P2;

  this->Camera1 = cv::Mat(3, 4, this->F.type());
  this->Camera2 = cv::Mat(3, 4, this->F.type());
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      this->Camera1.at<double>(i, j) = this->P1(i, j);
      this->Camera2.at<double>(i, j) = this->P2(i, j);
    }
  }

  X = this->Triangulate(u, v);
  std::cout << "Point new 1 : " << std::endl << X[0] << std::endl;
  std::cout << "Point new 2 : " << std::endl << X[1] << std::endl;

  std::cout << "P1 : " << std::endl << this->P1 << std::endl;
  std::cout << "P2 : " << std::endl << this->P2 << std::endl;
}

//---------------------------------------------------------------------------
void StereoVideoManager::SetCameraDistance(double distanceCamera)
{
  double length = this->RelT.norm();
  double scale = distanceCamera / length;
  this->RelT = this->RelT * scale;
  double distCam = this->RelT.norm();

  std::cout << "old length : " << length << std::endl;
  std::cout << "camera length : " << distCam << std::endl;

  // recompute camera matrix with the scale known
  // And now the cameras matrix
  this->P1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;

  this->P2 << this->RelR(0, 0), this->RelR(0, 1), this->RelR(0, 2), this->RelT(0),
              this->RelR(1, 0), this->RelR(1, 1), this->RelR(1, 2), this->RelT(1),
              this->RelR(2, 0), this->RelR(2, 1), this->RelR(2, 2), this->RelT(2);

  this->P1 = this->K1 * this->P1;
  this->P2 = this->K2 * this->P2;

  this->Camera1 = cv::Mat(3, 4, this->F.type());
  this->Camera2 = cv::Mat(3, 4, this->F.type());
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      this->Camera1.at<double>(i, j) = this->P1(i, j);
      this->Camera2.at<double>(i, j) = this->P2(i, j);
    }
  }
}

//---------------------------------------------------------------------------
std::vector<Eigen::Matrix<float, 3, 1> > StereoVideoManager::Triangulate(std::vector<cv::Point2f> pt1, std::vector<cv::Point2f> pt2)
{
  std::vector<Eigen::Matrix<float, 3, 1> > X;

  cv::Mat Res; // float type 5

               // Triangulate the points
  cv::triangulatePoints(this->Camera1, this->Camera2, pt1, pt2, Res);

  // Note the "-1" because the last coordinates is the homogeneous one
  for (int j = 0; j < Res.cols; ++j)
  {
    // Get the homogeneous scale factor
    double w = Res.at<float>(3, j);

    // current vector triangulated
    Eigen::Matrix<float, 3, 1> current;

    for (int i = 0; i < Res.rows - 1; ++i)
    {
      current(i) = Res.at<float>(i, j) / w;
    }
    X.push_back(current);
  }

  return X;
}

//---------------------------------------------------------------------------
void StereoVideoManager::ComputeSyncSignal(double* data1, double* data2, int* size1, int* size2, int maxAllocation, double startTime)
{
  // Save the current pos
  int pos1 = this->Video1.get(CV_CAP_PROP_POS_FRAMES);
  int pos2 = this->Video2.get(CV_CAP_PROP_POS_FRAMES);

  // Go to the first frames
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, startTime * this->Info[0].Fps);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, startTime * this->Info[0].Fps);

  int nbrFrame1 = this->Info[0].NbrFrame;
  int nbrFrame2 = this->Info[1].NbrFrame;
  int nbrSample = std::min(maxAllocation, std::min(nbrFrame1, nbrFrame2));

  std::cout << "frame 1 : " << nbrFrame1 << std::endl;
  std::cout << "frame 2 : " << nbrFrame2 << std::endl;
  std::cout << "fps 1 : " << this->Info[0].Fps << std::endl;
  std::cout << "fps 2 : " << this->Info[1].Fps << std::endl;

  cv::Mat Im;
  int H, W;
  int n = std::sqrt(1.0 * H * 1.0 * W / 20000.0);
  size1[0] = 0;
  size2[0] = 0;

  double cb = 333.3333;//114.0;
  double cg = 333.3333;//587.0;
  double cr = 333.3333;//299.0
  double s = 0.4;

  clock_t begin = clock();
  double progression = 0;
  int nbrIteration1 = std::min(maxAllocation, nbrFrame1 - static_cast<int>(startTime * this->Info[0].Fps));
  int nbrIteration2 = std::min(maxAllocation, nbrFrame2 - static_cast<int>(startTime * this->Info[0].Fps));
  double nbrOperations = nbrIteration1 + nbrIteration2;

  // fill first signal
  while (this->Video1.read(Im) && (size1[0] < maxAllocation))
  {
    H = Im.rows;
    W = Im.cols;
    std::vector<double> values;
    n = std::max(1, static_cast<int>(std::sqrt(1.0 * H * 1.0 * W / 20000.0)));
    //std::cout << "n : " << n << std::endl;
    // fill the values
    for (int i = 0; i < H; i = i + n)
    {
      for (int j = 0; j < W; j = j + n)
      {
        cv::Vec3b colour = Im.at<cv::Vec3b>(i, j);
        // compute the bright
        values.push_back((cb * colour[0] + cg * colour[1] + cb * colour[2]) / 1000.0);
      }
    }

    // get the median
    std::sort(values.begin(), values.end());

    double moyMidPercentil = 0;
    int startPercentil = s * values.size();
    int endPercentil = (1 - s) * values.size();
    for (int k = startPercentil; k < endPercentil; ++k)
    {
      moyMidPercentil += values[k];
    }

    moyMidPercentil /= (1.0 * endPercentil - 1.0 * startPercentil);
    data1[size1[0]] = moyMidPercentil;
    size1[0]++;
    progression = progression + 1;
    this->ComputationProgression->setValue(100.0 * progression / nbrOperations);
  }

  // fill second signal
  while (this->Video2.read(Im) && (size2[0] < maxAllocation))
  {
    H = Im.rows;
    W = Im.cols;
    std::vector<double> values;

    // fill the values
    for (int i = 0; i < H; i = i + n)
    {
      for (int j = 0; j < W; j = j + n)
      {
        cv::Vec3b colour = Im.at<cv::Vec3b>(i, j);
        // compute the bright
        values.push_back((cb * colour[0] + cg * colour[1] + cb * colour[2]) / 1000.0);
      }
    }

    // get the median
    std::sort(values.begin(), values.end());

    double moyMidPercentil = 0;
    int startPercentil = s * values.size();
    int endPercentil = (1 - s) * values.size();
    for (int k = startPercentil; k < endPercentil; ++k)
    {
      moyMidPercentil += values[k];
    }

    moyMidPercentil /= (1.0 * endPercentil - 1.0 * startPercentil);
    data2[size2[0]] = moyMidPercentil;
    size2[0]++;
    progression = progression + 1;
    this->ComputationProgression->setValue(100.0 * progression / nbrOperations);
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Elapsed time : " << elapsed_secs << std::endl;

  double fps1 = this->Info[0].Fps;
  double fps2 = this->Info[1].Fps;

  // if FPS are differents
  /*if (std::abs(fps1 - fps2) > 2)
  {
    // Adjust data to match same FPS
    if (fps1 > fps2)
    {
      std::vector<double> temp;
      int k = 0;
      int i = 0;
      while (k < size1[0])
      {
        temp.push_back(data1[k]);
        double currentTime = i / fps2;
        k = currentTime * fps1;
        i++;
      }

      size1[0] = temp.size();
      for (int j = 0; j < size1[0]; ++j)
      {
        data1[j] = temp[j];
      }
    }

    // Adjust data to match same FPS
    if (fps2 > fps1)
    {
      std::vector<double> temp;
      int k = 0;
      int i = 0;
      while (k < size2[0])
      {
        temp.push_back(data2[k]);
        double currentTime = i / fps1;
        k = currentTime * fps2;
        i++;
      }

      size2[0] = temp.size();
      for (int j = 0; j < size2[0]; ++j)
      {
        data2[j] = temp[j];
      }
    }
  }*/ // works but for now the soft does not support diff fps

  for (int k = size1[0] - 1; k > 0; --k)
  {
    data1[k] = data1[k] - data1[k - 1];
  }

  for (int k = size2[0] - 1; k > 0; --k)
  {
    data2[k] = data2[k] - data2[k - 1];
  }

  data1[0] = 0;
  data2[0] = 0;

  // Set the pos to the initial value
  this->Video1.set(CV_CAP_PROP_POS_FRAMES, pos1);
  this->Video2.set(CV_CAP_PROP_POS_FRAMES, pos2);
}

/*
// Save the current pos
int pos1 = this->Video1.get(CV_CAP_PROP_POS_FRAMES);
int pos2 = this->Video2.get(CV_CAP_PROP_POS_FRAMES);

// Go to the first frames
this->Video1.set(CV_CAP_PROP_POS_FRAMES, 0);
this->Video2.set(CV_CAP_PROP_POS_FRAMES, 0);

int nbrFrame1 = this->Info[0].NbrFrame;
int nbrFrame2 = this->Info[0].NbrFrame;
int nbrSample = std::min(std::min(nbrFrame1, nbrFrame2), maxAllocation);

double fps1 = this->Info[0].Fps;
double fps2 = this->Info[1].Fps;

double duration1 = nbrFrame1 / fps1;
double duration2 = nbrFrame2 / fps2;
double duration = std::min(duration1, duration2);

std::cout << "FPS 1 : " << fps1 << std::endl;
std::cout << "FPS 2 : " << fps2 << std::endl;
std::cout << "duration 1 : " << duration1 << std::endl;
std::cout << "duration 2 : " << duration2 << std::endl;

double stepSeconds = duration / nbrSample;

cv::Mat Im;
int H, W;
int n = 10;
size1[0] = 0;
size2[0] = 0;

double cb = 333.3333;//114.0;
double cg = 333.3333;//587.0;
double cr = 333.3333;//299.0;

size1[0] = nbrSample;
size2[1] = nbrSample;

for (int k = 0; k < nbrSample; ++k)
{
double currentTime = stepSeconds * k;
int currentPosition = fps1 * currentTime;
this->Video1.set(CV_CAP_PROP_POS_FRAMES, currentPosition);
this->Video1.read(Im);

H = Im.rows;
W = Im.cols;
std::vector<double> values;

// fill the values
for (int i = 0; i < H; i = i + n)
{
for (int j = 0; j < W; j = j + n)
{
cv::Vec3b colour = Im.at<cv::Vec3b>(i, j);
// compute the bright
values.push_back((cb * colour[0] + cg * colour[1] + cb * colour[2]) / 1000.0);
}
}

// get the median
std::sort(values.begin(), values.end());

data1[k] = values[values.size() / 2];
}

for (int k = 0; k < nbrSample; ++k)
{
double currentTime = stepSeconds * k;
int currentPosition = fps2 * currentTime;
this->Video2.set(CV_CAP_PROP_POS_FRAMES, currentPosition);
this->Video2.read(Im);

H = Im.rows;
W = Im.cols;
std::vector<double> values;

// fill the values
for (int i = 0; i < H; i = i + n)
{
for (int j = 0; j < W; j = j + n)
{
cv::Vec3b colour = Im.at<cv::Vec3b>(i, j);
// compute the bright
values.push_back((cb * colour[0] + cg * colour[1] + cb * colour[2]) / 1000.0);
}
}

// get the median
std::sort(values.begin(), values.end());

data2[k] = values[values.size() / 2];
}
*/