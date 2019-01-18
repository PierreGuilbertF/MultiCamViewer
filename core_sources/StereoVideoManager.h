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
 * @brief   Stereo Video
 *
 *
*/

#ifndef STEREO_VIDEO_MANAGER_H
#define STEREO_VIDEO_MANAGER_H

#define NCamera 2

// EIGEN
#include <Eigen/Dense>

// OPENCV
#include <opencv2/core/core.hpp>

// LOCAL
#include "Calibrator.h"
#include "Toolkit.h"

// QT : WIP
#include <QProgressBar>

class VideoInfo
{
public:
  VideoInfo();

  double CurrentPos;
  double RelativeCurrentPos;
  unsigned int Width;
  unsigned int Height;
  double Fps;
  unsigned int NbrFrame;
  unsigned int IndexFrame;
};

class StereoVideoManager
{
public:
  // Default constructor
  StereoVideoManager();

  // open the left video
  void OpenVideo1(std::string filename);

  // open the right video
  void OpenVideo2(std::string filename);

  // open the video indicated by the filename 1 & 2
  void OpenVideo(std::string filename1, std::string filename2);

  // Show the videos information
  void ShowInfo();

  // Get the video
  cv::VideoCapture* GetVideo(int idxVideo);

  // Get the calibrator
  Calibrator* GetCalibrator();

  // Update the videos information
  void UpdateInfo();

  // Update the intrinsics parameters
  // using internal parameters and
  // image sizes
  void UpdateIntrinsicParams();

  // Get the informations of a video
  VideoInfo GetVideoInfo(int idxVideo);

  // export the rectified images
  void ExportVideoRectified(std::string filename, int idxCamera);

  // Make the calibration of the camera using multiple
  // frame assuming that the two camera form a solid
  void SolidCameraCalibration(int nFrame, CalibratorParams parameters);

  // Simulate new pose using the entry rotation matrix
  void SimulateNewPose(Eigen::Matrix<double, 3, 3> R);

  // set the syncrhonization offsets
  void SetSyncs(double s1, double s2);

  // Set the fundamental matrix
  void SetF(cv::Mat Farg);

  // Set the fundamental and matched points
  void SetF(cv::Mat Farg, std::vector<cv::KeyPoint> K1, std::vector<cv::KeyPoint> K2);

  // Set the left matched keypoints
  void SetLeftMatched(std::vector<cv::KeyPoint> K);

  // Set the right matched keypoints
  void SetRightMatched(std::vector<cv::KeyPoint> K);

  // Set if the stereomanager should export the result
  void SetShouldExport(bool should, std::string filename);

  // Set the use prior
  void SetUsePrior(bool value);

  // Set the parameters of the calibrator
  void SetCalibratorParameters(CalibratorParams value);

  // return if the stereo is ready
  bool GetIsStereoReady();

  // return the number of fps of the camera
  double GetFps(int indexCamera);

  // return if the stereo is calibrated
  bool GetIsCalibrated();

  // Recovering the distance between cameras using the two
  // 3D points provided and the distance between them
  void RecoveringCamerasDistance(double distancePoints, std::vector<std::pair<cv::Point2f, cv::Point2f> > points);

  // Set the distance between the two cameras
  void SetCameraDistance(double distanceCamera);

  // return the homographie
  cv::Mat GetH(unsigned int index);

  // warp input images using the result of calibration
  std::pair<std::vector< cv::Point2f >, std::vector< cv::Point2f > > WarpCalibration(cv::Mat& src1, cv::Mat& src2, cv::Mat& dst1, cv::Mat& dst2);

  // Get the calibration progression
  double GetCalibrationProgression();

  // Get left / right match keypoints
  std::vector<cv::KeyPoint> GetLeftMatched();
  std::vector<cv::KeyPoint> GetRightMatched();

  // return the fundamental matrix
  cv::Mat GetF();

  // reset the playback to the beginning
  void ResetStreamPosition();

  // Set the stream pos
  void SetStreamPos(int pos);

  // Compute 3D points from matched points and camera matrix
  std::vector<Eigen::Matrix<float, 3, 1> > Triangulate(std::vector<cv::Point2f> pt1, std::vector<cv::Point2f> pt2);

  // Compute new homographies to simulate new pose of the cameras
  void SimulateNewPose(double leftParameters[3], double rightParameters[3]);

  // Compute new homographies to simulate new pose of the cameras
  void SimulateNewPose(double relativeParameters[3]);

  // Compute signal of the image to sync
  void ComputeSyncSignal(double* data1, double* data2, int* size1, int* size2, int maxAllocation, double startTime);

  // Set the Internal progress bar
  void SetProgressBar(QProgressBar* input);

  // Undistord hte image using the distortion coefficient
  // and the intrinsic camera matrix
  void UndistordImage(cv::Mat Input, cv::Mat& Output, unsigned int camIndex);

  //
  void SetShouldAlignBaseline(bool input);

protected:
  // Calibrator used to calibrate the two camera
  Calibrator Calibr;

  // Container of the two videos
  cv::VideoCapture Video1;
  cv::VideoCapture Video2;

  // Synchronization between cameras
  int sync1;
  int sync2;

  // Bolean incating if the video are opened
  bool VideoReady[2];
  bool IsStereoReady;

  // Indicate if the calibration is ready
  bool IsCalibrated;

  // Information about the video
  VideoInfo Info[2];

  // Fundamental matrix computed over multiple images
  cv::Mat F;
  std::vector<cv::KeyPoint> leftMatched;
  std::vector<cv::KeyPoint> rightMatched;

  // Export results during multiple frame calibration
  bool shouldExportResults;
  std::string exportFilename;

  bool UsePrior;

  // Parameters of the calibrator
  CalibratorParams paramC;

  // Parameters of stereo

  // Rotation of the first image
  Eigen::Matrix<double, 3, 3> R1;
  // Rotation of the second image
  Eigen::Matrix<double, 3, 3> R2;
  //Translation of the first image
  Eigen::Matrix<double, 3, 1> T1;
  // translation of the second image
  Eigen::Matrix<double, 3, 1> T2;
  // fundamental matrix
  Eigen::Matrix<double, 3, 3> Fund;
  // essential matrix
  Eigen::Matrix<double, 3, 3> E;
  // homography to straight the images
  Eigen::Matrix<double, 3, 3> H1;
  Eigen::Matrix<double, 3, 3> H2;
  // cameras matrix
  Eigen::Matrix<double, 3, 4> P1;
  Eigen::Matrix<double, 3, 4> P2;
  // relative rotation
  Eigen::Matrix<double, 3, 3> RelR;
  // relative translation
  Eigen::Matrix<double, 3, 1> RelT;
  // intrinsecs parameters of cameras
  Eigen::Matrix<double, 3, 3> K1;
  Eigen::Matrix<double, 3, 3> K2;
  cv::Mat K1New;
  cv::Mat K2New;
  // Field Of View of the cameras
  double FOV;
  // Lens distortion parameters
  std::vector<double> DistortionCoeff;
  std::vector<cv::Mat> distMappingXYLeft;
  std::vector<cv::Mat> distMappingXYRight;

  // Align the baseline with horizontal
  bool ShouldAlignBaseline;

  // Homography to straight the images
  cv::Mat Hcv1;
  cv::Mat Hcv2;

  // Camera matrix
  cv::Mat Camera1;
  cv::Mat Camera2;

  // Indicate the calobration progression
  double CalibrationProgression;

  void ComputeEssential();

  QProgressBar* ComputationProgression;
};

#endif
