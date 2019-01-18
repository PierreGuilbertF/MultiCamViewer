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
 * @class   Calibrator
 * @brief   Calibrate two cameras
 *
 * Calibrator is a convenience class for calibrating two or more camera.
 * It gives the intrinsecs and extrinsecs parameters of each camera relatively
 * to the first camera's camera frames
 *
*/
#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#define NCamera 2

// EIGEN
#include <Eigen/Dense>

// OPENCV
#include <opencv2/core/core.hpp>

// LOCAL
#include "Toolkit.h"

struct CalibratorParams
{
  // parameters setted regarding :
  // http://docs.opencv.org/3.1.0/da/df5/tutorial_py_sift_intro.html
  // Indicates the maximum number of features
  int nFeatures = 50000;

  // The number of layers in each octave. 3 is the value used in D. Lowe paper.
  // The number of octaves is computed automatically from the image resolution
  int nOctaves = 3;

  // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions.
  // The larger the threshold, the less features are produced by the detector
  double contrastThreshold = 0.02;

  // The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold,
  // i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).
  double edgeThreshold = 20;

  // The sigma of the Gaussian applied to the input image at the octave #0. If your image
  // is captured with a weak camera with soft lenses, you might want to reduce the number.
  double sigma = 1.6;

  // threshold used to keep a match. If a keypoints match with 2 keypoints with
  // score X1 and X2 and the ratio X1 / X2 > 0.75 the point wont be kept
  double ratioMatchingThresh = 0.75;

  // Parameters for finding the fundamental matrix
  // param1 determines the maximum distance between a pixel and its epipolar line
  // to be considered as an inlier. It depends on the noise and the image resolution.
  // Set to 3 for high res images
  double param1 = 0.02;

  // Specify a desirable level of confidence that the estimated matrix is correct
  // 99% is the recommanded value by opencv documentation
  double param2 = 0.99995;

};

class Calibrator
{
public:
  // default constructor
  Calibrator();

  // Set the image used for calibration for the camera
  // idxCamera. Return 0 if a problem is detected
  void SetImage(std::string filename, int idxCamera);

  // Set the image
  void SetImage(cv::Mat Im, int idxCamera);

  // return the image of the camera idxCamera
  cv::Mat GetImage(int idxCamera);

  // Get the keypoints of the image of the camera idxCamera
  std::vector<cv::KeyPoint> GetKeyPoint(int idxCamera);

  // Get the matched keypoints of the image of the camera idxCamera
  std::vector<cv::KeyPoint> GetMatchedKeyPoint(int idxCamera);

  // Get the keypoints matching
  std::vector<cv::DMatch> GetMatches();

  // Get the good keypoints matching
  std::vector<cv::DMatch> GetGoodMatches();

  // Launch the calibration processing. The extrinsecs parameters will
  // be given in the first camera frame. Hence, the extrinsec matrix
  // of the first camera will be identity.
  void Calibrate();

  // return the fundamental matrix
  cv::Mat GetF();

  // set the fundamental matrix if computed by another class
  void SetF(cv::Mat Farg, std::vector<cv::KeyPoint> leftMatch, std::vector<cv::KeyPoint> rightMatch);

  // return the fundamental matrix
  Eigen::Matrix<double, 3, 3> GetFundamental();

  // return the essential matrix
  Eigen::Matrix<double, 3, 3> GetE();

  // return camera matrix
  Eigen::Matrix<double, 3, 4> GetP(int idxImage);

  // Return the 3D points corresponding to the pair of 2D points
  std::vector<Eigen::Matrix<float, 3, 1> > Triangulate(std::vector<Eigen::Matrix<float, 2, 1> > pt1,
                                                        std::vector<Eigen::Matrix<float, 2, 1> > pt2);

  // Return the 3D points corresponding to the pair of 2D points
  std::vector<Eigen::Matrix<float, 3, 1> > Triangulate2(std::vector<Eigen::Matrix<float, 2, 1> > pt1,
                                                        std::vector<Eigen::Matrix<float, 2, 1> > pt2);

  // structure which contain the parameters of the calibrator
  CalibratorParams params;

  // show some results to visualy check the calibration
  void ShowCalibrationResult();

  // Get images which show the result of the calibration
  std::vector<cv::Mat> GetCalibrationResults();

  // Extract the extrinsecs parameters of the second camera
  // relatively to the first camera
  void ExtractExtrinsecParameters();

  // Set the Usage of prior knowledges
  void SetUsePrior(bool value);

  // Recompute the essential and fundamental matrix to apply transform
  void SimulateTransform(Eigen::Matrix<double, 3, 3> Rc, Eigen::Matrix<double, 3, 1> Tc,
                         Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2);

protected:
  // Indicates the number of camera
  int NumberCamera;

  // Images used to calibrate the cameras
  std::vector<cv::Mat> Images;

  // List of key points of the images
  // [cameraidx][keypointsidx]
  std::vector<std::pair<std::vector<cv::KeyPoint>, cv::Mat> > KeyPoints;

  // Keypoints matching over cameras after ransac
  std::vector<std::vector<cv::KeyPoint> > MatchedKeypoints;

  // Keypoints matching over cameras
  std::vector<cv::DMatch> Matches;

  // KeyPoints matching over cameras according
  // to the ransac filtering
  std::vector<cv::DMatch> GoodMatches;

  // Compute the keyPoints and descriptors of images pointed buy idx
  void ComputeKeyPoints();

  // Compute the essential matrix of the two cameras
  void ComputeEssentialMatrix();

  // Compute the essential matrix with a prior value of K
  void ComputeEssentialMatrixPriorK(Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2);

  // Match the keyPoints of images pointed buy idx using the descriptors
  void MatchKeyPoints();

  // Indicates if the images has been set
  std::vector<bool> IsImageSet;

  // Fundamental matrix between Im0 and Im1
  cv::Mat F;

  // Camera matrix
  cv::Mat P1;
  cv::Mat P2;

  // Max number of matched key point used in
  // the computation of the fundamental matrix
  int MaxMatchKeyPointNumber;

  Eigen::Matrix<double, 3, 3> Fundamental;
  Eigen::Matrix<double, 3, 3> Essential;
  Eigen::Matrix<double, 3, 4> Camera1;
  Eigen::Matrix<double, 3, 4> Camera2;
  Eigen::Matrix<double, 3, 3> K;
  Eigen::Matrix<double, 3, 1> T;
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 3, 1> Angles;

  // ude Prior knowledges about matrix
  bool UsePrior;
};

#endif
