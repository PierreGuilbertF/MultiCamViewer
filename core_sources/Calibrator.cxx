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

// LOCAL
#include "Calibrator.h"

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

//---------------------------------------------------------------------------
Calibrator::Calibrator()
{
  // Initialized to 0 cameras
  this->NumberCamera = NCamera;
  this->Images.resize(this->NumberCamera);
  this->KeyPoints.resize(this->NumberCamera);
  this->IsImageSet.resize(this->NumberCamera, false);
  this->MatchedKeypoints.resize(this->NumberCamera);

  // Max number of matched keyPoints
  this->MaxMatchKeyPointNumber = 10000;

  this->UsePrior = false;
}

//---------------------------------------------------------------------------
void Calibrator::SetF(cv::Mat Farg, std::vector<cv::KeyPoint> leftMatch, std::vector<cv::KeyPoint> rightMatch)
{
  // Set F
  this->F = Farg;

  // Set F eigen version
  for (int i = 0; i < F.rows; ++i)
  {
    for (int j = 0; j < F.cols; ++j)
    {
      this->Fundamental(i, j) = F.at<double>(i, j);
    }
  }

  // Set the new matches corresponding to F
  cv::Mat M1, M2;
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> u1(leftMatch, M1);
  std::pair<std::vector<cv::KeyPoint>, cv::Mat> u2(rightMatch, M1);
  std::vector<std::pair<std::vector<cv::KeyPoint>, cv::Mat> > temp;
  temp.push_back(u1);
  temp.push_back(u2);

  this->KeyPoints = temp;

  // Compute mean error:
  Eigen::Matrix<double, 3, 1> X, Y;
  double meanErr = 0;
  double maxErr = 0;
  for (unsigned int k = 0; k < leftMatch.size(); ++k)
  {
    X << leftMatch[k].pt.x, leftMatch[k].pt.y, 1;
    Y << rightMatch[k].pt.x, rightMatch[k].pt.y, 1;

    double err = std::abs(X.transpose() * this->Fundamental * Y);
    meanErr = meanErr + err;
    maxErr = std::max(maxErr, err);
  }
  std::cout << "mean error: " << meanErr << " pixels" << std::endl;
  std::cout << "max error: " << maxErr << " pixels" << std::endl;
  
  this->ComputeEssentialMatrix();
}

//---------------------------------------------------------------------------
void Calibrator::SetImage(std::string filename, int idxCamera)
{
  // open the image
  cv::Mat Im = OpenImage(filename);

  // Verify if the image has been correctly opened
  if ((Im.cols == 100) && (Im.rows == 100))
  {
    std::cout << "Calibrator : AddImage failed" << std::endl;
    return;
  }

  // Verify that the camera index is correct
  if (idxCamera < 0 || idxCamera >= this->NumberCamera)
  {
    std::cout << "Calibrator : idxCamera must be in [0, " << this->NumberCamera - 1 << "] given : " << idxCamera << std::endl;
    return;
  }

  this->Images[idxCamera] = Im;
  this->IsImageSet[idxCamera] = true;
}

//---------------------------------------------------------------------------
void Calibrator::SetUsePrior(bool value)
{
  this->UsePrior = value;
}

//---------------------------------------------------------------------------
void Calibrator::SetImage(cv::Mat Im, int idxCamera)
{
  // Verify that the camera index is correct
  if (idxCamera < 0 || idxCamera >= this->NumberCamera)
  {
    std::cout << "Calibrator : idxCamera must be in [0, " << this->NumberCamera - 1 << "] given : " << idxCamera << std::endl;
    return;
  }

  this->Images[idxCamera] = Im;
  this->IsImageSet[idxCamera] = true;
}

//---------------------------------------------------------------------------
cv::Mat Calibrator::GetImage(int idxCamera)
{
  // Verify the camera index
  if ((idxCamera < 0) || (idxCamera >= this->NumberCamera))
  {
    std::cout << "Calibrator : idxCamera must be in [0, " << this->NumberCamera - 1 << "] given : " << idxCamera << std::endl;
    return cv::Mat::zeros(100, 100, CV_64F);
  }

  return this->Images[idxCamera];
}

//---------------------------------------------------------------------------
std::vector<cv::KeyPoint> Calibrator::GetKeyPoint(int idxCamera)
{
  // Verify the camera index
  if ((idxCamera < 0) || (idxCamera >= this->NumberCamera))
  {
    std::cout << "Calibrator : idxCamera must be in [0, " << this->NumberCamera - 1 << "] given : " << idxCamera << std::endl;
    return this->KeyPoints[0].first;
  }

  return this->KeyPoints[idxCamera].first;
}

//---------------------------------------------------------------------------
std::vector<cv::KeyPoint> Calibrator::GetMatchedKeyPoint(int idxCamera)
{
  // Verify the camera index
  if ((idxCamera < 0) || (idxCamera >= this->NumberCamera))
  {
    std::cout << "Calibrator : idxCamera must be in [0, " << this->NumberCamera - 1 << "] given : " << idxCamera << std::endl;
    return this->KeyPoints[0].first;
  }

  return this->MatchedKeypoints[idxCamera];
}

//---------------------------------------------------------------------------
std::vector<cv::DMatch> Calibrator::GetMatches()
{
  return this->Matches;
}

//---------------------------------------------------------------------------
std::vector<cv::DMatch> Calibrator::GetGoodMatches()
{
  return this->GoodMatches;
}

//---------------------------------------------------------------------------
cv::Mat Calibrator::GetF()
{
  return this->F;
}

//---------------------------------------------------------------------------
void Calibrator::ComputeKeyPoints()
{
  // for each camera
  for (int k = 0; k < this->NumberCamera; ++k)
  {
    // convert the current image in grayscale
    cv::Mat gray;
    
    if (this->Images[k].channels() == 3)
    {
      cv::cvtColor(this->Images[k], gray, CV_BGR2GRAY);
    }
    else
    {
      this->Images[k].copyTo(gray);
    }

    // parameters setted regarding :
    // http://docs.opencv.org/3.1.0/da/df5/tutorial_py_sift_intro.html
    int nFeatures = this->params.nFeatures; // Indicates the maximum number of features
    int nOctaves = this->params.nOctaves;
    double contrastThreshold = this->params.contrastThreshold;
    double edgeThreshold = this->params.edgeThreshold;
    double sigma = this->params.sigma;

    // initialization of the sift extractor with setted parameters
    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> detector =
        cv::xfeatures2d::SiftFeatureDetector::create(nFeatures, nOctaves, contrastThreshold, edgeThreshold, sigma);

    // Extraction of the keypoints
    detector->detect(gray, this->KeyPoints[k].first);

    // Compute descriptors associated to the keyPoints
    nOctaves = 5; // Increase accuracy while computing descriptors
    cv::Ptr<cv::xfeatures2d::SiftDescriptorExtractor> SiftExtractor =
        cv::xfeatures2d::SiftDescriptorExtractor::create(nFeatures, nOctaves, contrastThreshold, edgeThreshold, sigma);
    SiftExtractor->compute(gray, this->KeyPoints[k].first, this->KeyPoints[k].second);

    std::cout << "Calibrator : found keyPoints : " << this->KeyPoints[k].first.size() << std::endl;
  }
}

//---------------------------------------------------------------------------
void Calibrator::MatchKeyPoints()
{
  // Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> bad_matches;
  std::vector<std::vector<cv::DMatch> > knnMatches;

  // Match the two images :
  // Image 1 will be the query points
  // Image 0 will be the trained points
  //matcher.match(this->KeyPoints[0].second, this->KeyPoints[1].second, bad_matches);
  matcher.knnMatch(this->KeyPoints[0].second, this->KeyPoints[1].second, knnMatches, 2);

  // filter the matches, if the ratio between the two first best match
  // is over 0.8 we remove the match. Not enought discriminative
  for (int k = 0; k < knnMatches.size(); ++k)
  {
    if (knnMatches[k].size() == 2)
    {
      double dist1 = knnMatches[k][0].distance;
      double dist2 = knnMatches[k][1].distance;
      double ratio = dist1 / dist2;

      if (ratio < this->params.ratioMatchingThresh)
      {
        bad_matches.push_back(knnMatches[k][0]);
      }
    }
    else
    {
      bad_matches.push_back(knnMatches[k][0]);
    }
  }

  std::cout << "Calibrator : Raw keypoints match number : " << bad_matches.size() << std::endl;

  // Now we will filter the bad matches
  double max_dist = 0;
  double min_dist = 10000;
  for(int k = 0; k < bad_matches.size(); ++k)
  {
    double dist = bad_matches[k].distance;

    if (dist < min_dist)
    {
      min_dist = dist;
    }
    if (dist > max_dist)
    {
      max_dist = dist;
    }
  }

  // Sort by dist matching score
  std::vector<std::pair<double, int> > indexSorted;
  for (int k = 0; k < bad_matches.size(); ++k)
  {
    indexSorted.push_back(std::pair<double, int>(bad_matches[k].distance, k));
  }
  std::sort(indexSorted.begin(), indexSorted.end());

  int cpt = 0;

  // Keep only match when the dist is inferior
  // of twice the min distance
  for (int k = 0; k < bad_matches.size(); ++k)
  {
    double dist = bad_matches[indexSorted[k].second].distance;
    //std::cout << "k : " << k << " dist : " << dist << std::endl;
    if ((dist < 10 * min_dist) && (cpt < this->MaxMatchKeyPointNumber))
    {
      this->Matches.push_back(bad_matches[indexSorted[k].second]);
      cpt++;
    }
  }

  //std::cout << "max dist : " << max_dist << std::endl;
  //std::cout << "min dist : " << min_dist << std::endl;

  std::cout << "Calibrator : number of match after filtering: " << this->Matches.size() << std::endl;
}

//---------------------------------------------------------------------------
void Calibrator::ComputeEssentialMatrix()
{
  //Extract the points from keyPoints
  std::vector<cv::Point2f> scene;
  for(int i = 0 ; i < this->KeyPoints[0].first.size(); ++i)
  {
      scene.push_back(this->KeyPoints[0].first[i].pt);
  }

  // Epilines1 are the epilines of the first image
  // Epilines2 are the epilines of the second image
  std::vector<cv::Vec<float,3> > epilines;

  // Use the keyPoints of first frame to compute the epipolar
  // lines of the second frames usgin F. Remind, F is forward 1 -> 2
  // F.transpose is backward 2 -> 1
  cv::computeCorrespondEpilines(scene, 1, F, epilines);


  // We will look for the epicenter of the second image. This center is defined
  // as the intersection of all the epipolar lines.

  // epipolar center
  Eigen::Vector2d e;
  e << 0, 0;

  unsigned int count = 0;
  for (int k = 0; k < epilines.size() - 1; ++k)
  {
    // current epipolar center computed
    Eigen::Vector2d ec;

    // Linear part of the constraint
    Eigen::Matrix<double, 2, 2> A;

    // Affine part of the constraint
    Eigen::Vector2d y;

    A << epilines[k][0],     epilines[k][1],
         epilines[k + 1][0], epilines[k + 1][1];

    y << -epilines[k][2],
         -epilines[k + 1][2];

    // It is ok to call inverse method since A
    // is a 2x2 matrix. The computation time should
    // be short
    ec = A.inverse() * y;

    // reject the degenerated case
    if (std::isnan(ec(0)) || std::isnan(ec(1)))
    {

    }
    else
    {
      e += ec;
      count++;
    }
  }

  e /= static_cast<double>(count);

  Eigen::Vector3d homogeneousE;
  homogeneousE << e(0), e(1), 1;

  // represent the skew-symmetric matrix of e
  // in homogeneous coordinates
  Eigen::Matrix<double, 3, 3> vectE;
  vectE <<  0,   -1,   e(1),
            1,    0,  -e(0),
           -e(1), e(0), 0;

  // arbitrary vector
  Eigen::Vector3d v; v << 1, 1, 1;

  // arbitrary scalar
  double lambda = 1;

  Eigen::Matrix<double, 3, 3> leftPart = vectE * this->Fundamental + homogeneousE * v.transpose();

  this->Camera1 << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0;

  this->Camera2 << leftPart(0, 0), leftPart(0, 1), leftPart(0, 2), lambda * e(0),
                   leftPart(1, 0), leftPart(1, 1), leftPart(1, 2), lambda * e(1),
                   leftPart(2, 0), leftPart(2, 1), leftPart(2, 2), lambda;

  // Initialization of the cv matrix
  this->P1 = cv::Mat::zeros(3, 4, 5);
  this->P2 = cv::Mat::zeros(3, 4, 5);
  for (int i = 0; i < this->P1.rows; ++i)
  {
    for (int j = 0; j < this->P1.cols; ++j)
    {
      this->P1.at<float>(i, j) = this->Camera1(i, j);
      this->P2.at<float>(i, j) = this->Camera2(i, j);
    }
  }

  std::cout << "Epipolar center : \n" << e << std::endl;
  //std::cout << "Camera 2, eigen : \n" << this->Camera2 << std::endl;
  std::cout << "Camera 2 : \n" << this->P2 << std::endl;
}

//---------------------------------------------------------------------------
void Calibrator::ComputeEssentialMatrixPriorK(Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2)
{
  // Compute the essential matrix
  this->Essential = Kprior2.transpose() * this->Fundamental * Kprior1;

  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(this->Essential, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double singularValue = (svd.singularValues()(0) + svd.singularValues()(1)) / 2.0;
  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Matrix<double, 3, 3> V = svd.matrixV();
  Eigen::Matrix<double, 3, 3> D;
  D << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  std::cout << "Its singular values are:" << std::endl << D << std::endl;
  std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << U << std::endl;
  std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << V << std::endl;

  Eigen::Matrix<double, 3, 3> RecE = U * D * V.transpose();
  std::cout << "E : " << std::endl << this->Essential << std::endl;
  std::cout << "Reconstructed E : " << std::endl << RecE << std::endl;

  // Camera matrix 1
  this->Camera1 << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0;

  Eigen::Matrix<double, 3, 3> W;
  W << 0, -1, 0,
       1,  0, 0,
       0,  0, 1;

  this->R = U * W * V.transpose();
  this->R = std::cbrt(1.0 / this->R.determinant()) * this->R;

  this->T << U(0, 2), U(1, 2), U(2, 2);

  std::cout << "R det : " << R.determinant() << std::endl;
  std::cout << "R : " << std::endl << this->R << std::endl;
  std::cout << "T : " << std::endl << this->T << std::endl;

  this->Camera2 << this->R(0, 0), this->R(0, 1), this->R(0, 2), this->T(0),
                   this->R(1, 0), this->R(1, 1), this->R(1, 2), this->T(0),
                   this->R(2, 0), this->R(2, 1), this->R(2, 2), this->T(0);

  this->Camera1 = Kprior1 * this->Camera1;
  this->Camera2 = Kprior2 * this->Camera2;

  // Initialization of the cv matrix
  this->P1 = cv::Mat::zeros(3, 4, 5);
  this->P2 = cv::Mat::zeros(3, 4, 5);
  for (int i = 0; i < this->P1.rows; ++i)
  {
    for (int j = 0; j < this->P1.cols; ++j)
    {
      this->P1.at<float>(i, j) = this->Camera1(i, j);
      this->P2.at<float>(i, j) = this->Camera2(i, j);
    }
  }
}

//---------------------------------------------------------------------------
void Calibrator::SimulateTransform(Eigen::Matrix<double, 3, 3> Rc, Eigen::Matrix<double, 3, 1> Tc,
                                   Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2)
{
  Eigen::Matrix<double, 3, 3> newR = Rc * this->R;
  Eigen::Matrix<double, 3, 1> newT = Tc + this->T;

  Eigen::Matrix<double, 3, 3> skewT;
  skewT <<        0, -newT(2),  newT(1),
            newT(2),        0, -newT(0),
           -newT(1),  newT(0),        0;

  this->Essential = skewT * R;
  this->Fundamental = (Kprior2.transpose()).inverse() * this->Essential * Kprior1.inverse();
  for (int i = 0; i < F.rows; ++i)
  {
    for (int j = 0; j < F.cols; ++j)
    {
      F.at<double>(i, j) = this->Fundamental(i, j);
    }
  }

  std::cout << "newR : " << std::endl << newR << std::endl;
  std::cout << "newT : " << std::endl << newT << std::endl;
  std::cout << "newE : " << std::endl << this->Essential << std::endl;
  std::cout << "newF : " << std::endl << this->Fundamental << std::endl;
}

//---------------------------------------------------------------------------
void Calibrator::Calibrate()
{
  // Verify that we set the images
  bool CanLaunchAlgorithm = true;
  for (int k = 0; k < this->NumberCamera; ++k)
  {
    CanLaunchAlgorithm &= this->IsImageSet[k];
  }

  if (!CanLaunchAlgorithm)
  {
    std::cout << "Calibrator : Can't launch calibration, some images are missings" << std::endl;
    return;
  }

  // Launch the keyPoints computation
  this->ComputeKeyPoints();

  if (this->KeyPoints[0].first.size() == 0 || this->KeyPoints[1].first.size() == 0)
  {
    std::cout << "no keypoint found in images, this is there resolutions:" << std::endl;
    std::cout << "im1 : [" << this->Images[0].rows << "," << this->Images[0].cols << "]" << std::endl;
    std::cout << "im2 : [" << this->Images[1].rows << "," << this->Images[1].cols << "]" << std::endl;
    return;
  }

  // Launch the matching computation
  this->MatchKeyPoints();

  cv::Mat MaskF, MaskH; // Mask of ransac computing

  // transform the matched keypoints into vector of Point2f
  std::vector<cv::Point2f> keyPoints1; // key point of the first view
  std::vector<cv::Point2f> keyPoints2; // key point of the second view
  int N = this->Matches.size();

  for (int k = 0; k < N; ++k)
  {
    /*std::cout << "Number match : " << N << " idx1 : "
    << this->CalibrationMatches[idxCamera][idxImage][k].trainIdx
    << " idx2 : " << this->CalibrationMatches[idxCamera][idxImage][k].queryIdx << std::endl;*/
    keyPoints1.push_back(this->KeyPoints[0].first[this->Matches[k].queryIdx].pt);
    keyPoints2.push_back(this->KeyPoints[1].first[this->Matches[k].trainIdx].pt);
  }

  // Parameters for finding the fundamental matrix
  // param1 determines the maximum distance between a pixel and its epipolar line
  // to be considered as an inlier. It depends on the noise and the image resolution.
  // Set to 3 for high res images
  this->params.param1;

  // Specify a desirable level of confidence that the estimated matrix is correct
  // 99% is the recommanded value by opencv documentation
  this->params.param2;

  if (keyPoints1.size() < 7 || keyPoints2.size() < 7)
  {
    std::cout << "No keypoints found, calibration on this image pair not processed" << std::endl;
    return;
  }

  // Compute the RANSAC to find the fundamental matrix
  //cv::Mat H = cv::findHomography(keyPoints1, keyPoints2, CV_RANSAC, 25, MaskH, 25000, 0.9995);
  this->F = cv::findFundamentalMat(keyPoints1, keyPoints2, MaskF, cv::FM_RANSAC, this->params.param1, this->params.param2);

  //std::cout << "Mask matrix size : " << MaskF.size() << std::endl;

  for (int k = 0; k < MaskF.rows; ++k)
  {
    int value = static_cast<int>(MaskF.at<uchar>(k, 0));
    if (value == 1)
    {
      this->GoodMatches.push_back(this->Matches[k]);

      this->MatchedKeypoints[0].push_back(this->KeyPoints[0].first[this->Matches[k].queryIdx]);
      this->MatchedKeypoints[1].push_back(this->KeyPoints[1].first[this->Matches[k].trainIdx]);
    }
  }

  std::cout << "Number inliers : " << this->GoodMatches.size() << std::endl;

  for (int i = 0; i < F.rows; ++i)
  {
    for (int j = 0; j < F.cols; ++j)
    {
      this->Fundamental(i, j) = F.at<double>(i, j);
    }
  }

  //std::cout << "Fundamental, eigen : \n" << this->Fundamental << std::endl;
  std::cout << "Fundamental : \n" << this->F << std::endl;

  /*if (this->UsePrior)
  {
    Eigen::Matrix<double, 3, 3> Kprior1;
    Kprior1 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    double theta = 0.95993108859; // 55 degre
    double w = static_cast<double>(this->Images[0].cols);
    double h = static_cast<double>(this->Images[0].rows);
    double f = w / (2 * std::tan(theta / 2.0));
    Kprior1(0, 0) = f;
    Kprior1(1, 1) = f;
    Kprior1(2, 2) = 1;
    Kprior1(0, 2) = w / 2.0;
    Kprior1(1, 2) = h / 2.0;

    Eigen::Matrix<double, 3, 3> Kprior2;
    Kprior2 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    theta = 0.95993108859; // 55 degre
    w = static_cast<double>(this->Images[1].cols);
    h = static_cast<double>(this->Images[1].rows);
    f = w / (2 * std::tan(theta / 2.0));
    Kprior2(0, 0) = f;
    Kprior2(1, 1) = f;
    Kprior2(2, 2) = 1;
    Kprior2(0, 2) = w / 2.0;
    Kprior2(1, 2) = h / 2.0;
    std::cout << "Kprior : " << std::endl << Kprior2 << std::endl;
    this->ComputeEssentialMatrixPriorK(Kprior1, Kprior2);

    theta = 0.43633231299; // 25 degre
    Eigen::Matrix<double, 3, 3> Rc;
    Rc << std::cos(theta), 0, -std::sin(theta),
          0,               1,                0,
          std::sin(theta), 0,  std::cos(theta);

    Eigen::Matrix<double, 3, 1> Tc;
    Tc << 0, 0, 0;

    this->SimulateTransform(Rc, Tc, Kprior1, Kprior2);
  }
  else
  {
    this->ComputeEssentialMatrix();
    this->ExtractExtrinsecParameters();
  }*/
}

//---------------------------------------------------------------------------
std::vector<Eigen::Matrix<float, 3, 1> > Calibrator::Triangulate(std::vector<Eigen::Matrix<float, 2, 1> > pt1,
                                                      std::vector<Eigen::Matrix<float, 2, 1> > pt2)
{
  std::vector<Eigen::Matrix<float, 3, 1> > X;
  std::vector<cv::Point2f> u;
  std::vector<cv::Point2f> v;

  for (int k = 0; k < pt1.size(); ++k)
  {
    cv::Point2f temp;
    temp.x = pt1[k](0);
    temp.y = pt1[k](1);
    u.push_back(temp);

    //std::cout << "pt1 : [" << u[u.size() - 1].x << ";" << u[u.size() - 1].y << "]" << std::endl;

    temp.x = pt2[k](0);
    temp.y = pt2[k](1);
    v.push_back(temp);

    //std::cout << "pt2 : [" << v[v.size() - 1].x << ";" << v[v.size() - 1].y << "]" << std::endl;
  }

  /*std::cout << "P1 : " << std::endl;
  std::cout << this->P1 << std::endl;
  std::cout << std::endl;

  std::cout << "P2 : " << std::endl;
  std::cout << this->P2 << std::endl;
  std::cout << std::endl;*/

  cv::Mat Res; // float type 5

  // Triangulate the points
  cv::triangulatePoints(this->P1, this->P2, u, v, Res);

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

  /*std::cout << "P1 type : " << this->P1.type() << std::endl;
  std::cout << "Res type : " << Res.type() << std::endl;*/
  return X;
}

//---------------------------------------------------------------------------
std::vector<Eigen::Matrix<float, 3, 1> > Calibrator::Triangulate2(std::vector<Eigen::Matrix<float, 2, 1> > pt1,
                                                      std::vector<Eigen::Matrix<float, 2, 1> > pt2)
{
  Eigen::Matrix<double, 3, 4> D1 = this->Camera1;
  Eigen::Matrix<double, 3, 4> D2 = this->Camera2;

  std::vector<Eigen::Matrix<float, 3, 1> > X;

  for (int k = 0; k < pt1.size(); ++k)
  {
    // Matrix which contains the two linear constraints :
    // u = P * X and u' = P' * X with (P, P') cameras matrix
    // (u, u') matched pair of points and X the 3D points
    Eigen::MatrixXf A;
    A.resize(4, 4);

    Eigen::Matrix<float, 2, 1> u1 = pt1[k];
    Eigen::Matrix<float, 2, 1> u2 = pt2[k];

    A << u1(0) * D1(2, 0) - D1(0, 0), u1(0) * D1(2, 1) - D1(0, 1), u1(0) * D1(2, 2) - D1(0, 2), u1(0) * D1(2, 3) - D1(0, 3),
         u1(1) * D1(2, 0) - D1(1, 0), u1(1) * D1(2, 1) - D1(1, 1), u1(1) * D1(2, 2) - D1(1, 2), u1(1) * D1(2, 3) - D1(1, 3),
         u2(0) * D2(2, 0) - D2(0, 0), u2(0) * D2(2, 1) - D2(0, 1), u2(0) * D2(2, 2) - D2(0, 2), u2(0) * D2(2, 3) - D2(0, 3),
         u2(1) * D2(2, 0) - D2(1, 0), u2(1) * D2(2, 1) - D2(1, 1), u2(1) * D2(2, 2) - D2(1, 2), u2(1) * D2(2, 3) - D2(1, 3);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    /*std::cout << "A:" << std::endl << A << std::endl;
    std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
    std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;*/

    Eigen::MatrixXf V = svd.matrixV();

    Eigen::Matrix<float, 3, 1> CurrentPoint;
    CurrentPoint << V(0, 3) / V(3, 3), V(1, 3) / V(3, 3), V(2, 3) / V(3, 3);
    X.push_back(CurrentPoint);
  }

  return X;
}

//---------------------------------------------------------------------------
Eigen::Matrix<double, 3, 3> Calibrator::GetFundamental()
{
  return this->Fundamental;
}

//---------------------------------------------------------------------------
Eigen::Matrix<double, 3, 3> Calibrator::GetE()
{
  return this->Essential;
}

//---------------------------------------------------------------------------
Eigen::Matrix<double, 3, 4> Calibrator::GetP(int idxImage)
{
  if (idxImage == 0)
  {
    return this->Camera1;
  }
  else
  {
    return this->Camera2;
  }
}

//---------------------------------------------------------------------------
void Calibrator::ShowCalibrationResult()
{
  // Create images to show the result : epipolar lines, matching, ...
  //-----------------------------------------------------
  cv::Mat Im1, Im2, Im1Key, Im2Key;
  this->GetImage(0).copyTo(Im1);
  this->GetImage(1).copyTo(Im2);
  Im1.copyTo(Im1Key);
  Im2.copyTo(Im2Key);

  cv::Mat F = this->GetF();

  DrawCircle(Im1Key, this->GetKeyPoint(0));
  DrawCircle(Im2Key, this->GetKeyPoint(1));

  cv::Mat BadMatchIm = DrawMatch(Im1, Im2,
                              this->GetKeyPoint(0), this->GetKeyPoint(1),
                              this->GetMatches());

  // The matcher is a list of corresponding index. That is why we set as input
  // GetKeyPoint and not GetMatchedKeyPoint. Since the matcher know which points are
  // matched in the GetKeyPoint list
  cv::Mat GoodMatchIm = DrawMatch(Im1, Im2,
                              this->GetKeyPoint(0), this->GetKeyPoint(1),
                              this->GetGoodMatches());

  std::pair<cv::Mat, cv::Mat> EpipolarLinesIm = DrawEpipolarLines(Im1, Im2,
                                                                  this->GetMatchedKeyPoint(0), this->GetMatchedKeyPoint(1),
                                                                  F, this->GetGoodMatches());

  // displayed information
  DisplayImage(Im1Key, "I1");
  DisplayImage(Im2Key, "I2");
  DisplayImage(BadMatchIm, "BadMatchIm");
  DisplayImage(GoodMatchIm, "GoodMatchIm");
  DisplayImage(EpipolarLinesIm.first, "Epipolar lines Im1");
  DisplayImage(EpipolarLinesIm.second, "Epipolar lines Im2");
}

//-----------------------------------------------------
std::vector<cv::Mat> Calibrator::GetCalibrationResults()
{
  // Create images to show the result : epipolar lines, matching, ...
  //-----------------------------------------------------
  cv::Mat Im1, Im2, Im1Key, Im2Key;
  this->GetImage(0).copyTo(Im1);
  this->GetImage(1).copyTo(Im2);
  Im1.copyTo(Im1Key);
  Im2.copyTo(Im2Key);

  cv::Mat F = this->GetF();

  DrawCircle(Im1Key, this->GetKeyPoint(0));
  DrawCircle(Im2Key, this->GetKeyPoint(1));

  cv::Mat BadMatchIm = DrawMatch(Im1, Im2,
                              this->GetKeyPoint(0), this->GetKeyPoint(1),
                              this->GetMatches());

  // The matcher is a list of corresponding index. That is why we set as input
  // GetKeyPoint and not GetMatchedKeyPoint. Since the matcher know which points are
  // matched in the GetKeyPoint list
  cv::Mat GoodMatchIm = DrawMatch(Im1, Im2,
                              this->GetKeyPoint(0), this->GetKeyPoint(1),
                              this->GetGoodMatches());

  std::pair<cv::Mat, cv::Mat> EpipolarLinesIm = DrawEpipolarLines(Im1, Im2,
                                                                  this->GetMatchedKeyPoint(0), this->GetMatchedKeyPoint(1),
                                                                  F, this->GetGoodMatches());

  std::vector<cv::Mat> Result;
  Result.push_back(Im1Key);
  Result.push_back(Im2Key);
  Result.push_back(BadMatchIm);
  Result.push_back(GoodMatchIm);
  Result.push_back(EpipolarLinesIm.first);
  Result.push_back(EpipolarLinesIm.second);

  return Result;
}

//-----------------------------------------------------
void Calibrator::ExtractExtrinsecParameters()
{
  if(!this->F.data)
  {
    std::cout << "Calibrator extract extrinsec : Compute the calibration first" << std::endl;
  }

  // Extraction of intresecs parameters
  // 3x3 matrix composed of the left side of P
  // since P = K[R | t], Pleft = KR
  Eigen::Matrix<double, 3, 3> Pleft;
  Pleft << this->Camera2(0, 0), this->Camera2(0, 1), this->Camera2(0, 2),
           this->Camera2(1, 0), this->Camera2(1, 1), this->Camera2(1, 2),
           this->Camera2(2, 0), this->Camera2(2, 1), this->Camera2(2, 2);

  // since Pl = KR, Pl*Plt = KRRtKt. But, R is orthogonal matrix
  // hence, RRt = I and PlPlt = KKt
  Eigen::Matrix<double, 3, 3> KKt = Pleft * Pleft.transpose();

  // We compute the cholesky of KKt to have only K
  this->K = Eigen::Matrix<double, 3, 3>(KKt.llt().matrixL().transpose());

  // Normalise K
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      K(i, j) = K(i, j) / K(2, 2);
    }
  }

  Eigen::Matrix<double, 3, 3> Kinv(K.inverse());
  Eigen::Matrix<double, 3, 4> RT = Kinv * this->Camera2;
  this->R << RT(0, 0), RT(0, 1), RT(0, 2),
             RT(1, 0), RT(1, 1), RT(1, 2),
             RT(2, 0), RT(2, 1), RT(2, 2);

  // Since R is orthogonal, we must normalise the determinant
  double det = R.determinant();
  double lambda = std::cbrt(1.0 / det);
  R = lambda * R;

  // Now we get T
  RT = lambda * RT;
  Eigen::Matrix<double, 3, 1> negRT;
  negRT << RT(0, 3), RT(1, 3), RT(2, 3);
  this->T = - R.inverse() * negRT;

  double PI = 3.14159265359;
  double phi = std::atan2(R(2, 1), R(2, 2)) / PI * 180;
  double theta = - std::asin(R(2, 0)) / PI * 180;
  double psi = - std::atan2(R(1, 0), R(0, 0)) / PI * 180;

  this->Angles << phi, theta, psi;

  std::cout << "K : " << std::endl;
  std::cout << K << std::endl;
  std::cout << "R det : " << R.determinant() << std::endl;
  std::cout << "lambda : " << lambda << std::endl;
  std::cout << R << std::endl;
  std::cout << "[phi, theta, psi] : [" << phi << "," << theta << "," << psi << "]" << std::endl;
  std::cout << "T : " << std::endl;
  std::cout << T << std::endl;
}
