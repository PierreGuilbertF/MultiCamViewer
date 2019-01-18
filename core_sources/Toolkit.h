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
 * @brief   Different tools to manipulate openCV functions and display
 * the results.
 *
*/

#ifndef TOOLKIT_H
#define TOOLKIT_H

// STD
#include <iostream>
#include <stdlib.h>

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// EIGEN
#include <Eigen/Dense>

// Open the image located by filename. If the filename
// is incorrect warning message will be generated and the
// result will be a zeros matrix
cv::Mat OpenImage(std::string filename);

// Display the image given in argument in a window named
// by the string given in argument
void DisplayImage(const cv::Mat& Im, std::string windowName);

// draw circles at the localization of keyPoints
void DrawCircle(cv::Mat& Im, std::vector<cv::KeyPoint> pts);

// draw epipolar lines on the images
std::pair<cv::Mat, cv::Mat> DrawEpipolarLines(cv::Mat& Im1, cv::Mat& Im2, std::vector<cv::KeyPoint> pts1,
                       std::vector<cv::KeyPoint> pts2, cv::Mat& F, std::vector<cv::DMatch> matches);

// draw the matching keyPoints between the images Im1 and Im2
cv::Mat DrawMatch(cv::Mat& Im1, cv::Mat& Im2, std::vector<cv::KeyPoint> pts1,
                  std::vector<cv::KeyPoint> pts2, std::vector<cv::DMatch> matches);

// Compute the depth map of a scene using two rectified images. The depth map is built
// using the disparity assuming a focal and a baseline length
cv::Mat ComputeDepthMapUncalibrated(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY);

// Find best match of a cropped image in a target image
std::pair<double, double> FindBestMatchOfCroppedIm(cv::Mat croppedIm, cv::Mat target, int blockSizeX, int blockSizeY, int i, int j, int maxDisparity);

// Compute the sum square disctance of two images
double SumSquareDistance(cv::Mat im1, cv::Mat im2);

// Using SGBM
cv::Mat ComputeDepthMapUncalibrated2(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY);

// Using BM
cv::Mat ComputeDepthMapUncalibrated3(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY);

// Convert a vector of non-homogeneous 2D points to a vector of homogenehous 2D points.
void to_homogeneous(const std::vector< cv::Point2f >& non_homogeneous, std::vector< cv::Point3f >& homogeneous);

// Convert a vector of homogeneous 2D points to a vector of non-homogenehous 2D points.
void from_homogeneous(const std::vector< cv::Point3f >& homogeneous, std::vector< cv::Point2f >& non_homogeneous);

// Transform a vector of 2D non-homogeneous points via an homography.
std::vector<cv::Point2f> transform_via_homography(const std::vector<cv::Point2f>& points, const cv::Matx33f& homography);

// Find the bounding box of a vector of 2D non-homogeneous points.
cv::Rect_<float> bounding_box(const std::vector<cv::Point2f>& p);

// Correct the brightness so that the histogram is centered and the standard
// deviation is maximised according to a maximum number of saturation determined by the threshold
void BrightnessRescale(cv::Mat& src, cv::Mat& dst, double saturationThreshold);

// Correct the brightness so that the histogram is centered and the standard
// deviation is maximised according to a maximum number of saturation determined by the threshold
void BrightnessRescaleBimodal(cv::Mat& src, cv::Mat& dst, double saturationThresholdLeft, double saturationThresholdRight);

// Correct the brightness so that the histogram is centered and the standard
// deviation is maximised according to a maximum number of saturation determined by the threshold
void BrightnessCalibration(cv::Mat& src, cv::Mat& target, cv::Mat& dst, double saturationThreshold);

// Warp the image src into the image dst through the homography H.
// The resulting dst image contains the entire warped image, this
// behaviour is the same of Octave's imperspectivewarp (in the 'image'
// package) behaviour when the argument bbox is equal to 'loose'.
// See http://octave.sourceforge.net/image/function/imperspectivewarp.html
std::vector< cv::Point2f > homography_warp(const cv::Mat& src, const cv::Mat& H, cv::Mat& dst);

// White balance using the grey assumption
void WhiteBalanceGreyAssumption(cv::Mat& src, cv::Mat& dst, double saturationRatio);

// Compute the interior rectangle of a warped image
cv::Rect InteriorRectangleOfWarpedImage(std::vector< cv::Point2f > input, double aspectRatio);

// Compute the interior rectangle of a convex polygone
double DiagLengthRectangleInsidePolygone(std::vector< cv::Point2f > input, double aspectRatio);

// Keep the smallest rectangle
std::pair<cv::Rect, cv::Rect> KeepSmallerRectangle(std::vector< cv::Point2f > inputL, std::vector< cv::Point2f > inputR, double yL, double yR, double aspectRatioR, double aspectRatioL);

// return a centered rectangle of the required dimension
cv::Rect CenteredRectangle(std::vector<cv::Point2f> input, int H, int W);
#endif
