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
 * @brief  Class which permits a manual matching of 2D points belonging
 * to two differents images. The matching pipeline is using openCV functions
 *
*/

#ifndef MANUALMATCHER_H
#define MANUALMATCHER_H

// STD
#include <iostream>
#include <stdlib.h>

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// EIGEN
#include <Eigen/Dense>

class ManualMatcher
{
public:

  // Constructor
  ManualMatcher(cv::Mat Im1, cv::Mat Im2);

  // Launch the matching pipeline
  void LaunchMatchingPipeline(unsigned int nbrMatch);

  // Save the matched points
  void SavePoints(std::string filename);

  // Get the matched points saved in a file
  void ReadMatchFromFile(std::string filename, std::vector<Eigen::Matrix<float, 2, 1>> *matchedPoints);

  // Get the selected points saved in a file
  std::vector<Eigen::Matrix<float, 2, 1>> ReadSelectedPointsFromFile(std::string filename);

  // Select points from an image and save them
  void SelectPointsAndSave(std::string filename, unsigned int nbrPts, unsigned int idxIm);

  // Read a selection of points over the frames of a video
  std::vector<Eigen::Matrix<float, 2, 1> > ReadSelectedPointsVideo(std::string filename);

  // Get the matched points
  std::vector<std::pair<cv::Point2f, cv::Point2f> > GetMatchedPoints();

protected:
  // Images used to make the manual match
  std::vector<cv::Mat> Images;

  // List of manualy matched points
  std::vector<std::pair<cv::Point2f, cv::Point2f> > X;
};

#endif
