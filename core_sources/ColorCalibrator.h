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
* @class   ColorCalibrator
* @brief   Calibrate color
*
*/

// EIGEN
#include <Eigen/Dense>

// OPENCV
#include <opencv2/core/core.hpp>

// LOCAL
#include "Toolkit.h"

// STD
#include <iostream>

class ColorCalibrator
{
public:
  ColorCalibrator();

  // Add a new image to populate the histograms
  void AddImage(cv::Mat& newInput);

  // Reset the calibrator data
  void ResetCalibrator();

  // Set the window size to smooth the calibration
  void SetWindowSize(int wsz);

  // Compute the coeff a and b so that a*h+b mathch the given moy and sigma
  void MatchTargetCalibration(cv::Mat& src, cv::Mat& dst, double moySrc, double sigmaSrc);

  // Get Moy
  double GetMoy();

  // Get sigma
  double GetSigma();

protected:
  // Size of the windows
  unsigned int WindowSize;

  // Vector of histogrammes
  std::vector<std::vector<int> > HList;

  // Histogramme of the WindowsSize images added to the calibrator
  std::vector<int> H;

  // Nbr of sample contained in H
  unsigned int NbrSample;

  // Nbr of quantum in the images (8bit -> 256 values)
  int NQuantum;

  // Indicates the current position in the rolling histogramme list
  int PosPointer;

  // average value of the cumumlated histogram
  double Moy;

  // standard deviation of the cumulated histogram
  double Sigma;
};