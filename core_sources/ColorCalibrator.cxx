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

#include "ColorCalibrator.h"

//---------------------------------------------------------------------------
ColorCalibrator::ColorCalibrator()
{
  this->NQuantum = 256;
  this->H.resize(this->NQuantum, 0);
  this->WindowSize = 15;
  this->Moy = 0;
  this->Sigma = 0;
  this->HList.resize(this->WindowSize, std::vector<int>(this->NQuantum, 0));
}

//---------------------------------------------------------------------------
void ColorCalibrator::ResetCalibrator()
{
  this->Moy = 0;
  this->Sigma = 0;

  for (int k = 0; k < this->NQuantum; ++k)
  {
    this->H[k] = 0;
    for (int i = 0; i < this->WindowSize; ++i)
    {
      this->HList[i][k] = 0;
    }
  }

  this->PosPointer = 0;
}

//---------------------------------------------------------------------------
void ColorCalibrator::SetWindowSize(int wsz)
{
  this->WindowSize = wsz;
  this->HList.resize(this->WindowSize, std::vector<int>(this->NQuantum, 0));
}

//---------------------------------------------------------------------------
void ColorCalibrator::AddImage(cv::Mat& newInput)
{
  // first remove the current histogram of the cumulated histogram
  // and reset its value to zero
  for (int k = 0; k < this->NQuantum; ++k)
  {
    this->H[k] -= this->HList[this->PosPointer][k];
    this->HList[this->PosPointer][k] = 0;
  }

  // Populate the histogramme corresponding of the current pos
  double brightness = 0;
  for (int i = 0; i < newInput.rows; ++i)
  {
    for (int j = 0; j < newInput.cols; ++j)
    {
      cv::Vec3b colour = newInput.at<cv::Vec3b>(i, j);
      // compute the bright
      brightness = (299.0 * colour[0] + 587.0 * colour[1] + 114.0 * colour[2]) / 1000.0;
      HList[this->PosPointer][static_cast<int>(brightness)]++;
    }
  }

  this->NbrSample = 0;
  this->Moy = 0;
  this->Sigma = 0;

  // Now, add the current histogram into the cumulated one
  for (int k = 0; k < this->NQuantum; ++k)
  {
    this->H[k] += this->HList[this->PosPointer][k];
    this->NbrSample += this->H[k];
    this->Moy += k * this->H[k];
  }

  this->Moy /= 1.0 * this->NbrSample;

  for (int k = 0; k < this->NQuantum; ++k)
  {
    this->Sigma += (1.0 * k - this->Moy) * (1.0 * k - this->Moy) * this->H[k];
  }

  this->Sigma = std::sqrt(this->Sigma / (1.0 * this->NbrSample));

  this->PosPointer++;
  this->PosPointer = this->PosPointer % this->WindowSize;
}

//---------------------------------------------------------------------------
void ColorCalibrator::MatchTargetCalibration(cv::Mat& src, cv::Mat& dst, double moySrc, double sigmaSrc)
{
  dst = cv::Mat::zeros(src.size(), src.type());

  double a = sigmaSrc / this->Sigma;
  double b = moySrc - a * this->Moy;

  for (int i = 0; i < dst.rows; ++i)
  {
    for (int j = 0; j < dst.cols; ++j)
    {
      for (int c = 0; c < 3; ++c)
      {
        dst.at<cv::Vec3b>(i, j)[c] =
          cv::saturate_cast<uchar>(a * src.at<cv::Vec3b>(i, j)[c] + b);
      }
    }
  }
}

//---------------------------------------------------------------------------
double ColorCalibrator::GetMoy()
{
  return this->Moy;
}

//---------------------------------------------------------------------------
double ColorCalibrator::GetSigma()
{
  return this->Sigma;
}