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


// LOCAL
#include "ManualMatcher.h"

// STD
#include <iostream>
#include <fstream>
#include <string>

//---------------------------------------------------------------------------
static void onMouse( int event, int x, int y, int, void* dataUser)
{
  cv::Point2f* X = static_cast<cv::Point2f*>(dataUser);
  if( event == cv::EVENT_FLAG_LBUTTON )
  {
    std::cout << "x : " << x << " ; y : " << y << std::endl;
    X->x = x;
    X->y = y;
  }
}

//---------------------------------------------------------------------------
ManualMatcher::ManualMatcher(cv::Mat Im1, cv::Mat Im2)
{
  this->Images.push_back(Im1);
  this->Images.push_back(Im2);
}

//---------------------------------------------------------------------------
void ManualMatcher::SelectPointsAndSave(std::string filename, unsigned int nbrPts, unsigned int idxIm)
{
  std::vector<cv::Point2f> list;
  // we will select the points
  for (int k = 0; k < nbrPts; ++k)
  {
    std::cout << "Iteration : " << k + 1 << std::endl;
    cv::Point2f Pt1;

    // create the window
    cv::namedWindow("Image", cv::WINDOW_NORMAL);

    // display the image
    cv::imshow("Image", this->Images[idxIm]);

    // create a mouse callback to get the clicked point
    cv::setMouseCallback("Image", onMouse, static_cast<void*>(&Pt1));

    // wait the user to push a keyboard button
    cv::waitKey(0);

    std::cout << "Points got : [" << Pt1.x << ";" << Pt1.y << "]" << std::endl;
    list.push_back(Pt1);
  }

  std::ofstream file;
  file.open(filename.c_str(), std::ios::out | std::ios::app | std::ios::binary);

  if (file.is_open())
  {
    file << "!Points belonging to first Image : [x, y]" << std::endl;
    for (int k = 0; k < list.size(); ++k)
    {
      file << list[k].x << " " << list[k].y << std::endl;
    }
    file << "#############" << std::endl;
  }

  file.close();
}

//---------------------------------------------------------------------------
std::vector<Eigen::Matrix<float, 2, 1>> ManualMatcher::ReadSelectedPointsFromFile(std::string filename)
{
  std::vector<Eigen::Matrix<float, 2, 1>> selectedPoints;
  std::ifstream file;
  file.open(filename.c_str());
  std::string line;

  int nbrIm = 0;

  if (file.is_open())
  {
    while (std::getline(file, line))
    {
      if (line.c_str()[0] != '!')
      {
        // changing the image
        if (line.c_str()[0] == '#')
        {
          break;
        }
        else
        {
          std::string::size_type sz;
          Eigen::Vector2f pt;

          // Get the x coordinate
          pt(0) = std::stof(line, &sz);

          // Get the y coordinate
          pt(1) = std::stof(line.substr(sz));

          selectedPoints.push_back(pt);
        }
      }
    }
  }

  return selectedPoints;
}

//---------------------------------------------------------------------------
std::vector<Eigen::Matrix<float, 2, 1> > ManualMatcher::ReadSelectedPointsVideo(std::string filename)
{
  std::vector<Eigen::Matrix<float, 2, 1>> selectedPoints;
  std::ifstream file;
  file.open(filename.c_str());
  std::string line;

  int nbrIm = 0;

  if (file.is_open())
  {
    while (std::getline(file, line))
    {
          std::string::size_type sz1;
          std::string::size_type sz2;
          Eigen::Vector2f pt;

          // Get the number of frame
          int i = std::stoi(line, &sz1);

          // Get the x coordinate
          pt(0) = std::stof(line.substr(sz1), &sz2);

          // Get the y coordinate
          pt(1) = std::stof(line.substr(sz1 + sz2));


          selectedPoints.push_back(pt);
    }
  }

  return selectedPoints;
}

//---------------------------------------------------------------------------
void ManualMatcher::LaunchMatchingPipeline(unsigned int nbrMatch)
{
  // we will match nbrMatch points
  for (int k = 0; k < nbrMatch; ++k)
  {
    std::string stringIteration = std::to_string(k);
    std::string leftName = "Please, click on the point and press space key for : Image left, point : " + stringIteration;
    std::string rightName = "Please, click on the point and press space key for : Image right, point : " + stringIteration;
    std::cout << "Iteration : " << k + 1 << std::endl;
    cv::Point2f Pt1, Pt2;

    // create the window
    cv::namedWindow(leftName.c_str(), cv::WINDOW_NORMAL);

    // display the image
    cv::imshow(leftName.c_str(), this->Images[0]);

    // create a mouse callback to get the clicked point
    cv::setMouseCallback(leftName.c_str(), onMouse, static_cast<void*>(&Pt1));

    // wait the user to push a keyboard button
    cv::waitKey(0);

    cv::destroyWindow(leftName.c_str());

    // Same thing image 2
    cv::namedWindow(rightName.c_str(), cv::WINDOW_NORMAL);
    cv::imshow(rightName.c_str(), this->Images[1]);
    cv::setMouseCallback(rightName.c_str(), onMouse, static_cast<void*>(&Pt2));
    cv::waitKey(0);

    cv::destroyWindow(rightName.c_str());

    std::cout << "Points got : [" << Pt1.x << ";" << Pt1.y << "] - [" << Pt2.x << ";" << Pt2.y << "]" << std::endl;

    std::pair<cv::Point2f, cv::Point2f> temp(Pt1, Pt2);
    this->X.push_back(temp);
  }
}

//---------------------------------------------------------------------------
void ManualMatcher::SavePoints(std::string filename)
{
  std::ofstream file;
  file.open(filename.c_str(), std::ios::out | std::ios::app | std::ios::binary);

  if (file.is_open())
  {
    file << "!Points belonging to first Image : [x, y]" << std::endl;
    for (int k = 0; k < this->X.size(); ++k)
    {
      file << this->X[k].first.x << " " << this->X[k].first.y << std::endl;
    }
    file << "#############" << std::endl;
    file << "!Points belonging to second Image : [x, y]" << std::endl;
    for (int k = 0; k < this->X.size(); ++k)
    {
      file << this->X[k].second.x << " " << this->X[k].second.y << std::endl;
    }
  }

  file.close();
}

//---------------------------------------------------------------------------
std::vector<std::pair<cv::Point2f, cv::Point2f> > ManualMatcher::GetMatchedPoints()
{
  return this->X;
}

//---------------------------------------------------------------------------
void ManualMatcher::ReadMatchFromFile(std::string filename, std::vector<Eigen::Matrix<float, 2, 1>>* matchedPoints)
{
  std::ifstream file;
  file.open(filename.c_str());
  std::string line;

  int nbrIm = 0;

  if (file.is_open())
  {
    while (std::getline(file, line))
    {
      if (line.c_str()[0] != '!')
      {
        // changing the image
        if (line.c_str()[0] == '#')
        {
          nbrIm++;
        }
        else
        {
          std::string::size_type sz;
          Eigen::Vector2f pt;

          // Get the x coordinate
          pt(0) = std::stof(line, &sz);

          // Get the y coordinate
          pt(1) = std::stof(line.substr(sz));

          matchedPoints[nbrIm].push_back(pt);
        }
      }
    }
  }
}
