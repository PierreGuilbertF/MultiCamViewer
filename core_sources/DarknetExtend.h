/*=========================================================================

  Program:   Visualization Toolkit
  Module:    DarknetExtend.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @brief   Extend some function to manipulate darknet
 *
 * Calibrator is a convenience class for calibrating two or more camera.
 * It gives the intrinsecs and extrinsecs parameters of each camera relatively
 * to the first camera's camera frames
 *
*/
#ifndef DARKNETEXTEND_H
#define DARKNETEXTEND_H

// STD
#include <iostream>
#include <stdlib.h>

// EIGEN
#include <eigen3/Eigen/Dense>

// LOCAL
#include "Calibrator.h"
#include "ManualMatcher.h"
#include "NumericalAnalysisTools.h"
#include "Toolkit.h"

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// YOLO
extern "C"
{
#include "../darknet_src/darknet.h"
}

struct BoundedBox
{
  int Left;
  int Right;
  int Top;
  int Bot;
};

std::vector<BoundedBox> GetRegionOfInterestYolo(image im, int num, float thresh, box *boxes, float **probs, float **masks, char **names, image **alphabet, int classes);

std::pair<std::vector<BoundedBox>, std::vector<BoundedBox> > LaunchDarknetAndGetROI(std::string filename1, std::string filename2, std::string weightFilename,
                            std::string yoloConfigFilename, std::string yoloConfigFilename2, std::string yoloLabelsFilename, double thresh);

#endif
