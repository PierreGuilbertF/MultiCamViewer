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
 * @brief   Numerical Analysis Tools
 *
*/
#ifndef NUMERICALTOOLS_H
#define NUMERICALTOOLS_H

// STD
#include <iostream>
#include <stdlib.h>
#include <vector>

// Eigen
#include <Eigen/Dense>

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Compute plane which best approximate the input point cloud (mean square)
Eigen::Matrix<float, 3, 3> GetBestPlane(std::vector<Eigen::Matrix<float, 3, 1> > Points, std::vector<float> Weight);

// Compute the intersection between the line [vect | point] and the plane [vect1 | vect2 | point]
Eigen::Matrix<float, 3, 1> IntersectionLinePlane(Eigen::Matrix<float, 3, 2> line, Eigen::Matrix<float, 3, 3> plane);

// Compute the mapping homography between two images knowing their camera matrix and the plane [vect1 | vect2 | point]
Eigen::Matrix<float, 3, 3> ComputeHomographyFromPlane(Eigen::Matrix<float, 3, 3> plane, Eigen::Matrix<float, 3, 4> P);

// Get the temporal shift between two temporal serie
std::pair<double, double> TimeSynchronization(double* data1, double* data2, int size1, int size2, int sampleRate);

// Compute a distance between two quadrilateral using area(AuB) / area(AnB)
double PolygonMetric(std::vector<Eigen::Vector2f> P1, std::vector<Eigen::Vector2f> P2);

// Compute the intersection between two convex polygon
std::vector<Eigen::Vector2f> ConvexPolygonIntersection(std::vector<Eigen::Vector2f> P1, std::vector<Eigen::Vector2f> P2);

// Compute the area of a polygon
double ComputeAreaConvexPolygon(std::vector<Eigen::Vector2f> P);

// Compute the relative position information from fundamental matrix
Eigen::Matrix<double, 3, 3> GetPositionInformationFromFund(Eigen::Matrix<double, 3, 3> Fund, Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2);
#endif
