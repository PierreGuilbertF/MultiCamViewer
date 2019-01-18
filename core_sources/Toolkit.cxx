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
 * @brief   Different tools to manipulate opencv classes
 *
*/

// LOCAL
#include "Toolkit.h"

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

//---------------------------------------------------------------------------
cv::Mat OpenImage(std::string filename)
{
  // Try to open the image
  cv::Mat result = cv::imread(filename, CV_LOAD_IMAGE_COLOR);

  // if the image is not opened
  if (!result.data)
  {
    std::cout << "Image at location : " << filename << " could not be opened" << std::endl;
    result = cv::Mat::zeros(100, 100, CV_64F);
  }

  return result;
}

//---------------------------------------------------------------------------
void DisplayImage(const cv::Mat& Im, std::string windowName)
{
  // create a window for display the image
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);

  // show the image
  cv::imshow(windowName, Im);

  // wait for the user to press a key
  std::cout << "Image displayed, press a key to close the window" << std::endl;
  cv::waitKey(0);
}

//---------------------------------------------------------------------------
void DrawCircle(cv::Mat& Im, std::vector<cv::KeyPoint> pts)
{
  cv::drawKeypoints(Im, pts, Im);
  std::cout << "drawn " << pts.size() << " points" << std::endl;
}

//---------------------------------------------------------------------------
cv::Mat DrawMatch(cv::Mat& Im1, cv::Mat& Im2, std::vector<cv::KeyPoint> pts1,
                  std::vector<cv::KeyPoint> pts2, std::vector<cv::DMatch> matches)
{
  cv::Mat img_matches;
  cv::drawMatches(Im1, pts1, Im2, pts2,
                  matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                  std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  return img_matches;
}

//---------------------------------------------------------------------------
std::pair<cv::Mat, cv::Mat> DrawEpipolarLines(cv::Mat& Im1, cv::Mat& Im2, std::vector<cv::KeyPoint> pts1,
                       std::vector<cv::KeyPoint> pts2, cv::Mat& F, std::vector<cv::DMatch> matches)
{
  std::cout << "size match1 : " << pts1.size() << std::endl;
  std::cout << "size match2 : " << pts2.size() << std::endl;
  cv::Mat res1 = Im1;
  cv::Mat res2 = Im2;

  //Extract the points from keyPoints
  std::vector<cv::Point2f> scene1;
  std::vector<cv::Point2f> scene2;
  for(int i = 0 ; i < matches.size(); ++i)
  {
      scene1.push_back(pts1[i].pt);
      scene2.push_back(pts2[i].pt);
  }

  // Epilines1 are the epilines of the first image
  // Epilines2 are the epilines of the second image
  std::vector<cv::Vec<float,3> > epilines1, epilines2;

  // Use the keyPoints1 of first frame to compute the epipolar
  // lines of the second frames usgin F. Remind, F is forward 1 -> 2
  // F.transpose is backward 2 -> 1
  cv::computeCorrespondEpilines(scene1, 1, F, epilines2);

  // Use the keyPoints2 of second frame to compute the epipolar
  // lines of the first frames usgin F. Remind, F is forward 1 -> 2
  // F.transpose is backward 2 -> 1
  cv::computeCorrespondEpilines(scene2, 2, F, epilines1);

  // random generator to colorize the lines
  cv::RNG rng(0);

  // Draw lines to show epilines
  for (int i = 0; i < pts2.size(); i++)
  {
    //a line is ax + by + c = 0 thus y = -a/b*x -c/b
    // with a = epiline[0], b = epiline[1], c = epiline[2]
    cv::Scalar color(rng(256),rng(256),rng(256));

    // First point
    cv::Point P1(Im1.cols,-epilines1[i][2]/epilines1[i][1]); //Point (0, -c/b)

    // Second point
    cv::Point P2(0, -epilines1[i][0]/epilines1[i][1]*Im1.cols - epilines1[i][2]/epilines1[i][1]); //Point (W, -a/b*W-c/b)

    // Draw the line
    cv::line(res1, P1,P2,color);

    // Draw the corresponding matching points
    cv::circle(res1, cv::Point(pts1[i].pt.x, pts1[i].pt.y), 5, color, 2);
  }

  // Draw lines to show epilines
  for (int i = 0; i < pts1.size(); i++)
  {
    //a line is ax + by + c = 0 thus y = -a/b*x -c/b
    // with a = epiline[0], b = epiline[1], c = epiline[2]
    cv::Scalar color(rng(256),rng(256),rng(256));

    // First point
    cv::Point P1(Im2.cols,-epilines2[i][2]/epilines2[i][1]); //Point (0, -c/b)

    // Second point
    cv::Point P2(0, -epilines2[i][0] / epilines2[i][1] * Im2.cols - epilines2[i][2] / epilines2[i][1]); //Point (W, -a/b*W-c/b)

    // Draw the line
    cv::line(res2, P1,P2,color);

    // Draw the corresponding matching points
    cv::circle(res2, cv::Point(pts2[i].pt.x, pts2[i].pt.y), 5, color, 2);
  }

  return std::pair<cv::Mat, cv::Mat>(res1, res2);
}

//---------------------------------------------------------------------------
cv::Mat ComputeDepthMapUncalibrated(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY)
{
  int radiusX = static_cast<int>(std::ceil(static_cast<double>(blockSizeX) / 2.0));
  int radiusY = static_cast<int>(std::ceil(static_cast<double>(blockSizeY) / 2.0));
  cv::Mat RectGrey1, RectGrey2;
  cv::cvtColor(Rectified1, RectGrey1, CV_BGR2GRAY);
  cv::cvtColor(Rectified2, RectGrey2, CV_BGR2GRAY);
  cv::Mat DepthMap = cv::Mat::zeros(RectGrey1.size(), CV_32F);

  double maxDepth = 0;

  int nbrLine = RectGrey1.rows - 2 * radiusX;
  double percentil = 0.01; // every 1%
  int percentilInt = percentil * nbrLine;
  int count = 0;
  for (int i = radiusX; i < RectGrey1.rows - radiusX; ++i)
  {
    if (i % percentilInt == 0)
    {
      int percentageDone = percentil * 100 * count;
      std::cout << "DepthMap computation : " << percentageDone << "%" << std::endl;
      count++;
    }
    for (int j = radiusY; j < RectGrey1.cols - radiusY; ++j)
    {
      cv::Rect roi(j - radiusY, i - radiusX, blockSizeY, blockSizeX);
      cv::Mat croppedImage = RectGrey1(roi);
      std::pair<double, double> matchIndex = FindBestMatchOfCroppedIm(croppedImage, RectGrey2, blockSizeX, blockSizeY, i, j, 50);
      int y0 = matchIndex.second;
      int y1 = j;

      // same point
      if (y0 == y1)
      {
        DepthMap.at<float>(i, j) = -1;
      }
      else
      {
        float depth = 1.0 / std::abs(y0 - y1);
        DepthMap.at<float>(i, j) = depth;
        if (depth > maxDepth)
        {
          maxDepth = depth;
        }
      }
    }
  }

  cv::Mat Res = cv::Mat::zeros(DepthMap.size(), CV_8U);
  for (int i = 0; i < DepthMap.rows; ++i)
  {
    for (int j = 0; j < DepthMap.cols; ++j)
    {
      double x = DepthMap.at<float>(i, j);
      if (x >= 0)
      {
        double y = 254 - (x / maxDepth * 254);
        int intY = static_cast<int>(y);
        uchar ucharY = static_cast<uchar>(y);
        Res.at<uchar>(i, j) = ucharY;
      }
      else
      {
        Res.at<uchar>(i, j) = 0;
      }
    }
  }

  return Res;
}

//---------------------------------------------------------------------------
std::pair<double, double> FindBestMatchOfCroppedIm(cv::Mat croppedIm, cv::Mat target, int blockSizeX, int blockSizeY, int i, int j, int maxDisparity)
{
  int radiusX = static_cast<int>(std::ceil(static_cast<double>(blockSizeX) / 2.0));
  int radiusY = static_cast<int>(std::ceil(static_cast<double>(blockSizeY) / 2.0));
  std::pair<double, double> res(-1, -1);
  double minDist = 1e20;

  int kMin = std::max(radiusY, j - maxDisparity);
  int kMax = std::min(target.cols - radiusY, j + maxDisparity);

  for (int k = kMin; k < kMax; ++k)
  {
    cv::Rect roi (k - radiusY, i - radiusX, blockSizeY, blockSizeX);
    cv::Mat croppedIm2 = target(roi);
    double dist = SumSquareDistance(croppedIm, croppedIm2);

    if (dist < minDist)
    {
      minDist = dist;
      res.first = i;
      res.second = k;
    }
  }

  if (res.first < 0 || res.second < 0)
  {
    res.first = i;
    res.second = j;
  }

  return res;
}

//---------------------------------------------------------------------------
double SumSquareDistance(cv::Mat im1, cv::Mat im2)
{
  if( (im1.rows != im2.rows) || (im1.cols != im2.cols) )
  {
    std::cout << "Error images must have same dimension" << std::endl;
    return 0;
  }

  double dist = 0;
  for(int i = 0; i < im1.rows; ++i)
  {
    for(int j = 0; j < im1.cols; ++j)
    {
      dist = dist + std::abs( static_cast<double>(im1.at<uchar>(i,j)) - static_cast<double>(im2.at<uchar>(i,j)) );
    }
  }

  return dist;
}

//---------------------------------------------------------------------------
cv::Mat ComputeDepthMapUncalibrated2(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY)
{
  /* http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereosgbm-stereosgbm
   * minDisparity - Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so
   * this parameter needs to be adjusted accordingly.
   *
   * numDisparities - Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter
   * must be divisible by 16.
   *
   * SADWindowSize - Matched block size. It must be an odd number >=1. Normally, it should be somewhere in the 3..11 range.
   *
   * P1 - The first parameter controlling the disparity smoothness. See below.
   *
   * P2 - The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity
   * change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
   * The algorithm requires P2 > P1. See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown
   * (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize, respectively).
   *
   * disp12MaxDiff - Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
   *
   * preFilterCap - Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by
   * [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
   *
   * uniquenessRatio - Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to
   * consider the found match correct. Normally, a value within the 5-15 range is good enough.
   *
   * speckleWindowSize - Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering.
   * Otherwise, set it somewhere in the 50-200 range.
   *
   * speckleRange - Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be
   * implicitly multiplied by 16. Normally, 1 or 2 is good enough.
   *
   * fullDP - Set it to true to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo
   *  and huge for HD-size pictures. By default, it is set to false.
    */

  int minDisparity = -64;
  int numDisparity = 192;
  int SADWindowSize = 3;
  int P1 = 600;
  int P2 = 2400;
  int disp12MaxDiff = 30;
  int preFilterCap = 4;
  int uniquenessRatio = 1;
  int speckleWindowSize = 150;
  int speckleRange = 2;
  bool fullDP = false;

  cv::Mat RectGrey1, RectGrey2;
  cv::cvtColor(Rectified1, RectGrey1, CV_BGR2GRAY);
  cv::cvtColor(Rectified2, RectGrey2, CV_BGR2GRAY);
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity,
                                                        numDisparity,
                                                        SADWindowSize,
                                                        P1,
                                                        P2,
                                                        disp12MaxDiff,
                                                        preFilterCap,
                                                        uniquenessRatio,
                                                        speckleWindowSize,
                                                        speckleRange,
                                                        fullDP);

  cv::Mat depth, depth8;
  sgbm->compute(RectGrey1, RectGrey2, depth);

  cv::normalize(depth, depth8, 0, 255, CV_MINMAX, CV_8U);

  return depth8;
}

//---------------------------------------------------------------------------
cv::Mat ComputeDepthMapUncalibrated3(cv::Mat Rectified1, cv::Mat Rectified2, int blockSizeX, int blockSizeY)
{
  cv::Mat RectGrey1, RectGrey2;
  cv::cvtColor(Rectified1, RectGrey1, CV_BGR2GRAY);
  cv::cvtColor(Rectified2, RectGrey2, CV_BGR2GRAY);

  cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(16 * 5, 21);
  sbm->setBlockSize(21);
  sbm->setNumDisparities(7 * 16);
  sbm->setPreFilterSize(5);
  sbm->setPreFilterCap(61);
  sbm->setMinDisparity(-39);
  sbm->setTextureThreshold(507);
  sbm->setUniquenessRatio(0);
  sbm->setSpeckleWindowSize(0);
  sbm->setSpeckleRange(8);
  sbm->setDisp12MaxDiff(1);

  cv::Mat depth, depth8;
  sbm->compute(RectGrey1, RectGrey2, depth);

  cv::normalize(depth, depth8, 0, 255, CV_MINMAX, CV_8U);

  return depth8;
}

// Convert a vector of non-homogeneous 2D points to a vector of homogenehous 2D points.
void to_homogeneous(const std::vector< cv::Point2f >& non_homogeneous, std::vector< cv::Point3f >& homogeneous)
{
    homogeneous.resize(non_homogeneous.size());
    for (size_t i = 0; i < non_homogeneous.size(); i++) {
        homogeneous[i].x = non_homogeneous[i].x;
        homogeneous[i].y = non_homogeneous[i].y;
        homogeneous[i].z = 1.0;
    }
}

// Convert a vector of homogeneous 2D points to a vector of non-homogenehous 2D points.
void from_homogeneous(const std::vector< cv::Point3f >& homogeneous, std::vector< cv::Point2f >& non_homogeneous)
{
    non_homogeneous.resize(homogeneous.size());
    for (size_t i = 0; i < non_homogeneous.size(); i++) {
        non_homogeneous[i].x = homogeneous[i].x / homogeneous[i].z;
        non_homogeneous[i].y = homogeneous[i].y / homogeneous[i].z;
    }
}

// Transform a vector of 2D non-homogeneous points via an homography.
std::vector<cv::Point2f> transform_via_homography(const std::vector<cv::Point2f>& points, const cv::Matx33f& homography)
{
    std::vector<cv::Point3f> ph;
    to_homogeneous(points, ph);
    for (size_t i = 0; i < ph.size(); i++) {
        ph[i] = homography*ph[i];
    }
    std::vector<cv::Point2f> r;
    from_homogeneous(ph, r);
    return r;
}

// Find the bounding box of a vector of 2D non-homogeneous points.
cv::Rect_<float> bounding_box(const std::vector<cv::Point2f>& p)
{
    cv::Rect_<float> r;
    float x_min = std::min_element(p.begin(), p.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {return lhs.x < rhs.x; })->x;
    float x_max = std::max_element(p.begin(), p.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {return lhs.x < rhs.x; })->x;
    float y_min = std::min_element(p.begin(), p.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {return lhs.y < rhs.y; })->y;
    float y_max = std::max_element(p.begin(), p.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {return lhs.y < rhs.y; })->y;
    return cv::Rect_<float>(x_min, y_min, x_max - x_min, y_max - y_min);
}

// Warp the image src into the image dst through the homography H.
// The resulting dst image contains the entire warped image, this
// behaviour is the same of Octave's imperspectivewarp (in the 'image'
// package) behaviour when the argument bbox is equal to 'loose'.
// See http://octave.sourceforge.net/image/function/imperspectivewarp.html
std::vector< cv::Point2f > homography_warp(const cv::Mat& src, const cv::Mat& H, cv::Mat& dst)
{
    std::vector< cv::Point2f > corners;
    corners.push_back(cv::Point2f(0, 0));
    corners.push_back(cv::Point2f(src.cols, 0));
    corners.push_back(cv::Point2f(0, src.rows));
    corners.push_back(cv::Point2f(src.cols, src.rows));
    corners.push_back(cv::Point2f(src.cols / 2, src.rows / 2));

    std::vector< cv::Point2f > projected = transform_via_homography(corners, H);
    cv::Rect_<float> bb = bounding_box(projected);

    cv::Mat_<double> translation = (cv::Mat_<double>(3, 3) << 1, 0, -bb.tl().x, 0, 1, -bb.tl().y, 0, 0, 1);

    cv::warpPerspective(src, dst, translation*H, bb.size());

    cv::Mat G = translation * H;
    return transform_via_homography(corners, G);
}

//---------------------------------------------------------------------------
void BrightnessRescaleBimodal(cv::Mat& src, cv::Mat& dst, double saturationThresholdLeft, double saturationThresholdRight)
{
  int nQuantum = 256;
  int halfNQuantum = nQuantum / 2;
  dst = cv::Mat::zeros(src.size(), src.type());

  double cb = 333.3333;//114.0;
  double cg = 333.3333;//587.0;
  double cr = 333.3333;//299.0

  // Create the histogram of the current image
  std::vector<double> hist(256, 0);
  double brightness = 0;
  for (int i = 0; i < src.rows; ++i)
  {
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b colour = src.at<cv::Vec3b>(i, j);
      // compute the bright
      brightness = (cb * colour[0] + cg * colour[1] + cb * colour[2]) / 1000.0;
      hist[static_cast<int>(brightness)]++;
    }
  }

  double total = 1.0 * src.cols * src.rows;

  // normalize the histogram to have a pdf
  double moy = 0;
  for (int k = 0; k < 256; ++k)
  {
    hist[k] /= total;
    moy += hist[k] * k;
  }

  // shift the histogram so that the mean in centered
  //double b =
  int ignoreAlreadySaturated = 0;
  int indexLeft[2] = { ignoreAlreadySaturated, halfNQuantum };
  int indexRight[2] = { halfNQuantum, nQuantum - 1 - ignoreAlreadySaturated };
  int count = 0;

  // current percentage of saturation
  double saturationLeft[2] = { hist[indexLeft[0]], hist[indexLeft[1]] };
  double saturationRight[2] = { hist[indexRight[0]], hist[indexRight[1]] };

  while (saturationLeft[0] < (saturationThresholdLeft / 2.0))
  {
    indexLeft[0]++;
    saturationLeft[0] += hist[indexLeft[0]];
  }

  while (saturationLeft[1] < (saturationThresholdLeft / 2.0))
  {
    indexLeft[1]--;
    saturationLeft[1] += hist[indexLeft[1]];
  }

  while (saturationRight[0] < (saturationThresholdRight / 2.0))
  {
    saturationRight[0]++;
    saturationRight[0] += hist[saturationRight[0]];
  }

  while (saturationRight[1] < (saturationThresholdRight / 2.0))
  {
    indexRight[1]--;
    saturationRight[1] += hist[indexRight[1]];
  }

  double aL = static_cast<double>(halfNQuantum - 0) / (static_cast<double>(indexLeft[1] - indexLeft[0]));
  double bL = -aL * static_cast<double>(indexLeft[0]);

  double aR = static_cast<double>(nQuantum - halfNQuantum) / (static_cast<double>(indexRight[1] - indexRight[0]));
  double bR = halfNQuantum - aR * static_cast<double>(indexRight[0]);

  for (int i = 0; i < dst.rows; ++i)
  {
    for (int j = 0; j < dst.cols; ++j)
    {
      int R = src.at<cv::Vec3b>(i, j)[2];
      int G = src.at<cv::Vec3b>(i, j)[1];
      int B = src.at<cv::Vec3b>(i, j)[0];
      brightness = (cr * R + cg * G + cb * B) / 1000.0;
      double value = brightness;

      if (value < halfNQuantum)
      {
        R = aL * R + bL;
        G = aL * G + bL;
        B = aL * B + bL;

        if (R > halfNQuantum || G > halfNQuantum || B > halfNQuantum)
        {
          R = src.at<cv::Vec3b>(i, j)[2];
          G = src.at<cv::Vec3b>(i, j)[1];
          B = src.at<cv::Vec3b>(i, j)[0];
        }
      }
      else
      {
        //value = std::max(1.0 * halfNQuantum, aR * value + bR);
        //value = aR * value + bR;
        R =  halfNQuantum, aR * R + bR;
        G =  halfNQuantum, aR * G + bR;
        B =  halfNQuantum, aR * B + bR;

        if (true/*R < halfNQuantum || G < halfNQuantum || B < halfNQuantum*/)
        {
          R = src.at<cv::Vec3b>(i, j)[2];
          G = src.at<cv::Vec3b>(i, j)[1];
          B = src.at<cv::Vec3b>(i, j)[0];
        }
      }

      dst.at<cv::Vec3b>(i, j)[2] = cv::saturate_cast<uchar>(R);
      dst.at<cv::Vec3b>(i, j)[1] = cv::saturate_cast<uchar>(G);
      dst.at<cv::Vec3b>(i, j)[0] = cv::saturate_cast<uchar>(B);

    }
  }

  std::cout << "left saturation 0 : " << saturationLeft[0] << std::endl;
  std::cout << "left saturation 1 : " << saturationLeft[1] << std::endl;
  std::cout << "right saturation 0 : " << saturationRight[0] << std::endl;
  std::cout << "right saturation 1 : " << saturationRight[1] << std::endl;
  std::cout << "final saturation : " << saturationLeft[0] + saturationRight[0] + saturationLeft[1] + saturationRight[1] << std::endl;
  std::cout << "left index 0 : " << indexLeft[0] << std::endl;
  std::cout << "left index 1 : " << indexLeft[1] << std::endl;
  std::cout << "right index 0 : " << indexRight[0] << std::endl;
  std::cout << "right index 1 : " << indexRight[1] << std::endl;
  std::cout << "moy : " << moy << std::endl;
}

//---------------------------------------------------------------------------
void BrightnessRescale(cv::Mat& src, cv::Mat& dst, double saturationThreshold)
{
  int nQuantum = 256;
  dst = cv::Mat::zeros(src.size(), src.type());

  // Create the histogram of the current image
  std::vector<double> hist(256, 0);
  double brightness = 0;
  for (int i = 0; i < src.rows; ++i)
  {
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b colour = src.at<cv::Vec3b>(i, j);
      // compute the bright
      brightness = (299.0 * colour[0] + 587.0 * colour[1] + 114.0 * colour[2]) / 1000.0;
      hist[static_cast<int>(brightness)]++;
    }
  }

  double total = 1.0 * src.cols * src.rows;

  // normalize the histogram to have a pdf
  double moy = 0;
  for (int k = 0; k < 256; ++k)
  {
    hist[k] /= total;
    moy += hist[k] * k;
  }

  // shift the histogram so that the mean in centered
  //double b =
  int ignoreAlreadySaturated = 4;
  int indexLeft = ignoreAlreadySaturated;
  int indexRight = nQuantum - 1 - ignoreAlreadySaturated;
  int count = 0;

  // current percentage of saturation
  double saturationLeft = hist[indexLeft];
  double saturationRight = hist[indexRight];

  while (saturationLeft < (saturationThreshold / 2.0))
  {
    indexLeft++;
    saturationLeft += hist[indexLeft];
  }

  while (saturationRight < (saturationThreshold / 2.0))
  {
    indexRight--;
    saturationRight += hist[indexRight];
  }

  double a = static_cast<double>(nQuantum) / (static_cast<double>(indexRight - indexLeft));
  double b = -a * static_cast<double>(indexLeft);

  for (int i = 0; i < dst.rows; ++i)
  {
    for (int j = 0; j < dst.cols; ++j)
    {
      for(int c = 0; c < 3; ++c)
      {
        dst.at<cv::Vec3b>(i, j)[c] =
            cv::saturate_cast<uchar>(a * src.at<cv::Vec3b>(i, j)[c] + b);
      }
    }
  }

  std::cout << "left saturation : " << saturationLeft << std::endl;
  std::cout << "right saturation : " << saturationRight << std::endl;
  std::cout << "final saturation : " << saturationLeft + saturationRight << std::endl;
  std::cout << "left index : " << indexLeft << std::endl;
  std::cout << "right index : " << indexRight << std::endl;
  std::cout << "moy : " << moy << std::endl;
}

//---------------------------------------------------------------------------
void BrightnessCalibration(cv::Mat& src, cv::Mat& target, cv::Mat& dst, double saturationThreshold)
{
  int nQuantum = 256;
  dst = cv::Mat::zeros(src.size(), src.type());

  // Create the histogram of the current
  std::vector<double> histSrc(256, 0);
  double brightness = 0;
  for (int i = 0; i < src.rows; ++i)
  {
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b colour = src.at<cv::Vec3b>(i, j);
      // compute the bright
      brightness = (299.0 * colour[0] + 587.0 * colour[1] + 114.0 * colour[2]) / 1000.0;
      histSrc[static_cast<int>(brightness)]++;
    }
  }
  // Create the histogram of the target
  std::vector<double> histTarget(256, 0);
  for (int i = 0; i < target.rows; ++i)
  {
    for (int j = 0; j < target.cols; ++j)
    {
      cv::Vec3b colour = target.at<cv::Vec3b>(i, j);
      // compute the bright
      brightness = (299.0 * colour[0] + 587.0 * colour[1] + 114.0 * colour[2]) / 1000.0;
      histTarget[static_cast<int>(brightness)]++;
    }
  }

  double totalSrc = 1.0 * src.cols * src.rows;
  double totalTarget = 1.0 * target.cols * target.rows;

  // normalize the histogram to have a pdf
  double moySrc = 0;
  double moyTarget = 0;
  for (int k = 0; k < 256; ++k)
  {
    histSrc[k] /= totalSrc;
    moySrc += histSrc[k] * k;

    histTarget[k] /= totalTarget;
    moyTarget += histTarget[k] * k;
  }

  double varSrc = 0;
  double varTarget = 0;
  for (int k = 0; k < 256; ++k)
  {
    varSrc += histSrc[k] * (k - moySrc) * (k - moySrc);
    varTarget += histTarget[k] * (k - moyTarget) * (k - moyTarget);
  }

  varSrc = std::sqrt(varSrc);
  varTarget = std::sqrt(varTarget);


  double a = varTarget / varSrc;
  double b = moyTarget - varTarget / varSrc * moySrc;

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
void WhiteBalanceGreyAssumption(cv::Mat& src, cv::Mat& dst, double saturationRatio)
{
  dst = cv::Mat::zeros(src.size(), src.type());

  // Histogram of each channel
  int nQuantum = 256;
  std::vector<std::vector<int> > HList(3, std::vector<int>(nQuantum, 0));

  // Populate histograms
  for (int i = 0; i < dst.rows; ++i)
  {
    for (int j = 0; j < dst.cols; ++j)
    {
      cv::Vec3b colour = src.at<cv::Vec3b>(i, j);
      HList[0][colour[0]]++;
      HList[1][colour[1]]++;
      HList[2][colour[2]]++;
    }
  }

  int nbrPixels = src.cols * src.rows;
  int limitMin[3], limitMax[3];

  // Compute the limit so that they keep the saturationRatio
  // for each channel
  for (int i = 0; i < 3; ++i)
  {
    // cumulate the histogram
    for (int j = 0; j < nQuantum - 1; ++j)
    {
      HList[i][j + 1] += HList[i][j];
    }

    limitMin[i] = 0;
    limitMax[i] = nQuantum - 1;

    // increase min limot
    while (HList[i][limitMin[i]] < saturationRatio * nbrPixels)
    {
      limitMin[i]++;
    }

    // decrease the max limit
    while (HList[i][limitMax[i]] > (1 - saturationRatio) * nbrPixels)
    {
      limitMax[i]--;
    }
  }

  // Now rescale each channel independently to maximiza histogram range
  for (int i = 0; i < src.rows; ++i)
  {
    for (int j = 0; j < src.cols; ++j)
    {
      cv::Vec3b colour = src.at<cv::Vec3b>(i, j);
      for (int c = 0; c < 3; ++c)
      {
        // clamping
        colour[c] = std::max(colour[c], static_cast<uchar>(limitMin[c]));
        colour[c] = std::min(colour[c], static_cast<uchar>(limitMax[c]));

        colour[c] = static_cast<uchar>(255.0 * (colour[c] - limitMin[c]) / (limitMax[c] - limitMin[c]));
      }

      dst.at<cv::Vec3b>(i, j) = colour;
    }
  }
}

//---------------------------------------------------------------------------
cv::Rect InteriorRectangleOfWarpedImage(std::vector< cv::Point2f > input, double aspectRatio)
{
  Eigen::Matrix<double, 2, 1> A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 2, 1> C;
  Eigen::Matrix<double, 2, 1> D;

  A << input[0].y, input[0].x;
  B << input[1].y, input[1].x;
  C << input[3].y, input[3].x;
  D << input[2].y, input[2].x;

  // Get the top and the bottom of the inside rectangle. 
  double xmin = std::max(A(0), B(0));
  double xmax = std::min(C(0), D(0));

  // Now we want the two sides, compute intersection with borders
  // ymin
  Eigen::Matrix<double, 2, 1> AD = D - A;
  double ya = A(1) + (xmin - A(0)) / AD(0) * AD(1);
  double yd = A(1) + (xmax - A(0)) / AD(0) * AD(1);
  double ymin = std::max(ya, yd);

  // ymax
  Eigen::Matrix<double, 2, 1> BC = C - B;
  double yb = B(1) + (xmin - B(0)) / BC(0) * BC(1);
  double yc = B(1) + (xmax - B(0)) / BC(0) * BC(1);
  double ymax = std::min(yb, yc);

  double xRange = xmax - xmin;
  double yRange = ymax - ymin;

  double x0 = xRange / 2 + xmin;
  double y0 = yRange / 2 + ymin;

  // test X
  double expectedYRange = aspectRatio * xRange;

  if (expectedYRange > yRange)
  {
    xRange = yRange / aspectRatio;
    yRange = ymax - ymin;
  }
  else
  {
    xRange = xmax - xmin;
    yRange = aspectRatio * xRange;
  }

  ymin = y0 - yRange / 2;
  ymax = y0 + yRange / 2;
  xmin = x0 - xRange / 2;
  xmax = x0 + xRange / 2;

  return cv::Rect(ymin, xmin, ymax - ymin, xmax - xmin);
}

//---------------------------------------------------------------------------
cv::Rect CenteredRectangle(std::vector<cv::Point2f> input, int H, int W)
{
  // Center of the image
  Eigen::Matrix<double, 2, 1> Center = Eigen::Matrix<double, 2, 1>(input[4].y, input[4].x);

  return cv::Rect(Center(1) - W / 2, Center(0) - H / 2, W, H);
}

//---------------------------------------------------------------------------
double DiagLengthRectangleInsidePolygone(std::vector< cv::Point2f > input, double aspectRatio)
{
  // Order the polygone and make a loop
  std::vector<Eigen::Matrix<double, 2, 1> > ConvexPolygone;
  ConvexPolygone.push_back(Eigen::Matrix<double, 2, 1>(input[0].y, input[0].x));
  ConvexPolygone.push_back(Eigen::Matrix<double, 2, 1>(input[1].y, input[1].x));
  ConvexPolygone.push_back(Eigen::Matrix<double, 2, 1>(input[3].y, input[3].x));
  ConvexPolygone.push_back(Eigen::Matrix<double, 2, 1>(input[2].y, input[2].x));
  ConvexPolygone.push_back(Eigen::Matrix<double, 2, 1>(input[0].y, input[0].x));

  // Center of the image
  Eigen::Matrix<double, 2, 1> Center = Eigen::Matrix<double, 2, 1>(input[4].y, input[4].x);

  // The diagonals of the rectangle we want to
  // fit inside the polygone. We will preserve
  // the aspect ratio
  Eigen::Matrix<double, 2, 1> u1, u2;
  u1 << 1, aspectRatio;
  u2 << -1, aspectRatio;
  u1.normalize();
  u2.normalize();

  double ymin = 1000000000;

  // we will compute the intersection between the diagonals
  // and the edges of the polygone. As soon as a diagonal has
  // crossed an edge the rectangle can't expanse anymore
  for (unsigned int k = 0; k < ConvexPolygone.size() - 1; ++k)
  {
    Eigen::Matrix<double, 2, 1> P, v;
    P = ConvexPolygone[k] - Center;;
    v = ConvexPolygone[k + 1] - ConvexPolygone[k];
    v.normalize();

    Eigen::Matrix<double, 2, 2> M1, M2;
    M1 << u1(0), -v(0), u1(1), -v(1);
    M2 << u2(0), -v(0), u2(1), -v(1);

    // Find parametrics coordinate of the intersection point
    Eigen::Matrix<double, 2, 1> params1 = M1.inverse() * P;
    Eigen::Matrix<double, 2, 1> params2 = M2.inverse() * P;

    Eigen::Matrix<double, 2, 1> X1, X2;
    X1 = params1(0) * u1;
    X2 = params1(1) * v + P;

    std::cout << "Intersection: " << X1.transpose() + Center.transpose() << std::endl;
    std::cout << "line: " << ConvexPolygone[k + 1].transpose() << std::endl;
    std::cout << "line: " << ConvexPolygone[k].transpose() << std::endl;
    std::cout << std::endl;

    if ((X1 - X2).norm() > 1e-6)
    {
      std::cout << "Error computing the intersection" << std::endl;
    }
    ymin = std::min(std::abs(X1(0)), ymin);

    X1 = params2(0) * u2;
    X2 = params2(1) * v + P;

    if ((X1 - X2).norm() > 1e-6)
    {
      std::cout << "Error computing the intersection" << std::endl;
    }
    ymin = std::min(std::abs(X1(0)), ymin);
  }

  std::cout << "diagonal length: " << ymin << std::endl;

  return ymin;
}

//---------------------------------------------------------------------------
std::pair<cv::Rect, cv::Rect> KeepSmallerRectangle(std::vector< cv::Point2f > inputL, std::vector< cv::Point2f > inputR, double yL, double yR, double aspectRatioR, double aspectRatioL)
{
  cv::Point2f Cl = inputL[4];
  cv::Point2f Cr = inputR[4];

  cv::Rect rectL, rectR;
  //cv::Rect(x, y, dx, dy);
  if (yL < yR)
  {
    double xL = Cl.x - aspectRatioL * yL;
    double xR = Cr.x - aspectRatioL * yL;

    double yLe = Cl.y - yL;
    double yRi = Cr.y - yL;

    double dx = 2 * aspectRatioL * yL;
    double dy = 2 * yL;
    rectL = cv::Rect(xL, yLe, dx, dy);
    rectR = cv::Rect(xR, yRi, dx, dy);
  }
  else
  {
    double xL = Cl.x - aspectRatioR * yR;
    double xR = Cr.x - aspectRatioR * yR;

    double yLe = Cl.y - yR;
    double yRi = Cr.y - yR;

    double dx = 2 * aspectRatioR * yR;
    double dy = 2 * yR;
    rectL = cv::Rect(xL, yLe, dx, dy);
    rectR = cv::Rect(xR, yRi, dx, dy);
  }

  std::pair<cv::Rect, cv::Rect> ret(rectL, rectR);
  return ret;
}