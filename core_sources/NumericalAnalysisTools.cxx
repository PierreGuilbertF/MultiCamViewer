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

// LOCAL
#include "NumericalAnalysisTools.h"

// STD
#include <iostream>
#include <fstream>

//---------------------------------------------------------------------------
Eigen::Matrix<float, 3, 3> GetBestPlane(std::vector<Eigen::Matrix<float, 3, 1> > Points, std::vector<float> Weight)
{
  // PCA to get best plane

  // Distribution matrix : Points.size() realisation
  // 3 random variables (X, Y, Z)
  Eigen::MatrixXd M;
  M.resize(Points.size(), 3);

  // One is the vector of R^k with all coordinate set to 1
  // G is the gravity center the Matrix M. The vector which all coordinate
  // is the barycenter of the random variable according to the weight setted.
  Eigen::MatrixXd G, One;
  G.resize(Points.size(), 1);
  One.resize(Points.size(), 1);

  // Matrix which represent the weight
  Eigen::MatrixXd D;
  D.resize(Points.size(), Points.size());

  double SumWeight = 0;
  for (int i = 0; i < Points.size(); ++i)
  {
    SumWeight += Weight[i];
    for (int j = 0; j < Points.size(); ++j)
    {
      D(i, j) = 0;
    }
  }

  for (int i = 0; i < Points.size(); ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      M(i, j) = Points[i](j);
    }
    D(i, i) = Weight[i] / SumWeight;
    One(i) = 1;
  }

  G = M.transpose() * D * One;

  Eigen::MatrixXd M2 = M - One * G.transpose();

  // Variance - Covariance matrix
  Eigen::MatrixXd V = M2.transpose() * D * M2;

  // V is symmetric and positive, thanks to the spectral theorem
  // V is diagonaziable. The eigen vector will determines the direction
  // of the best approximation plane. The gravity center determine a point
  // of the plane
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(V);
  std::cout << "Eigen values are : \n" << eigensolver.eigenvalues() << std::endl;
  std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
            << "corresponding to these eigenvalues:\n"
            << eigensolver.eigenvectors() << std::endl;

  Eigen::MatrixXd U = eigensolver.eigenvectors();

  std::cout << "Weight matrix : " << std::endl;
  std::cout << D << std::endl;
  std::cout << "Distribution matrix : " << std::endl;
  std::cout << M << std::endl;
  std::cout << "Mean matrix : " << std::endl;
  std::cout << G << std::endl;

  Eigen::Matrix<float, 3, 3> Res;
  Res << U(0, 2), U(0, 1), G(0),
         U(1, 2), U(1, 1), G(1),
         U(2, 2), U(2, 1), G(2);

  std::cout << "Best plane : [Vect1 | Vect2 | Point] : \n" << Res << std::endl;

  return Res;
}

//---------------------------------------------------------------------------
Eigen::Matrix<float, 3, 1> IntersectionLinePlane(Eigen::Matrix<float, 3, 2> line, Eigen::Matrix<float, 3, 3> plane)
{
  Eigen::Vector3f u, v, w, A, B, n;
  u << line(0, 0), line(1, 0), line(2, 0);
  v << plane(0, 0), plane(1, 0), plane(2, 0);
  w << plane(0, 1), plane(1, 1), plane(2, 1);
  A << line(0, 1), line(1, 1), line(2, 1);
  B << plane(0, 2), plane(1, 2), plane(2, 2);

  // othogonal vector of the plane
  n = v.cross(w);

  double lambda = - ((A - B).dot(n)) / (u.dot(n));

  Eigen::Matrix<float, 3, 1> X;
  X(0) = A(0) + lambda * u(0);
  X(1) = A(1) + lambda * u(1);
  X(2) = A(2) + lambda * u(2);

  //std::cout << "Intersection : " << X << std::endl;
  return X;
}

//---------------------------------------------------------------------------
Eigen::Matrix<float, 3, 3> ComputeHomographyFromPlane(Eigen::Matrix<float, 3, 3> plane, Eigen::Matrix<float, 3, 4> P)
{
  Eigen::Vector3f u, v, A, n;
  u << plane(0, 0), plane(1, 0), plane(2, 0);
  v << plane(0, 1), plane(1, 1), plane(2, 1);
  A << plane(0, 2), plane(1, 2), plane(2, 2);

  // orthogonal vector of the plane
  n = u.cross(v);

  // Homogeneous equation of the plane
  Eigen::Matrix<float, 3, 1> Ph;
  double coeff = -n.dot(A);
  Ph << n(0) / coeff, n(1) / coeff, n(2) / coeff;

  // Temp matrix :
  Eigen::Matrix<float, 3, 3> RotPart;
  RotPart << P(0, 0), P(0, 1), P(0, 2),
             P(1, 0), P(1, 1), P(1, 2),
             P(2, 0), P(2, 1), P(2, 2);

  Eigen::Matrix<float, 3, 1> TPart;
  TPart << P(0, 3), P(1, 3), P(2, 3);

  Eigen::Matrix<float, 3, 3> H;
  H = RotPart - TPart * Ph.transpose();

  return H;
}

//---------------------------------------------------------------------------
std::pair<double, double> TimeSynchronization(double* data1, double* data2, int size1, int size2, int sampleRate)
{
  // Variables initialization
  cv::Mat S1(size1, 1, CV_32F);
  cv::Mat S2(size2, 1, CV_32F);

  // We will multiply the input signals with a gaussian function
  // to ensure the continuity in the boundaries
  /*double sigma1 = 1.0 * size1 / 6.0;
  double x1 = 1.0 * size1 / 2.0;
  double sigma2 = 1.0 * size2 / 6.0;
  double x2 = 1.0 * size2 / 2.0;
  for (int k = 0; k < size1; ++k)
  {
    data1[k] = data1[k] * std::exp(-std::pow(1.0 * k - x1, 2) / (2.0 * sigma1 * sigma1));
  }
  for (int k = 0; k < size2; ++k)
  {
    data2[k] = data2[k] * std::exp(-std::pow(1.0 * k - x2, 2) / (2.0 * sigma2 * sigma2));
  }*/

  // First we will compute the mean of the two signal
  // since the intercorrelation needs 0-mean signals
  double mean1 = 0;
  double mean2 = 0;
  for(int i = 0; i < size1; ++i)
  {
    mean1 += data1[i];
  }
  for(int i = 0; i < size2; ++i)
  {
    mean2 += data2[i];
  }
  mean1 = mean1 / static_cast<double>(size1);
  mean2 = mean2 / static_cast<double>(size2);

  // Sample period
  double Te = 1.0 / static_cast<double>(sampleRate);

  // fill the signal vectors S1 and S2. We also
  // substract the mean of the signals to have
  // 0-means signal as the intercorrelation needs it
  for (int i = 0; i < size1; ++i)
  {
    S1.at<float>(i, 0) = data1[i] - mean1;
  }

  for (int i = 0; i < size2; ++i)
  {
    S2.at<float>(i, 0) = data2[i] - mean2;
  }

  // expand input image to optimal size
  // on the border add zero values
  cv::Mat S1padded, S2padded;
  int m1 = cv::getOptimalDFTSize(S1.rows);
  int n1 = cv::getOptimalDFTSize(S1.cols);
  int m2 = cv::getOptimalDFTSize(S2.rows);
  int n2 = cv::getOptimalDFTSize(S2.cols);
  int n = std::max(n1, n2);
  int m = std::max(m1, m2);
  std::cout << "Padded : " << m - S1.rows << "; " << n - S1.cols << std::endl;

  cv::copyMakeBorder(S1, S1padded, 0, m - S1.rows, 0, n - S1.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  cv::copyMakeBorder(S2, S2padded, 0, m - S2.rows, 0, n - S2.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

  // Add to the expanded signal another plane with zeros
  cv::Mat planes1[2] = {cv::Mat_<float>(S1padded), cv::Mat::zeros(S1padded.size(), CV_32F)};
  cv::Mat planes2[2] = {cv::Mat_<float>(S2padded), cv::Mat::zeros(S2padded.size(), CV_32F)};
  cv::Mat complexS1, complexS2;
  cv::merge(planes1, 2, complexS1);
  cv::merge(planes2, 2, complexS2);

  // compute the discret fourier transform
  // using FFT algorithm
  cv::dft(complexS1, complexS1);
  cv::dft(complexS2, complexS2);

  // planes[0] = Re(DFT(I)), planes[1] = Im(DFT(I))
  cv::split(complexS1, planes1);
  cv::split(complexS2, planes2);

  // Since we want the intercorrelation of the
  // signal and not the convolution we have to
  // transform g(t) -> g(-t) which leads to a
  // symmetrical operation according to X-axis
  std::cout << "Size planes2 : " << planes2[0].size() << std::endl;
  std::cout << "cols planes2 : " << planes2[0].cols << std::endl;
  std::cout << "rows planes2 : " << planes2[0].rows << std::endl;
  for (int i = 0; i < planes2[2].rows; ++i)
  {
    // conjugate
    planes2[1].at<float>(i, 0) = -planes2[1].at<float>(i, 0);
  }

  // Now we will multiply these two signal
  int nmult = std::min(m1, m2);
  cv::Mat planesProduct[2] = {cv::Mat::zeros(nmult, 1, CV_32F), cv::Mat::zeros(nmult, 1, CV_32F)};
  for(int i = 0; i < nmult; ++i)
  {
    float a = planes1[0].at<float>(i, 0);
    float b = planes1[1].at<float>(i, 0);
    float ap = planes2[0].at<float>(i, 0);
    float bp = planes2[1].at<float>(i, 0);

    planesProduct[0].at<float>(i, 0) = a * ap - b * bp;
    planesProduct[1].at<float>(i, 0) = a * bp + ap * b;
  }

  cv::Mat complexProduct;
  cv::merge(planesProduct, 2, complexProduct);
  cv::Mat interCorrelation;
  cv::idft(complexProduct, interCorrelation, cv::DFT_SCALE | cv::DFT_COMPLEX_OUTPUT);
  cv::Mat planeF[2];
  cv::split(interCorrelation, planeF);

  // Now we will apply a gaussian filter ron the signal
  // Since the 2 audio sensor are separated with a distance of
  // 10 cm, and the sound speed is 344m.s we want a resolution of 
  // 2 * distance / speed * sample_rate index
  double resolutionIndex = 1.0 / 2.0 * 0.1 / 349.0 * sampleRate;
  cv::Mat interCorrFiltered;
  cv::Size sizeG(0, 0);

  //cv::GaussianBlur(planeF[0], interCorrFiltered, sizeG, resolutionIndex, 0, cv::BORDER_DEFAULT);


  double moy = 0;
  double u = 0;
  int index = -1;

  for (int i = 0; i < planeF[0].rows; ++i)
  {
    double module = planeF[0].at<float>(i, 0) * planeF[0].at<float>(i, 0);

    if (module > u)
    {
      u = module;
      index = i;
    }
    moy += module;
  }

  moy /= static_cast<double>(planeF[0].rows);



  std::cout << "score1 : " << u / moy << std::endl;
  std::cout << "max value : " << u << " index : " << index << " time : "<< static_cast<float>(index) / static_cast<float>(sampleRate) << std::endl;

  return std::pair<double, double>(static_cast<float>(index) / static_cast<float>(sampleRate), u);
}

//---------------------------------------------------------------------------
double PolygonMetric(std::vector<Eigen::Vector2f> P1, std::vector<Eigen::Vector2f> P2)
{
  std::vector<Eigen::Vector2f> P1nP2 = ConvexPolygonIntersection(P1, P2);
  double areaP1nP2 = ComputeAreaConvexPolygon(P1nP2);
  double areaP1 = ComputeAreaConvexPolygon(P1);
  double areaP2 = ComputeAreaConvexPolygon(P2);

  double dist = areaP1nP2 / (areaP1 + areaP2 - areaP1nP2);

  return dist * 100;
}

//---------------------------------------------------------------------------
double ComputeAreaConvexPolygon(std::vector<Eigen::Vector2f> P)
{
  double area = 0;

  // We need at least 3 points, otherwise
  // the area is null
  if (P.size() > 2)
  {
    // close the loop
    P.push_back(P[0]);

    // Area is the sum of the oriented area of the triangles
    // which compose the polygon
    for (int i = 0; i < P.size() - 1; ++i)
    {
      Eigen::Vector2f V1 = P[i];
      Eigen::Vector2f V2 = P[i + 1];

      area = area + 1.0 / 2.0 * (V1(0) * V2(1) - V1(1) * V2(0));
    }
  }

  return area;
}

//---------------------------------------------------------------------------
std::vector<Eigen::Vector2f> ConvexPolygonIntersection(std::vector<Eigen::Vector2f> P1, std::vector<Eigen::Vector2f> P2)
{
  std::vector<Eigen::Vector2f> Q;
  const double epsilon = 1e-5;

  // Close the loop
  P1.push_back(P1[0]);
  P2.push_back(P2[0]);

  Eigen::Matrix<float, 2, 2> Rot;
  Rot << 0, -1,
         1, 0;

  // for each edges of the first polygon
  for (int i = 0; i < P1.size() - 1; ++i)
  {
    // Get the points of the current edge
    Eigen::Vector2f v1 = P1[i];
    Eigen::Vector2f v2 = P1[i + 1];
    Eigen::Vector2f v = v2 - v1;
    v = v / v.norm();

    // Orthogonal vector of the current edge such as
    // the other vertices are "below" the demi-plane
    Eigen::Vector2f n = Rot * v;

    // Affine equation of the line
    Eigen::Vector3f H;
    H << n(0), n(1), -n(0) * v1(0) - n(1) * v1(1);

    Q.resize(0);
    Q.clear();

    // for each edges in the second polygon
    for (int j = 0; j < P2.size() - 1; ++j)
    {
      // Get the points of the current edge
      Eigen::Vector2f u1 = P2[j];
      Eigen::Vector2f u2 = P2[j + 1];
      Eigen::Vector2f u = u2 - u1;
      u = u / u.norm();

      Eigen::Vector3f V, U;
      V << v(0), v(1), 0;
      U << u(0), u(1), 0;

      Eigen::Vector3f VcrossU = V.cross(U);
      double scoreAlignement = VcrossU.norm();

      Eigen::Vector2f X;
      bool isIntersectionComputed = false;

      if (scoreAlignement < epsilon)
      {
        // Vector aligned, no intersections
        //std::cout << "Vector aligned !" << std::endl;
      }
      else
      {
        // Compute intersection betwen U1U2 and V1V2
        Eigen::Vector2f n2 = Rot * u;
        Eigen::Vector3f H2;
        H2 << n2(0), n2(1), -n2(0) * u1(0) - n2(1) * u1(1);
        Eigen::Vector2f Y(-H(2), -H2(2));
        Eigen::Matrix<float, 2, 2> M;
        M << H(0), H(1),
             H2(0), H2(1);

        X = M.inverse() * Y;
        isIntersectionComputed = true;
      }

      double distU1 = H(0) * u1(0) + H(1) * u1(1) + H(2);
      double distU2 = H(0) * u2(0) + H(1) * u2(1) + H(2);

      // If U1 is inside H and U2 outside H
      if (distU1 < 0 && distU2 > 0)
      {
        Q.push_back(u1);
        if(isIntersectionComputed)
          Q.push_back(X);
        else
          std::cout << "Oups ?" << std::endl;
      }
      else if (distU1 > 0 && distU2 > 0)
      {
        // nothing
      }
      else if (distU1 > 0 && distU2 < 0)
      {
        if(isIntersectionComputed)
          Q.push_back(X);
        else
          std::cout << "Oups !" << std::endl;
      }
      else
      {
        Q.push_back(u1);
      }
      //std::cout << "Premier solution : " << X1 << std::endl;
      //std::cout << "Second solution : " << X2 << std::endl;
    }
    P2 = Q;
    if (P2.size() == 0)
      break;

    P2.push_back(P2[0]);
  }

  return Q;
}

//---------------------------------------------------------------------------
Eigen::Matrix<double, 3, 3> GetPositionInformationFromFund(Eigen::Matrix<double, 3, 3> Fund, Eigen::Matrix<double, 3, 3> Kprior1, Eigen::Matrix<double, 3, 3> Kprior2)
{
  // Compute E from fundamental and Kpriors
  Eigen::Matrix<double, 3, 3> E = Kprior2.transpose() * Fund * Kprior1;
  //std::cout << "Essential matrix : " << std::endl << E << std::endl;

  // now compute R and T from Essential matrix
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double, 3, 3> U = svd.matrixU();
  Eigen::Matrix<double, 3, 3> V = svd.matrixV();
  Eigen::Matrix<double, 3, 3> D;
  double singularValue = (svd.singularValues()(0) + svd.singularValues()(1)) / 2.0;
  D << svd.singularValues()(0), 0, 0, 0, svd.singularValues()(1), 0, 0, 0, svd.singularValues()(2);

  /*std::cout << "Essential singular values are:" << std::endl << D << std::endl;
  std::cout << "Essential left singular vectors are the columns of the thin U matrix:" << std::endl << U << std::endl;
  std::cout << "Essential right singular vectors are the columns of the thin V matrix:" << std::endl << V << std::endl;*/

  Eigen::Matrix<double, 3, 3> W;
  W << 0, -1, 0,
    1, 0, 0,
    0, 0, 1;

  // Compute R and rescale R so that det(R) = 1
  Eigen::Matrix<double, 3, 3> R = U * W * V.transpose();
  R = std::cbrt(1.0 / R.determinant()) * R;

  double phi = std::atan2(R(2, 1), R(2, 2)) * 180 / 3.14159265359;
  double theta = -std::sin(R(2, 0)) * 180 / 3.14159265359;
  double psi = -std::atan2(R(1, 0), R(0, 0)) * 180 / 3.14159265359;

  // We should not have a big angle between the two views. If it is the case
  // we assume that it is because we chose the wrong solution among the 4
  if (std::abs(phi) > 100 || std::abs(theta) > 100 || std::abs(psi) > 100)
  {
    R = U * W.transpose() * V.transpose();
    R = std::cbrt(1.0 / R.determinant()) * R;

    phi = std::atan2(R(2, 1), R(2, 2)) * 180 / 3.14159265359;
    theta = -std::sin(R(2, 0)) * 180 / 3.14159265359;
    psi = -std::atan2(R(1, 0), R(0, 0)) * 180 / 3.14159265359;
  }

  //std::cout << "Rotation angles : [" << phi << ";" << theta << ";" << psi << "]" << std::endl;

  // Compute T
  Eigen::Matrix<double, 3, 1> T;
  T << U(0, 2), U(1, 2), U(2, 2);

  Eigen::Matrix<double, 3, 3> Id;
  Id << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;

  Eigen::Quaternion<double> q2(R);
  Eigen::Quaternion<double> q1(Id);

  Eigen::Quaternion<double> qdemi = q2.slerp(0.5, q1);
  Eigen::Matrix<double, 3, 3> Rdemi = qdemi.toRotationMatrix();

  phi = std::atan2(R.transpose()(2, 1), R.transpose()(2, 2)) * 180 / 3.14159265359;
  theta = -std::sin(R.transpose()(2, 0)) * 180 / 3.14159265359;
  psi = -std::atan2(R.transpose()(1, 0), R.transpose()(0, 0)) * 180 / 3.14159265359;

  std::vector<double> output;
  output.push_back(phi);
  output.push_back(theta);
  output.push_back(psi);

  return R;
}
