#pragma once
#include <Eigen/core>

Eigen::Matrix<double , 3, 1> computeData(const Eigen::Matrix4d &vNNx, const Eigen::Matrix4d &vNNy, const Eigen::Matrix4d &vNNz, double vInitialU, double vInitialV, double vInitialT, 
										 const double vA, const double vB, const double vC, const double vF1, const double vF2, const double vF3);