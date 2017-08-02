#pragma once
#include <Eigen/core>

double computeDerivativeU(const Eigen::Matrix4d &vTempMatrix, const double vParaU, const double vParaV);
double computeDerivativeV(const Eigen::Matrix4d &vTempMatrix, const double vParaU, const double vParaV);
