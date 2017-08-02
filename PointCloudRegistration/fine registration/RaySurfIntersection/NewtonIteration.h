#pragma once
#include <boost/tuple/tuple.hpp>
#include <Eigen/core>
#define EPSILON 1E-06

bool useNewtonSolveEqution(const boost::tuple<double, double, double> &vCoordinate, const boost::tuple<double, double, double> &vDirection, boost::tuple<double, double, double> &vInitial, 
						   const Eigen::Matrix4d &vNNx, const Eigen::Matrix4d &vNNy, const Eigen::Matrix4d &vNNz);