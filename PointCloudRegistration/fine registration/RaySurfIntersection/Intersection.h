#pragma once
#include <Eigen/core>
#include "..\ICPType.h"


bool intersect(const Eigen::Matrix<double, 4, 4> vCtrlPnts[3], const Eigen::Matrix<double, 3, 1> &vRayPnt, const Eigen::Matrix<double, 3, 1> &vRayDir, 
			   Eigen::Matrix<double, 2, 1> &voEndMatrix, hiveRegistration::ColVector3 &voIntersectPoints);
bool intersect(const Eigen::Matrix<double, 4, 4> vCtrlPnts[3], const Eigen::Matrix<double, 3, 1> &vRayPnt, const Eigen::Matrix<double, 3, 1> &vRayDir, 
			   Eigen::Matrix<double, 2, 1> &voUV);
