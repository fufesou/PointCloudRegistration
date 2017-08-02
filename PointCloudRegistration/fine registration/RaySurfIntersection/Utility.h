#pragma once
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "..\ICPType.h"

void genBSplineVerticesFile(const char *vFileName, const Eigen::Matrix4d vCtlPnts[3]);
void fillPlyHeader(std::ofstream &voOutFile, unsigned int vNumPoints);
void export2Ply(const char *vFileName, const hiveRegistration::VecColVector3 &vPointSet);