#pragma once 
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "Singleton.h"
#include "ICPType.h"

namespace hiveRegistration
{
	typedef Eigen::Matrix<double, 2, 1> ColVector2;

	struct SControlPointsMatrixInfo
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		std::pair<int, int>		  NumTotalRowAndCols;
		std::pair<double, double> SizeEachBlock;
		RegionDim3				  Range;

	private:
		friend hiveCommon::CSingleton<SControlPointsMatrixInfo>;
		SControlPointsMatrixInfo() {}
	};
	typedef Eigen::Matrix<VecColVector3, Eigen::Dynamic, Eigen::Dynamic> MatrixControlPoints;
}