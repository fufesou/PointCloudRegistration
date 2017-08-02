#pragma once
#include "Singleton.h"
#include "ICPType.h"


namespace hiveRegistration
{
	struct SCtrolMatrixData
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		struct CtrlMatrix
		{
			ColVector3				  Original;
			Eigen::Matrix3d			  MatWorld2Local;
			RegionDim3                Range;
			std::pair<double, double> SizeEachBlock;
		};

		typedef std::vector<CtrlMatrix, Eigen::aligned_allocator<CtrlMatrix> > VecCtrlMatrix;

		std::pair<int, int> RowAndCol;
		VecCtrlMatrix       VecCtrlMatData;

	private:
		friend hiveCommon::CSingleton<SCtrolMatrixData>;
		SCtrolMatrixData() {}
	};
}	