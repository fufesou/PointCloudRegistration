#pragma once
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "ControlPointsData.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CSquareCellsInitialization
	{
	public:
		CSquareCellsInitialization(void);
		~CSquareCellsInitialization(void);

		static std::string getSizeCtrlMatrixRow() { return boost::algorithm::to_upper_copy(std::string("SquareCellsInitialization.SizeCMRow")); }
		static std::string getSizeCtrlMatrixCol() { return boost::algorithm::to_upper_copy(std::string("SquareCellsInitialization.SizeCMCol")); }

#ifdef _DEBUG
		static std::string getOutputFilePathRoot() { return boost::algorithm::to_upper_copy(std::string("SquareCellsInitialization.OFPathRoot")); }
#endif

		void genControlPointsMatrix(const hivePointCloud::CPointCloud* vPointCloud, SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints);
		void refreshControlPointsMatrix(SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints) const
		{
			__genControlPointsMatrix(voControlPointsMatrix, voControlPoints);
		}

private:
		const hivePointCloud::CPointCloud* m_pPointCloud;
		void				__genControlPointsMatrix(SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints) const;
		void				__removeRedundantPoints(const SControlPointsMatrixInfo& vControlPointsMatrix, MatrixControlPoints& vioControlPoints) const;
		void				__initControlPointsMatrix(const RegionDim3& vRegion, SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints) const;
		RegionDim3			__computeRegion() const;

#ifdef _DEBUG
		void				__storeControlPointsMatrix2File(const MatrixControlPoints& vControlPoints, int vIndexMaxRows, int vIndexMaxCols) const;
#endif

	};
}

