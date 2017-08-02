#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include "ControlPointsData.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CPairEstimationNormal2Plane
	{
	public:
		CPairEstimationNormal2Plane();

		static std::string getKeySurfacePatchSize() { return boost::algorithm::to_upper_copy(std::string("PairEstimationNormal2Plane.SPS")); }
		static std::string getKeyPairSquareDistFactor() { return boost::algorithm::to_upper_copy(std::string("PairEstimationNormal2Plane.PSDF")); }
		static std::string getKeyCorPointsBuildingType() { return boost::algorithm::to_upper_copy(std::string("PairEstimationNormal2Plane.CPBT")); }

		void determineCorrespondencePairSet(
			const CorrespondencePointSet& vTgtCorPointSet, const hivePointCloud::CPointCloud* vSourcePointCloud,
			const Eigen::Matrix3d& vR, const ColVector3& vT,
			CorrespondencePairSet& voCorPairSet);
		
		void determineCorrespondencePairSet(const CorrespondencePointSet& vSrcCorPointSet, const MatrixControlPoints &vTgtPoints, 
			CorrespondencePairSet& voCorPairSet, std::vector<unsigned int>& voIntersectedPointSet);
	
	private:
		struct SRegionIndexRange
		{
			int ColBegin;
			int ColEnd;
			int RowBegin;
			int RowEnd;
		};

		int m_SurfacePathSize;
		double m_PairSquareDistFactor;
		double m_MaxRayMarchLength;

		void __computePatchIndexRange(const int vCenterIndex, const int vTotal, int& voBegin, int& vEnd) const;
		void __computeMaxRayMarchLength(const SControlPointsMatrixInfo &vMatrix);
		void __computeIntersectPointWithEnoughSurfacePatchs( const SRegionIndexRange& vIndexRange, const ColVector3 vRayOrigin, const ColVector3 vDir, const hiveRegistration::MatrixControlPoints &vControlPoints, std::pair<int, int>& voIdxPair, hiveRegistration::ColVector2& voUV, bool& voIsIntersected);
		void __computeControlPointsByMatrix( const hivePointCloud::CPointCloud* vSourcePointCloud, const Eigen::Matrix3d& vR, const ColVector3& vT, MatrixControlPoints& voMatrixControlPoints );
		void __genControlPointsMatrix(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, MatrixControlPoints& voControlPoints) const;
		void __removeRedundantPoints(MatrixControlPoints& vioControlPoints) const;
		void __initControlPoints(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, MatrixControlPoints& voControlPoints) const;
		void __computeControlPointsByClosestPoint(const CorrespondencePointSet& vTargetCorrespondencePointSet, 
			const hivePointCloud::CPointCloud* vSourcePointCloud, const Eigen::Matrix3d& vR, const ColVector3& vT, MatrixControlPoints& voMatrixControlPoints);

		void __parseConfig(void);
	};
}