#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/tuple/tuple.hpp>
#include "LCSNSType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CNormalShootingLCSNS
	{
	public:
		CNormalShootingLCSNS();

		static std::string getKeyNumNeibP()			{ return boost::algorithm::to_upper_copy(std::string("NormalShootingLCSNS.NumNeibP")); }
		static std::string getKeyLenFact2Move()		{ return boost::algorithm::to_upper_copy(std::string("NormalShootingLCSNS.LF2M")); }
		
		void determineCorrespondencePairSet(
			const CorrespondencePointSet& vTgtCorPointSet, const hivePointCloud::CPointCloud* vSourcePointCloud,
			const Eigen::Matrix3d& vR, const hiveRegistration::ColVector3& vT,
			CorrespondencePairSet& voCorPairSet);
		
		void determineCorrespondencePairSet( const CorrespondencePointSet& vSrcCorPointSet, const CorrespondencePointSet &vTgtCorPointSet, CorrespondencePairSet& voCorPairSet, std::vector<unsigned int>& voIntersectedPointSet );

	private:
		int m_MatrixGenerationType;
		double m_ZDistThresholdFactor;
		double m_LenFact2Move;
		unsigned int m_NumNeibP;
		unsigned int m_SizeExtendMat;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pKNN;

		struct SRegionIndexRange
		{
			int ColBegin;
			int ColEnd;
			int RowBegin;
			int RowEnd;
		};

		void __parseConfig(void);
		void __computeControlPointsByMatrix( const hivePointCloud::CPointCloud& vTransformedPointCloud, int vSampleIdx, ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const;
		void __genControlPointsMatrix( const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, int vSampleIdx, ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const;
		void __removeRedundantPoints( const CandCtrlPnts& vCandCtrlPnts, int vSampleIdx, ControlPoints& voSrcCtrlPnts, bool& voIsUseful ) const;
		void __initControlPoints( const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, int vSampleIdx, CandCtrlPnts& voControlPoints ) const;
		void __computeIntersectPointWithEnoughSurfacePatchs( 
			const SRegionIndexRange& vIndexRange, const ColVector3& vRayOrigin, const ColVector3& vDir, const ControlPoints& vControlPoints, 
			std::pair<int, int>& voIntersectedSurfIdx, ColVector2& voUV, bool& voIsIntersected) const;
	};
}