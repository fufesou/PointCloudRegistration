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
	class CNormalShootingLCSNSClosest
	{
	public:
		CNormalShootingLCSNSClosest();

		static std::string getKeyClosestDistFact() { return boost::to_upper_copy(std::string("NormalShootingLCSNSClosest.CDF")); }
		static std::string getKeyLenFact2Move()   { return boost::to_upper_copy(std::string("NormalShootingLCSNSClosest.LF2M")); }
		
		void setTgtPointCloud(const hivePointCloud::CPointCloud* vTgtPointCloud);
		void determineCorrespondencePairSet( 
			const CorrespondencePointSet& vSrcCorPntSet, 
			const Eigen::Matrix3d& vR, const ColVector3& vT, 
			CorrespondencePairSet& voCorPairSet );
		void determineCorrespondencePairSet( 
			const CorrespondencePointSet& vSrcCorPntSet, 
			const CorrespondencePointSet &vTgtCorPntSet,
			CorrespondencePairSet& voCorPairSet, 
			std::vector<unsigned int>& voIntersectedPointSet );

	private:
		int m_MatRows;
		int m_MatCols;
		double m_LenFact2Move;
		double m_ClosestDistFactor;

		const hivePointCloud::CPointCloud* m_pTgtPntCld;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pKNN;

		struct SRegionIndexRange
		{
			int ColBegin;
			int ColEnd;
			int RowBegin;
			int RowEnd;
		};

		void __parseConfig(void);
		void __computeIntersectPointWithEnoughSurfacePatchs( 
			const SRegionIndexRange& vIndexRange, const ColVector3& vRayOrigin, const ColVector3& vDir, const ControlPoints& vControlPoints, 
			std::pair<int, int>& voIntersectedSurfIdx, ColVector2& voUV, bool& voIsIntersected) const;
	};
}