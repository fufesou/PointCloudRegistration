#pragma once
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include "KNNSearch.h"


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
	class CLCPTriangle
	{
	public:
		static std::string getKeyCongruentFactor()    { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.CF")); }
		static std::string getKeyTriangleEdgeFactor() { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.TEF")); }
		static std::string getKeyMatchRatio()         { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.MR")); }
		static std::string getKeySimTriFactor()       { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.STF")); }

		CLCPTriangle(const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud, const std::vector<unsigned>& vLoopSamplePnts, const std::vector<unsigned>& vMatchSamplePnts);

		void comRotationAndTranslation(std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairdPntGroup, double vUnitSquareDist, Eigen::Matrix3d& voR, Eigen::Vector3d& voT);

	private:
		double m_SimilarTriangleFactor;
		double m_TriangleEdgeFactor;
		double m_CongruentFactor;
		double m_MatchRatio;
		double m_UnitSquareDist;

		const std::vector<unsigned>& m_LoopPnts;
		const std::vector<unsigned>& m_MatchPnts;
		const hivePointCloud::CPointCloud& m_LoopPointCloud;
		const hivePointCloud::CPointCloud& m_MatchPointCloud;
		hiveCommon::CKNNSearch m_LoopKNN;
		hiveCommon::CKNNSearch m_MatchKNN;

		void __parseParams(void);
		void __findEdgeVertexSet(const std::pair<unsigned, std::vector<unsigned> >& vPntGroupA, const std::pair<unsigned, std::vector<unsigned> >& vPntGroupB, std::pair<unsigned, std::vector<unsigned> > voPrePairedAB[2]) const;
		void __findEdgeVertexSet(const std::pair<unsigned, std::vector<unsigned> > vPrePairedAB[2], const std::pair<unsigned, std::vector<unsigned> >& vPntGroupC, std::pair<unsigned, std::vector<unsigned> > voTriPntGroupPair[3]) const;
		void __comRTWith3PntPairs(const std::pair<unsigned, unsigned> vPntPairs[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT) const;
		unsigned __findNumPntsInTolerance(const Eigen::Matrix3d& vR, const Eigen::Vector3d& vT);
		unsigned __comOptimalRT(const std::pair<unsigned, std::vector<unsigned> > vTriPntGroup[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT);
	};
}