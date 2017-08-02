#pragma once
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include "KNNSearch.h"
#include "ICPType.h"


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
		static std::string getKeyEarlyReturnRatio()   { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.ERR")); }
		static std::string getKeyAcceptMatchRatio()   { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.AMR")); }
		static std::string getKeySimTriFactor()       { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.STF")); }
		static std::string getKeySamplerStrID()       { return boost::algorithm::to_upper_copy(std::string("LCPTriangle.SSI")); }

		CLCPTriangle(const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud);

		bool comRotationAndTranslation(std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, double vUnitSquareDist, Eigen::Matrix3d& voR, Eigen::Vector3d& voT);
		bool comRTWithLastElement(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, double vUnitSquareDist, Eigen::Matrix3d& voR, Eigen::Vector3d& voT);

	private:
		double m_SimilarTriangleFactor;
		double m_TriangleEdgeFactor;
		double m_CongruentFactor;
		double m_EarlyReturnRatio;
		double m_AcceptMatchRatio;
		double m_UnitSquareDist;
		std::string m_SampleStrID;

		const hivePointCloud::CPointCloud& m_LoopPointCloud;
		const hivePointCloud::CPointCloud& m_MatchPointCloud;
		SamplePointSet m_LoopSamplePointSet;
		hiveCommon::CKNNSearch m_MatchKNN;

		void __parseParams(void);
		void __comRTWith3PntPairs(const std::pair<unsigned, unsigned> vPntPairs[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT) const;
		unsigned __findNumPntsInTolerance(const Eigen::Matrix3d& vR, const Eigen::Vector3d& vT);
		unsigned __comOptimalRT(
			const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, 
			unsigned vI, unsigned vK, unsigned vJ, 
			Eigen::Matrix3d& voR, Eigen::Vector3d& voT);
	};
}