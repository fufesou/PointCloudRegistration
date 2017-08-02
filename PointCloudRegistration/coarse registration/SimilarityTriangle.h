#pragma once
#include <utility>
#include <boost/algorithm/string.hpp>
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
	class CIndexSubset;

	class CSimilarityTriangle
	{
	public:
		CSimilarityTriangle(const CIndexSubset* vLoopIdxSet, const CIndexSubset* vMatchIdxSet);
		CSimilarityTriangle(const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud, 
			const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN);
		~CSimilarityTriangle();

		static std::string getKeySimSquaredDistFact() { return boost::algorithm::to_upper_copy(std::string("SimilarityTriangle.SSDF")); }
		static std::string getKeyNormCosDist()        { return boost::algorithm::to_upper_copy(std::string("SimilarityTriangle.NCD")); }
		static std::string getKeyNumNeighbor()        { return boost::algorithm::to_upper_copy(std::string("SimilarityTriangle.NN")); }
		static std::string getKeyGssCurvTolerance()   { return boost::algorithm::to_upper_copy(std::string("SimilarityTriangle.GCT")); }

		void fillValidCandidate(
			unsigned int vLoopIdx, const std::vector<unsigned int>& vCorCandidateSet, 
			std::vector<unsigned>& voValidCandSet) const;

	private:
		double m_SquareDistThreshold;
		double m_NormCosDist;
		double m_GssCurvTol;
		std::vector<unsigned> m_LoopSampleKNNIdxSet;
		std::vector<unsigned> m_MatchSampleKNNIdxSet;
		const hivePointCloud::CPointCloud& m_LoopPointCloud;
		const hivePointCloud::CPointCloud& m_MatchPointCloud;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pLoopSampleKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pMatchSampleKNN;

		void __init(void);
		void __parseCtrlParams(void);
		void __convSubIdxSet2WholeIdxSet(const std::vector<unsigned>& vWholeSampleIdxSet, std::vector<unsigned>& vioIdxSet) const;

		void __computeNeibSquareDistSet(
			unsigned vIdx, 
			const hivePointCloud::CPointCloud& vPointCloud,
			const std::vector<unsigned>& vIdxSet, 
			hiveCommon::CKNNSearch* vKNN, 
			std::vector<std::pair<double, unsigned> >& voNeibDist) const;
		void __findValidPntThroughCongruentTriangle(
			const std::vector<std::pair<double, unsigned> >& vLoopNeibDist, 
			const std::vector<std::pair<double, unsigned> >& vMatchNeibDist,
			std::vector<unsigned>& voValidPntSet) const;

		boost::shared_ptr<hiveCommon::CKNNSearch> 
			__constructSampleKNN(const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned>& vIdxSet, int vNumNeighbor);

		bool __isWithinNormTolerance(unsigned vLoop0, unsigned vLoop1, unsigned vMatch0, unsigned vMatch1) const;
		bool __isWithinGssTolerance(unsigned vLoopIdx, unsigned vMatchIdx) const;
	};
}