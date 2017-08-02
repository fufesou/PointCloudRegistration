#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/algorithm/string.hpp>
#include "BaseCorrespondenceRejection.h"

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CRejectionCentroidDist : public CBaseCorrespondenceRejection
	{
		friend boost::serialization::access;

	public:
		CRejectionCentroidDist(void);

		static std::string getClassSig()		      { return boost::to_upper_copy(std::string("RejectionCentroidDist")); }
		static std::string getKeyMaxSearchNum()       { return boost::to_upper_copy(std::string("RejectionCentroidDist.MSN")); }
		static std::string getKeyMaxSearchDist()      { return boost::to_upper_copy(std::string("RejectionCentroidDist.MSD")); }
		static std::string getKeyCriticalDistFactor() { return boost::to_upper_copy(std::string("RejectionCentroidDist.CDF")); }

	protected:
		~CRejectionCentroidDist(void) { }

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;

	private:
		int    m_DefaultNumNeighbors;
		double m_DefaultMaxSearchRadius;
		double m_DefaultCriticalDistFactor;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pSrcKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pTgtKNN;

		void __initKNN();
		void __comDiffSquaredDist2CentroidVec(const CorrespondencePairSet& vAllCorrespondencePairs, std::vector<double>& voDiffSquaredDistVec);
		void __computeCentroid(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, const std::vector<unsigned int>& vNeighbors, double* voCentroid);
		void __searchNeighbors(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, std::vector<unsigned int>& voNeighbors);
		void __searchDesignedNumNeighbors(const double* vTgtPnt, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, std::vector<unsigned int>& voNeighbors);
		void __searchDesignedRadiusNeighbors(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, std::vector<unsigned int>& vioNeighbors);
		void __reject(const CorrespondencePairSet& vAllCorPairs, const std::vector<double>& vDistSet, double vMeanDist, double vStdVariance, CorrespondencePairSet& voValidCorPairs) const;
		double __comSquaredDist2Centroid(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN);

		struct DistCmp 
		{
			const double* Ptr2Pnt;
			const double* Ptr2PntCldPos;

			DistCmp(const double* vPnt, const hivePointCloud::CPointCloud* vPointCloud);

			bool operator()(unsigned int vIdx, double vDesignedDist);
		};
	};
}