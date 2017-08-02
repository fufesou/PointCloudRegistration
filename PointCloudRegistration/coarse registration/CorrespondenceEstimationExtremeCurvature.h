#pragma once
#include <boost/serialization/access.hpp>
#include <boost/shared_ptr.hpp>
#include "PointCloud.h"
#include "ICPType.h"
#include "BaseCorrespondenceEstimation.h"

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CSimilarityTriangle;
	class CIndexSubset;

	class CCorrespondenceEstimationExtremeCurvature : public CBaseCorrespondenceEstimation
	{
		friend boost::serialization::access;

	public:
		CCorrespondenceEstimationExtremeCurvature();

		static std::string getClassSig()                  { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature")); }
		static std::string getKeyFistTolerance()          { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.FT")); }
		static std::string getKeySecondTolerance()        { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.ST")); }
		static std::string getKeyMatchGaussianTolerance() { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.MGT")); }
		static std::string getKeyCentroidNumPnts()        { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.CNP")); }
		static std::string getKeySqureDist2CentFact()     { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.SD2CF")); }
		static std::string getKeySampleKNNRangeMin()      { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.SKNNRMIN")); }
		static std::string getKeySampleKNNRangeMax()      { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.SKNNRMAX")); }
		static std::string getKeyValidSampleRangeMin()    { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.VSRMIN")); }
		static std::string getKeyValidSampleRangeMax()    { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.VSRMAX")); }
		static std::string getKeyUseDist2CentRestrection(){ return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.UseDist2CentRestrection")); }
		static std::string getKeyUseSimTriangle()         { return boost::to_upper_copy(std::string("CorrespondenceEstimationExtremeCurvature.UseSimTriangle")); }

	protected:
		virtual ~CCorrespondenceEstimationExtremeCurvature(void) {}

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(
			const CorrespondencePointSet& vTgtCorPointSet, 
			const Eigen::Matrix3d& vR, 
			const ColVector3& vT, 
			CorrespondencePairSet& vioCorrespondencePairs) override;

	private:
		double m_FirstTol;
		double m_SecondTol;
		double m_GssTol;
		double m_SampleRatioRange[2];
		double m_SampleKNNRange[2];
		double m_Dist2CentFact;
		double m_UnitSquareDist;
		unsigned m_CentNumNeibs;
		int m_UseDist2CentRestrection;
		int m_UseSimTri;

		boost::shared_ptr<CSimilarityTriangle>    m_pSimTriangle;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pLoopKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pMatchKNN;
		const hivePointCloud::CPointCloud* m_pLoopPointCloud;
		const hivePointCloud::CPointCloud* m_pMatchPointCloud;

		void __parseCtrlParams(void);
		void __setLoopMatchStructData(const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN);
		void __setSimMeasurement(const std::vector<unsigned>& vLoopKNN, const std::vector<unsigned>& vMatchKNN);
		void __constructPointCloudKNN(const hivePointCloud::CPointCloud& vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch>& voKNN);
		void __restrictCentroid(unsigned vLoopIdx, std::vector<unsigned>& vioCorCandSet);
		ColVector3 __computeNeiborsCentroid(unsigned vIdx, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, const hivePointCloud::CPointCloud& vPointCloud);
		CIndexSubset* __sampleSourcePointCloud(void);
		void __restrictLoopSamplePoints(CIndexSubset* vioInitSamplePoints, std::vector<unsigned>& voLoopKNN, double voRetainRange[2]) const;
		void __restrictMatchSamplePoints(CIndexSubset* vioInitSamplePoints, std::vector<unsigned>& voMatchKNN, const double vRetainRange[2]) const;

		void __doLoopMatch(
			CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
			std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet
			);

		bool __matchPoint(unsigned int vLoopIdx, const std::vector<unsigned int>& vMatchIdxSet, std::vector<unsigned int>& voCandidateSet);

		inline bool __isWithinTolerance(double vLK1, double vLK2, double vRK1, double vRK2)
		{
			return ((vLK1-vRK1)*(vLK1-vRK1) + (vLK2-vRK2)*(vLK2-vRK2)) <= m_FirstTol && 
				(abs(( vLK1-vRK1 )*( vLK2-vRK2 )) <= m_SecondTol);
		}
		inline bool __isWithinGaussianTolerance(double vLGssCurv, double vRGssCurv)
		{
			return fabs(vLGssCurv - vRGssCurv) < m_GssTol;
		}
	};
}