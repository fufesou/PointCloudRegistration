#pragma once
#include <utility>
#include <map>
#include <boost/serialization/access.hpp>
#include <boost/shared_ptr.hpp>
#include "PointCloud.h"
#include "ICPType.h"
#include "BaseCorrespondenceEstimation.h"

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveSearch
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CLCPTriangle;
	class CIndexSubset;
	class CSpinImagesGenerator;
	class CSimilarityTriangle;

	class CCorrespondenceEstimationASI : public CBaseCorrespondenceEstimation
	{
		friend boost::serialization::access;

	public:
		CCorrespondenceEstimationASI();

		static std::string getClassSig()                      { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI")); }
		static std::string getKeyExtendGaussianRange()        { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.EGR")); }
		static std::string getKeySimilaritySampleRangeMin()   { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.SSRMIN")); }
		static std::string getKeySimilaritySampleRangeMax()   { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.SSRMAX")); }
		static std::string getKeySpinDistThreshold()          { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.SDT")); }
		static std::string getKeyNumMatchSection()            { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.NMS")); }
		static std::string getKeyMaxNumMatchPntEachStep()     { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.MNMPES")); }
		static std::string getKeyNumSpinNeibs()               { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.NSN")); }
		static std::string getKeyExtendGaussianRangeFactor()  { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.EGRF")); }
		static std::string getKeyUseSimTriangle()             { return boost::to_upper_copy(std::string("CorrespondenceEstimationASI.UseSimTriangle")); }

	protected:
		virtual ~CCorrespondenceEstimationASI(void) {}

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(
			const CorrespondencePointSet& vTgtCorPointSet, 
			const Eigen::Matrix3d& vR, 
			const ColVector3& vT, 
			CorrespondencePairSet& vioCorrespondencePairs) override;

	private:
		double m_ExtGssRange;
		double m_SimSmpRange[2];
		double m_SpinDistThreshold;
		double m_NumMatchSection;
		double m_ExtendGssRangeFactor;
		int m_UseSimTri;
		unsigned int m_ImageWidth;
		unsigned int m_ImageHeight;
		unsigned int m_NumSpinNeibs;
		unsigned int m_MaxNumMatchPntEachStep;
		std::vector<std::pair<unsigned int, std::vector<unsigned int> > > m_VecIdxGrpPairSet;
		std::map<unsigned int, boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1> > > m_MapLoopImageData;

		void __parseCtrlParams(void);
		void __resetSamplePointSetByRange(CIndexSubset* vioSrcSubset, CIndexSubset* vioTgtSubset) const;
		void __setSimilaritySamplePointSet(const CIndexSubset* vSamplePoints, std::vector<unsigned>& voSubset) const;
		void __matchFirstCall(const CIndexSubset* vLoopIdxSet);

		void __doLoopMatch(
			CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
			std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet
			);

		bool __comCurRT(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vIdxGroupSet, const CIndexSubset* vLoopIdxSet, CLCPTriangle& vLCPTriangle, Eigen::Matrix3d& voR, Eigen::Vector3d& voT);
		void __comCurMatchItrRange(const CIndexSubset* vMatchIdxSet, double vSectionRatio, std::vector<unsigned int>::const_iterator vioMatchItrRange[2]) const;
		void __comCurMatchSpinImgs(const CIndexSubset* vMatchIdxSet, std::vector<unsigned int>::const_iterator vMatchItrRange[2], Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voImgs) const;
		void __comCurGssCurvRange(const CIndexSubset* vMatchIdxSet, std::vector<unsigned int>::const_iterator vMatchItrRange[2], double voCurCurvRange[2]) const;
		void __setCurLoopIdxRange(const double vCurCurvRange[2], const CIndexSubset* vLoopIdxSet, unsigned int voIdxRange[2]) const;
		double __comStepRatio(unsigned int vNumMatchSubset) const;

		CIndexSubset* __sampleSourcePointCloud(void);
		boost::shared_ptr<CSimilarityTriangle> __createSimilarityTriangle(const CIndexSubset* vLoopIdxSet, const CIndexSubset* vMatchIdxSet) const;
		boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1> > __getOrCreateCurCentImg(unsigned int vImgIdx, const CIndexSubset* vLoopIdxSet);
	};
}