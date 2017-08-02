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
	class CIndexSubset;

	class CCorrespondenceEstimationSpinImages : public CBaseCorrespondenceEstimation
	{
		friend boost::serialization::access;

	public:
		CCorrespondenceEstimationSpinImages();

		static std::string getClassSig()                  { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages")); }
		static std::string getKeyMatchGaussianTolerance() { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.MGT")); }
		static std::string getKeyValidSampleRangeMin()    { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.VSRMIN")); }
		static std::string getKeyValidSampleRangeMax()    { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.VSRMAX")); }
		static std::string getKeySampleKNNRangeMin()      { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.SKNNRMIN")); }
		static std::string getKeySampleKNNRangeMax()      { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.SKNNRMAX")); }
		static std::string getKeySpinDistThreshold()      { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.SDT")); }
		static std::string getKeyPCADimension()           { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.PCADim")); }
		static std::string getKeyNumSpinNeibs()           { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.NSN")); }
		static std::string getKeyUseSimTriangle()         { return boost::to_upper_copy(std::string("CorrespondenceEstimationSpinImages.UseSimTriangle")); }

	protected:
		virtual ~CCorrespondenceEstimationSpinImages(void) {}

		virtual void _setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud) override;
		virtual void _determineCorrespondencesV(
			const CorrespondencePointSet& vTgtCorPointSet, 
			const Eigen::Matrix3d& vR, 
			const ColVector3& vT, 
			CorrespondencePairSet& vioCorrespondencePairs) override;

	private:
		double m_GssTol;
		double m_SampleRatioRange[2];
		double m_SampleKNNRange[2];
		double m_SpinDistThreshold;
		unsigned m_NumSpinNeib;
		unsigned m_PCADimension;
		int m_UseSimTri;

		void __parseCtrlParams(void);
		void __restrictLoopSamplePoints( CIndexSubset* vioInitSamplePoints, std::vector<unsigned>& voLoopKNN, double voRetainRange[2]) const;
		void __restrictMatchSamplePoints( CIndexSubset* vioInitSamplePoints, std::vector<unsigned>& voMatchKNN, const double vRetainRange[2]) const;
		void __doLoopMatch(
			CIndexSubset* vLoopIdxSet, CIndexSubset* vMatchIdxSet, 
			std::vector<unsigned int>& voMatchedLoopIdxSet, std::vector<unsigned int>& voMatchedMatchIdxSet
			);

		CIndexSubset* __sampleSourcePointCloud(void);
	};
}