#pragma once
#include <boost/tuple/tuple.hpp>
#include <boost/algorithm/string.hpp>
#include "ICPType.h"
#include "ICPConstGlobleValue.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CBaseSampler;
	class CBaseCorrespondenceEstimation;
	class CBaseCorrespondenceRejection;
	class CBaseTransformationEstimation;
	class CDefaultConvergenceCriteria;

	class CIterativeFit
	{
	public:
		static std::string getKeySamplePointCloud(void) { return boost::to_upper_copy(std::string("IterativeFit.SPC")); }

		CIterativeFit() : m_pSourcePointCloud(NULL) , m_pTargetPointCloud(NULL) { }

		// these functions are all temporary, use a class named CComputationParams which is set by config file and user interface to control registration
		void setFitClasses(const std::string& vPointSamplerSig, const std::string& vCorrespondenceEsitmationSig, const std::string& vCorrespondenceRejectionSig, const std::string& vTransformationEsitmationSig);
		void setSampler(const std::string& vPointSamplerSig);
		void setCorrespondenceEstimation(const std::string& vCorrespondenceEsitmationSig);
		void setCorrespondenceRejection(const std::string& vCorrespondenceRejectionSig);
		void setTransformationEstimation(const std::string& vTransformationEsitmationSig);
		void setConvergenceCriterial(const std::string& vConvergenceCriterialSig);

		boost::tuple<Eigen::Matrix3d, ColVector3, std::string> fit(
			const hivePointCloud::CPointCloud& vSourcePointCloud, 
			const hivePointCloud::CPointCloud& vTargetPointCloud,
			const Eigen::Matrix3d& vInitR, const ColVector3& vInitT);

	protected:
		enum EFitFlag { FIT_END, FIT_NOT_END };

		virtual void _updateV();
		virtual bool _isCorrespondencePointSetValidV();
		virtual EFitFlag _fitStepV(unsigned int vIterationCounter, Eigen::Matrix3d& vioR, ColVector3& vioT);

	private:
		const hivePointCloud::CPointCloud* m_pSourcePointCloud;
		const hivePointCloud::CPointCloud* m_pTargetPointCloud;

		CorrespondencePointSet		   m_SampleSubset;
		CBaseSampler*                  m_pPointSampler;
		CBaseCorrespondenceEstimation* m_pCorrespondenceEstimation;
		CBaseCorrespondenceRejection*  m_pCorrespondenceRejection;
		CBaseTransformationEstimation* m_pTransformationEsitmation;
		CDefaultConvergenceCriteria*   m_pConvergenceCriterial;

		void __setMethodPointCloud();
		void __fitLoop(Eigen::Matrix3d& vioR, ColVector3& vioT);
		void __setPointCloud(const hivePointCloud::CPointCloud& vSourcePointCloud, const hivePointCloud::CPointCloud& vTargetPointCloud);
		std::string __getConvergenceMsg();
	};
}