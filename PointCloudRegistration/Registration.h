#pragma once
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CIterativeFit;

	class CRegistartion
	{
	public:
		typedef boost::tuple<Eigen::Matrix3d, ColVector3, std::string, std::string> RotTraCoarseFineFitMsg;
		typedef boost::tuple<Eigen::Matrix3d, ColVector3, std::string> RotTraFitMsg;

		CRegistartion();

		RotTraCoarseFineFitMsg fit(
			const hivePointCloud::CPointCloud& vSourcePointCloud, 
			const hivePointCloud::CPointCloud& vTargetPointCloud, 
			const Eigen::Matrix3d& vInitR, ColVector3& vInitT);

		RotTraFitMsg coarseFit(
			const hivePointCloud::CPointCloud& vSourcePointCloud, 
			const hivePointCloud::CPointCloud& vTargetPointCloud,
			const Eigen::Matrix3d& vInitR, ColVector3& vInitT);

		RotTraFitMsg fineFit(
			const hivePointCloud::CPointCloud& vSourcePointCloud, 
			const hivePointCloud::CPointCloud& vTargetPointCloud, 
			const Eigen::Matrix3d& vInitR, ColVector3& vInitT);

		// these methods should be moved to some other classes
		void setSampler(const std::string& vPointSamplerSig);
		void setCorrespondenceEstimation(const std::string& vCorrespondenceEsitmationSig);
		void setCorrespondenceRejection(const std::string& vCorrespondenceRejectionSig);
		void setTransformationEstimation(const std::string& vTransformationEsitmationSig);

	private:
		boost::shared_ptr<CIterativeFit> m_pCoarseIterativeFit;
		boost::shared_ptr<CIterativeFit> m_pFineIterativeFit;

		void __init();
		void __parseConfigFile();
		void __fillICPClassesSig();
		void __setCoarseFitClasses();
		void __setFineFitClasses();
	};
}