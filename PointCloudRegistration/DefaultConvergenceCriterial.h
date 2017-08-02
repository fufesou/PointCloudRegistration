#pragma once
#include <map>
#include <boost/algorithm/string.hpp>
#include "ICPType.h"
#include "BaseConvergenceCriteria.h"

namespace hiveRegistration
{
	class CDefaultConvergenceCriteria : public CBaseConvergenceCriteria
	{
		friend class boost::serialization::access;

	public:
		CDefaultConvergenceCriteria ();

		static std::string getClassSig()                         { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria")); }
		static std::string getKeyMaxIterationSimilarTransforms() { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.MIST")); }
		static std::string getKeyFailureAfterMaxIterations()     { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.FAMI")); }
		static std::string getKeyRotationThreshold()             { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.RT")); }
		static std::string getKeyTranslationThreshold()          { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.TT")); }
		static std::string getKeyRelativeMSE()                   { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.RMSE")); }
		static std::string getKeyAbsoluteMSE()                   { return boost::to_upper_copy(std::string("DefaultConvergenceCriteria.AMSE")); }
		static std::string getKeyMaxIterations()                 { return CBaseConvergenceCriteria::getKeyMaxIterations(); }
		static std::string getKeyConvergenceState()              { return CBaseConvergenceCriteria::getKeyConvergenceState(); }

		void setCorrespondencePairSet(const CorrespondencePairSet& vCorrespondencePairs) { m_pCorrespondencePairs = &vCorrespondencePairs; }
		bool operator()(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter);
		bool isConverged(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter);

	protected:
		virtual bool _isConvergedV(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT, unsigned int vIterationCounter);

	private:
		enum EConvergenceState
		{
			CONVERGENCE_CRITERIA_NOT_CONVERGED,
			CONVERGENCE_CRITERIA_ITERATIONS,
			CONVERGENCE_CRITERIA_TRANSFORM,
			CONVERGENCE_CRITERIA_ABS_MSE,
			CONVERGENCE_CRITERIA_REL_MSE,
			CONVERGENCE_CRITERIA_NO_CORRESPONDENCES
		};

		double m_PreMSE;
		double m_CurMSE;
		Eigen::Matrix3d m_PreRot;
		ColVector3      m_PreTra;
		unsigned int m_IterationsSimilarTransforms;
		const CorrespondencePairSet* m_pCorrespondencePairs;

		std::string m_OutputDistanceFile;

		void __init();
		bool __isIterationEndedV(unsigned int vIterationCounter);
		bool __isTransformationEnded(const Eigen::Matrix3d& vCurR, const ColVector3& vCurT);
		bool __isMSEAbsoluteEnded();
		bool __isMSERelativeEnded();
		double __calculateMSE () const;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}