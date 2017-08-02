#pragma once
#include <boost/algorithm/string.hpp>
#include "ControlPointsData.h"

namespace hiveRegistration
{
	class CPairEstimationNormal2Plane;
	class CReconstructedSurfaceData;

	class CEstimationAuxiliaryPairConstraint
	{
	public:
		CEstimationAuxiliaryPairConstraint();
		~CEstimationAuxiliaryPairConstraint();

		static std::string getKeyPairThreshold()         { return boost::algorithm::to_upper_copy(std::string("EstimationAuxiliaryPairConstraint.PT")); }
		static std::string getKeyAuxiliarySearchRadius() { return boost::algorithm::to_upper_copy(std::string("EstimationAuxiliaryPairConstraint.ASR")); }

		void doAuxiliaryPairConstraint(const CorrespondencePairSet& vInitCorPairSet, CorrespondencePairSet& voCorrespondencePointPairSet);

	private:
		CorrespondencePairSet m_pInitCorrespondencePairSet;
		CorrespondencePairSet m_pAuxiCorrespondencePairSet;
		std::vector<unsigned int> m_MapInitAuxi;
		boost::shared_ptr<CPairEstimationNormal2Plane> m_pPairEstimationNormal2Plane;

		void __setInitCorrespondencePairSet(const CorrespondencePairSet& vInitCorPairSet);
		void __setUpAuxiliaryCorrespondencePairSet(const MatrixControlPoints& vTgtPoints);
		void __setUpAuxiliarySourceCorrespondencePointSet(CorrespondencePointSet& voAuxiSrcCorrespondencePointSet);
		void __pairAuxiliaryCorrespondencePairSet(
			const CorrespondencePointSet& vAuxiliaryTargetCorrespondencePointSet, 
			const MatrixControlPoints& vTgtPoints);
		void __doAuxiliaryPairConstraint(CorrespondencePairSet& voCorrespondencePointPairSet);	
	};
}