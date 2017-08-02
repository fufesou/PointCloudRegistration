#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include "ICPType.h"

namespace hiveRegistration
{
	class CNormalShootingLCSNSClosest;
	class CReconstructedSurfaceData;

	class CAuxiliaryConstraintLCSNSClosest
	{
	public:
		CAuxiliaryConstraintLCSNSClosest();
		~CAuxiliaryConstraintLCSNSClosest();

		static std::string getKeyPairThreshold()         { return boost::algorithm::to_upper_copy(std::string("AuxiliaryConstraintLCSNSClosest.PT")); }
		static std::string getKeyAuxiliarySearchRadius() { return boost::algorithm::to_upper_copy(std::string("AuxiliaryConstraintLCSNSClosest.ASR")); }

		void doAuxiliaryPairConstraint(const CorrespondencePairSet& vInitCorPairSet, CorrespondencePairSet& voCorrespondencePointPairSet);

	private:
		double m_PairThreshold;
		double m_AuxiSreachRadius;

		CorrespondencePairSet m_pInitCorrespondencePairSet;
		CorrespondencePairSet m_pAuxiCorrespondencePairSet;
		std::vector<unsigned int> m_MapInitAuxi;
		boost::shared_ptr<CNormalShootingLCSNSClosest> m_pPairEstimationNormal2Plane;

		void __setInitCorrespondencePairSet(const CorrespondencePairSet& vInitCorPairSet);
		void __setUpAuxiliaryCorrespondencePairSet();
		void __setUpAuxiliarySourceCorrespondencePointSet(CorrespondencePointSet& voAuxiSrcCorrespondencePointSet);
		void __pairAuxiliaryCorrespondencePairSet(const CorrespondencePointSet& vAuxiliarySourceCorrespondencePointSet);
		void __doAuxiliaryPairConstraint(CorrespondencePairSet& voCorrespondencePointPairSet);	
		void __parseConfig(void);
	};
}