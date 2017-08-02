#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseCorrespondenceRejection.h"
#include "LCSNSType.h"

namespace hiveRegistration
{
	class CCorrespondenceRejectionLCSNSClosest : public CBaseCorrespondenceRejection
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceRejectionLCSNSClosest();

		static std::string getClassSig()                        { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNSClosest")); }
		static std::string getKeyDoCurvatureConstraint()        { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNSClosest.DCC")); }
		static std::string getKeyCurvatureConstraintThreshold() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNSClosest.CCT")); }
		static std::string getKeyCurvatureRatioFile()			{ return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNSClosest.CRFile")); }

	protected:
		~CCorrespondenceRejectionLCSNSClosest();

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;

	private:
		int m_DoCurvatureConstraint;

		void __computePrincipalCurvatureSet( 
			const std::pair<double, double>& vUVSet, const std::pair<int, int>& vIndexPairSet, const ControlPoints& vControlPoints, 
			std::pair<double, double>& voPrincipalCurvature ) const;
		void __applyCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs);

		void __parseConfig(void);
	};
}