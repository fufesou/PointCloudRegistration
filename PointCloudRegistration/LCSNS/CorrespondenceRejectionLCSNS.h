#pragma once
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseCorrespondenceRejection.h"
#include "LCSNSType.h"

namespace hiveRegistration
{
	class CCorrespondenceRejectionLCSNS : public CBaseCorrespondenceRejection
	{
		friend class boost::serialization::access;

	public:
		CCorrespondenceRejectionLCSNS();

		static std::string getClassSig()                        { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNS")); }
		static std::string getKeyDoCurvatureConstraint()        { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNS.DCC")); }
		static std::string getKeyCurvatureConstraintThreshold() { return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNS.CCT")); }
		static std::string getKeyCurvatureRatioFile()			{ return boost::algorithm::to_upper_copy(std::string("CorrespondenceRejectionLCSNS.CRFile")); }

	protected:
		~CCorrespondenceRejectionLCSNS();

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;

	private:
		int m_DoCurvatureConstraint;

		void __computePrincipalCurvatureSet( 
			const std::pair<double, double>& vUVSet, const std::pair<int, int>& vIndexPairSet, const ControlPoints& vControlPoints, 
			std::pair<double, double>& voPrincipalCurvature ) const;
		void __conv2OWorldCoordsWithCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs);
		void __conv2OWorldCoordsWithoutCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs);

		void __parseConfig(void);
	};
}