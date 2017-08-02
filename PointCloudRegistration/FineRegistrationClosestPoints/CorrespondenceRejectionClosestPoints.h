#pragma once
#include <boost/serialization/access.hpp>
#include <boost/algorithm/string.hpp>
#include "BaseCorrespondenceRejection.h"

namespace hiveRegistration
{
	class CCorrespondenceRejectionClosestPoints : public CBaseCorrespondenceRejection
	{
		friend boost::serialization::access;

	public:
		CCorrespondenceRejectionClosestPoints(void);

		static std::string getClassSig() { return boost::to_upper_copy(std::string("CorrespondenceRejectionClosestPoints")); }

	protected:
		virtual ~CCorrespondenceRejectionClosestPoints(void) {}

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;
	};
}