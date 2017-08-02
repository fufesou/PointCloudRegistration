#pragma once
#include "BaseCorrespondenceRejection.h"


namespace hiveRegistration
{
	class CCorrespondenceRejectionLCP : public CBaseCorrespondenceRejection
	{
		friend boost::serialization::access;

	public:
		static std::string getClassSig() { return boost::to_upper_copy(std::string("CorrespondenceRejectionLCP")); }

		CCorrespondenceRejectionLCP();

	protected:
		virtual ~CCorrespondenceRejectionLCP(void) {}

		virtual void _rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs) override;

	private:
	};
}