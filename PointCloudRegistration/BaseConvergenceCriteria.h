#pragma once
#include <string>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/any.hpp>
#include "BaseProduct.h"

namespace hiveRegistration
{
	class CBaseConvergenceCriteria : public hiveCommon::CBaseProduct
	{
		friend boost::serialization::access;
	public:
		CBaseConvergenceCriteria();

		bool operator()(unsigned int vIterationCounter)  { return _isConvergedV(vIterationCounter); }
		bool isConverged(unsigned int vIterationCounter) { return _isConvergedV(vIterationCounter); }

		static std::string getClassSig()            { std::string Tmp("BaseConvergenceCriteria"); boost::algorithm::to_upper(Tmp); return Tmp; }
		static std::string getKeyMaxIterations()    { std::string Tmp("BaseConvergenceCriteria.MI"); boost::algorithm::to_upper(Tmp); return Tmp; }
		static std::string getKeyConvergenceState() { std::string Tmp("BaseConvergenceCriteria.CS"); boost::algorithm::to_upper(Tmp); return Tmp; }

		const std::string& getConvergenceMsg() const;
		void setMapConvergenceType(const char vState, const std::string& vStrConvergenceType);

	protected:
		virtual ~CBaseConvergenceCriteria() {}

		virtual bool _isConvergedV(unsigned int vIterationCounter)
		{
			return __isIterationEndedV(vIterationCounter);
		}

		virtual bool __isIterationEndedV(unsigned int vIterationCounter) const;

	private:
		enum EConvergenceState
		{
			CONVERGENCE_CRITERIA_NOT_CONVERGED,
			CONVERGENCE_CRITERIA_ITERATIONS
		};

		std::map<char, std::string> m_MapConvergenceType;

		void __init();
	};
}