#include "BaseConvergenceCriteria.h"
#include "HiveCommon.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CBaseConvergenceCriteria> TheCreator("BaseConvergenceCriteria");

CBaseConvergenceCriteria::CBaseConvergenceCriteria() 
{
	__init();
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
bool CBaseConvergenceCriteria::__isIterationEndedV(unsigned int vIterationCounter) const
{
	unsigned int MaxIterations;
	bool CastSuccess = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxIterations(), MaxIterations);
	_ASSERT(CastSuccess);

	return vIterationCounter >= MaxIterations;
}

//*********************************************************************************
//FUNCTION:
void CBaseConvergenceCriteria::__init()
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyMaxIterations(), unsigned int(1000));
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyConvergenceState(), static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED));

	m_MapConvergenceType.insert(std::make_pair(static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED), "NotConverged"));
	m_MapConvergenceType.insert(std::make_pair(static_cast<char>(CONVERGENCE_CRITERIA_ITERATIONS), "Criteria_Iterations"));
}

//*********************************************************************************
//FUNCTION:
const std::string& CBaseConvergenceCriteria::getConvergenceMsg() const
{
	char ConvergenceState;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyConvergenceState(), ConvergenceState);
	_ASSERT(DumpRes);

	const std::string& StrNotConverged = (m_MapConvergenceType.find(static_cast<char>(CONVERGENCE_CRITERIA_NOT_CONVERGED)))->second;

	std::map<char, std::string>::const_iterator CIter = m_MapConvergenceType.find(ConvergenceState);
	return (CIter == m_MapConvergenceType.end()) ? StrNotConverged : (CIter->second);
}

//*********************************************************************************
//FUNCTION:
void CBaseConvergenceCriteria::setMapConvergenceType(const char vState, const std::string& vStrConvergenceType)
{
	m_MapConvergenceType[vState] = vStrConvergenceType;
}
