#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include "CommonInterface.h"
#include "EventLoggerInterface.h"
#include "HiveCommonMicro.h"
#include "BaseSearch.h"

using namespace hiveSearch;

CBaseSearch::CBaseSearch(void) : m_IsAccelerationStructureBuilt(false)
{
}

CBaseSearch::~CBaseSearch(void)
{
}

//*********************************************************************************
//FUNCTION:
const double* CBaseSearch::SInputData::getInputDataAt(unsigned int vIndex) const
{
	return (vIndex < NumInput) ? &pData[vIndex * Dimension] : NULL;
}

//*********************************************************************************
//FUNCTION:
void CBaseSearch::setInputData(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vDimension)
{
	m_InputData.pData     = vInputPoint;
	m_InputData.NumInput  = vNumInputPoint;
	m_InputData.Dimension = vDimension;
	m_IsAccelerationStructureBuilt = false;
}

//*********************************************************************************
//FUNCTION:
std::vector<unsigned int>* CBaseSearch::executeSearch(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, bool vMultiThreaded)
{
	_ASSERT((vNumNearestNeighbor > 0) && (vNumTargetPoint > 0) && vTargetPoint);

	std::vector<unsigned int> *pResult = NULL;
	try
	{
		if (!m_IsAccelerationStructureBuilt) m_IsAccelerationStructureBuilt = __buildAccelerationStructure();

		if (m_IsAccelerationStructureBuilt)
		{
			pResult = new std::vector<unsigned int>;
			pResult->reserve(vNumTargetPoint * vNumNearestNeighbor);
			_executeSearchV(vNumNearestNeighbor, vTargetPoint, vNumTargetPoint, vMultiThreaded, *pResult);
			if (m_IsOutputResultRequested) __outputSearchResult(vNumNearestNeighbor, vTargetPoint, vNumTargetPoint, pResult);
		}
		return pResult;
	}
	catch (...)
	{
		delete pResult;
		return NULL;
	}
}

//*********************************************************************************
//FUNCTION:
bool CBaseSearch::__buildAccelerationStructure()
{
	_ASSERT(!m_IsAccelerationStructureBuilt);
	hiveEventLogger::hiveOutputEvent("Start building acceleration data structure...");
	if (_buildAccelerationStructureV())
	{
		hiveEventLogger::hiveOutputEvent("Finish building acceleration data structure.");
		return true;
	}
	else
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, "Fail to build the acceleration data structure for search.");
		return false;
	}
}

//*********************************************************************************
//FUNCTION:
void CBaseSearch::__outputSearchResult(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, const std::vector<unsigned int>* vResult) const
{
	_ASSERT(m_InputData.pData && (vResult->size() == vNumTargetPoint * vNumNearestNeighbor));

	std::string ResultStr = "Input of the search:\n";
	ResultStr += _BOOST_STR1("  Number of input data: %1%\n", m_InputData.NumInput);
	ResultStr += _BOOST_STR1("  Dimension of input data: %1%\n", m_InputData.Dimension);
	ResultStr += std::string("Output of the search:\n");
	for (unsigned int i=0; i<vNumTargetPoint; i++)
	{
		const double *pTargetPoint = &vTargetPoint[i*m_InputData.Dimension];
		ResultStr += _BOOST_STR1("  Target point: %1%\n", __convertData2String(pTargetPoint));
		for (unsigned int k=0; k<vNumNearestNeighbor; k++)
		{
			const double *pInputPoint = getInputDataAt(vResult->at(i*vNumNearestNeighbor+k));
			ResultStr += _BOOST_STR2("    %1% : %2%\n", _computeDisV(pTargetPoint, pInputPoint), __convertData2String(pInputPoint));
		}
	}

	hiveEventLogger::hiveOutputEvent(ResultStr);
}

//*********************************************************************************
//FUNCTION:
std::string CBaseSearch::__convertData2String(const double *vData) const
{
	_ASSERT(vData && (m_InputData.Dimension > 0));
	std::string Result;
	for (unsigned int i=0; i<m_InputData.Dimension; i++)
	{
		Result += _BOOST_STR1("%1% ", vData[i]);
	}
	return Result;
}

//*********************************************************************************
//FUNCTION:
double CBaseSearch::_computeDisV(const double *vData1, const double *vData2) const
{
	double Ret = 0;
	for (unsigned int i=0; i<m_InputData.Dimension; ++i)
	{
		Ret += (vData1[i] - vData2[i]) * (vData1[i] - vData2[i]);
	}
	return Ret;
}