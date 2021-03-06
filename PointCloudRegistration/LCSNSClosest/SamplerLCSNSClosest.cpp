#include "SamplerLCSNSClosest.h"
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "Bicubic.h"
#include "CommonLCSNSSubset.hpp"
#include "ControlMatrixLCSNSClosest.h"
#include "LCSNSType.h"


// #ifdef _DEBUG
// #include "TestUnitity.h"
// #endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CSamplerLCSNSClosest> TheCreator(CSamplerLCSNSClosest::getClassSig());

CSamplerLCSNSClosest::CSamplerLCSNSClosest()
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyCorPointNum(), 100);
}

CSamplerLCSNSClosest::~CSamplerLCSNSClosest()
{
}

//*********************************************************************************
//FUNCTION:
CorrespondencePointSet CSamplerLCSNSClosest::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	if (NULL == m_pAllCenterPointSet.get())
	{
		__initAllCenterPointSet(vPointCloud);
	}

	return m_pAllCenterPointSet;
}

//*********************************************************************************
//FUNCTION:
void CSamplerLCSNSClosest::__initAllCenterPointSet(const hivePointCloud::CPointCloud& vSourcePointCloud)
{
	PtrSimpleCtrlPntSet CtrlPntsSet;
	CControlMatrixLCSNSClosest ControlMatrixLCSNS;
	ControlMatrixLCSNS.genControlPointsMatrix(&vSourcePointCloud, CtrlPntsSet);

	PtrSimpleCtrlPntSet RestrictedCtrlPntsSet(new SimpleCtrlPntSet);

	int NumK;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCorPointNum(), NumK);
	_ASSERT(DumpRes);

	int SizeInitCtrlPntsSet = CtrlPntsSet->size();
	if (NumK < SizeInitCtrlPntsSet)
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, 
			"size of sample point is bigger than the number of center points in surfaces, k is reset to the number of center points in surfaces");

		double SampleRatio = 1.0 * NumK / SizeInitCtrlPntsSet;
		//#pragma omp parallel for
		for (int i=0; i<SizeInitCtrlPntsSet; ++i)
		{
			if (ICP_RAND(0.0, 1.0) < SampleRatio)
			{
				RestrictedCtrlPntsSet->push_back((*CtrlPntsSet)[i]);
			}
		}
	}
	else
	{
		RestrictedCtrlPntsSet = CtrlPntsSet;
	}

	int SizeCtrlMat[2];
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNSClosest::getKeySizeCtrlMatrixRow(), SizeCtrlMat[0]);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CControlMatrixLCSNSClosest::getKeySizeCtrlMatrixCol(), SizeCtrlMat[1]);
	_ASSERT(DumpRes);
	int CtrlMatSpan = 4;
	unsigned int InitSurfIdx[2] = { (SizeCtrlMat[0] - CtrlMatSpan) >> 1, (SizeCtrlMat[1] - CtrlMatSpan) >> 1 };

	std::vector<std::pair<int, int> > SurfIdxSet(RestrictedCtrlPntsSet->size(), std::make_pair(InitSurfIdx[0], InitSurfIdx[1]));
	std::vector<std::pair<double, double> > UVSet(RestrictedCtrlPntsSet->size(), std::make_pair(0.5, 0.5));

	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pCorrespondencePointSet = new CCommonLCSNSSubset<PtrSimpleCtrlPntSet>(RestrictedCtrlPntsSet);
	pCorrespondencePointSet->swapUVSet(SurfIdxSet, UVSet);

// #ifdef _DEBUG
// 	export2Ply("TestData\\output\\CorPointPairs\\FineReg_Sample\\CtlPntsQAll.ply", pCorrespondencePointSet->getPointSet(), pCorrespondencePointSet->getNormalSet());
// #endif

	m_pAllCenterPointSet = makeSharedPtr(pCorrespondencePointSet);
}