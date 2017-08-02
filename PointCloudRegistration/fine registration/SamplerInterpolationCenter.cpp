#include "SamplerInterpolationCenter.h"
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include ".\Bicubic.h"
#include "PointCloudSubset.h"
#include "SquareCellsInitialization.h"
#include "ControlPointsData.h"

// #ifdef _DEBUG
// #include "TestUnitity.h"
// #endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CSamplerInterpolationCenter> TheCreator(CSamplerInterpolationCenter::getClassSig());

CSamplerInterpolationCenter::CSamplerInterpolationCenter()
{
	const unsigned int NumSamplerPoint = 100;
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyCorPointNum(), NumSamplerPoint);
}

CSamplerInterpolationCenter::~CSamplerInterpolationCenter()
{
}

//*********************************************************************************
//FUNCTION:
CorrespondencePointSet CSamplerInterpolationCenter::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	if (NULL == m_pAllCenterPointSet.get())
	{
		__initAllCenterPointSet(vPointCloud);
	}

	return __setUpCorrespondencePointSet();
}

//*********************************************************************************
//FUNCTION:
void CSamplerInterpolationCenter::__findInterpolationSurfaceCenterSet(const MatrixControlPoints& vControlPoints, VecColVector3& voPointSet, std::vector<std::pair<int, int> >& voIndexPairSet, VecColVector3& voNormalSet)
{
	const int IndexMaxRows = hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance()->NumTotalRowAndCols.first - 3;
	const int IndexMaxCols = hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance()->NumTotalRowAndCols.second - 3;

	voPointSet.clear();
	voIndexPairSet.clear();

	Bicubic::CBicubic BicubicObj;

	const double ValueU = 0.5, ValueV = 0.5;

	for (int i=0; i<IndexMaxRows; ++i)
	{
		for (int k=0; k<IndexMaxCols; ++k)
		{
			if ( !vControlPoints(i,	k  ).empty() && !vControlPoints(i+1, k  ).empty() && !vControlPoints(i+2, k  ).empty() && !vControlPoints(i+3, k  ).empty() &&
				 !vControlPoints(i,	k+1).empty() && !vControlPoints(i+1, k+1).empty() && !vControlPoints(i+2, k+1).empty() && !vControlPoints(i+3, k+1).empty() &&
				 !vControlPoints(i,	k+2).empty() && !vControlPoints(i+1, k+2).empty() && !vControlPoints(i+2, k+2).empty() && !vControlPoints(i+3, k+2).empty() &&
				 !vControlPoints(i,	k+3).empty() && !vControlPoints(i+1, k+3).empty() && !vControlPoints(i+2, k+3).empty() && !vControlPoints(i+3, k+3).empty() )
			{
				Eigen::Matrix4d ControlPoints[3];
				ControlPoints[0] 
				<<	vControlPoints(i  , k  )[0](0, 0), vControlPoints(i  , k+1)[0](0, 0), vControlPoints(i  , k+2)[0](0, 0), vControlPoints(i  , k+3)[0](0, 0),
					vControlPoints(i+1, k  )[0](0, 0), vControlPoints(i+1, k+1)[0](0, 0), vControlPoints(i+1, k+2)[0](0, 0), vControlPoints(i+1, k+3)[0](0, 0),
					vControlPoints(i+2, k  )[0](0, 0), vControlPoints(i+2, k+1)[0](0, 0), vControlPoints(i+2, k+2)[0](0, 0), vControlPoints(i+2, k+3)[0](0, 0),
					vControlPoints(i+3, k  )[0](0, 0), vControlPoints(i+3, k+1)[0](0, 0), vControlPoints(i+3, k+2)[0](0, 0), vControlPoints(i+3, k+3)[0](0, 0);
				ControlPoints[1] 
				<<	vControlPoints(i  , k  )[0](1, 0), vControlPoints(i  , k+1)[0](1, 0), vControlPoints(i  , k+2)[0](1, 0), vControlPoints(i  , k+3)[0](1, 0),
					vControlPoints(i+1, k  )[0](1, 0), vControlPoints(i+1, k+1)[0](1, 0), vControlPoints(i+1, k+2)[0](1, 0), vControlPoints(i+1, k+3)[0](1, 0),
					vControlPoints(i+2, k  )[0](1, 0), vControlPoints(i+2, k+1)[0](1, 0), vControlPoints(i+2, k+2)[0](1, 0), vControlPoints(i+2, k+3)[0](1, 0),
					vControlPoints(i+3, k  )[0](1, 0), vControlPoints(i+3, k+1)[0](1, 0), vControlPoints(i+3, k+2)[0](1, 0), vControlPoints(i+3, k+3)[0](1, 0);
				ControlPoints[2] 
				<<	vControlPoints(i  , k  )[0](2, 0), vControlPoints(i  , k+1)[0](2, 0), vControlPoints(i  , k+2)[0](2, 0), vControlPoints(i  , k+3)[0](2, 0),
					vControlPoints(i+1, k  )[0](2, 0), vControlPoints(i+1, k+1)[0](2, 0), vControlPoints(i+1, k+2)[0](2, 0), vControlPoints(i+1, k+3)[0](2, 0),
					vControlPoints(i+2, k  )[0](2, 0), vControlPoints(i+2, k+1)[0](2, 0), vControlPoints(i+2, k+2)[0](2, 0), vControlPoints(i+2, k+3)[0](2, 0),
					vControlPoints(i+3, k  )[0](2, 0), vControlPoints(i+3, k+1)[0](2, 0), vControlPoints(i+3, k+2)[0](2, 0), vControlPoints(i+3, k+3)[0](2, 0);

				ColVector3 Point;
				BicubicObj.compute3DPointPosByUV(ValueU, ValueV, ControlPoints, Point);

				ColVector3 Normal;
				BicubicObj.computeNormal(ValueU, ValueV, ControlPoints, Normal);

				voPointSet.push_back(Point);
				voIndexPairSet.push_back(std::make_pair(i, k));
				voNormalSet.push_back(Normal);

// #ifdef _DEBUG
// 				static int s_CtlIdx = 0;
// 				++s_CtlIdx;
// 				static char FileName[255];
// 				sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\%d_%d_%d.ply", s_CtlIdx, i, k);
// 				VecColVector3 CtlPnts;
// 				CtlPnts.push_back(vControlPoints(i,	k  )[0]); CtlPnts.push_back(vControlPoints(i+1,	k  )[0]); CtlPnts.push_back(vControlPoints(i+2,	k  )[0]); CtlPnts.push_back(vControlPoints(i+3,	k  )[0]);
// 				CtlPnts.push_back(vControlPoints(i,	k+1)[0]); CtlPnts.push_back(vControlPoints(i+1,	k+1)[0]); CtlPnts.push_back(vControlPoints(i+2,	k+1)[0]); CtlPnts.push_back(vControlPoints(i+3,	k+1)[0]);
// 				CtlPnts.push_back(vControlPoints(i,	k+2)[0]); CtlPnts.push_back(vControlPoints(i+1,	k+2)[0]); CtlPnts.push_back(vControlPoints(i+2,	k+2)[0]); CtlPnts.push_back(vControlPoints(i+3,	k+2)[0]);
// 				CtlPnts.push_back(vControlPoints(i,	k+3)[0]); CtlPnts.push_back(vControlPoints(i+1,	k+3)[0]); CtlPnts.push_back(vControlPoints(i+2,	k+3)[0]); CtlPnts.push_back(vControlPoints(i+3,	k+3)[0]);
// 				export2Ply(FileName, CtlPnts);
// 				sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\%d_%d_%d_smp.ply", s_CtlIdx, i, k);
// 				CtlPnts.swap(VecColVector3(1, Point));
// 				export2Ply(FileName, CtlPnts);
// #endif

			}
		}
	}

// #ifdef _DEBUG
// 	static int s_Int = 0;
// 	++s_Int;
// 	static char FileName[255];
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\%d.ply", s_Int);
// 	export2Ply(FileName, voPointSet);
// #endif

}

//*********************************************************************************
//FUNCTION:
void CSamplerInterpolationCenter::__initAllCenterPointSet(const hivePointCloud::CPointCloud& vSourcePointCloud)
{
	MatrixControlPoints s_TgtControlPoints;
	CSquareCellsInitialization SquareCellsInitialization;
	SquareCellsInitialization.genControlPointsMatrix(&vSourcePointCloud, *hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance(), s_TgtControlPoints);

	VecColVector3 PointSet;
	VecColVector3 NormalSet;
	std::vector<std::pair<int, int> > IndexPairSet;
	__findInterpolationSurfaceCenterSet(s_TgtControlPoints, PointSet, IndexPairSet, NormalSet);

	CBicubicGridSubset* pCorrespondencePointSet = new CBicubicGridSubset(s_TgtControlPoints);
	pCorrespondencePointSet->swapPointSet(PointSet, NormalSet, IndexPairSet);
	m_pAllCenterPointSet = makeSharedPtr(pCorrespondencePointSet);
}

//*********************************************************************************
//FUNCTION:
CorrespondencePointSet CSamplerInterpolationCenter::__setUpCorrespondencePointSet()
{
	CBicubicGridSubset* pAllCenterPoints = dynamic_cast<CBicubicGridSubset*>(m_pAllCenterPointSet.get());
	_ASSERT(pAllCenterPoints);

	int NumK;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCorPointNum(), NumK);
	_ASSERT(DumpRes);

	if (NumK < pAllCenterPoints->getPointSet().size())
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, 
			"size of sample point is bigger than the number of center points in surfaces, k is reset to the number of center points in surfaces");

		const MatrixControlPoints& AllMatCtrlPntSet = pAllCenterPoints->getControlPoints();
		const std::vector<std::pair<int, int> >& AllIdxPairSet = pAllCenterPoints->getIndexPairSet();
		const VecColVector3& AllPntSet = pAllCenterPoints->getPointSet();
		const VecColVector3& AllNormSet = pAllCenterPoints->getNormalSet();
		std::vector<std::pair<int, int> > RestrictedIdxPairSet;
		VecColVector3 RestrictedPntSet;
		VecColVector3 RestrictedNormSet;

		CorrespondencePointSet RestrictedSamplePointSet = makeSharedPtr(new CBicubicGridSubset(AllMatCtrlPntSet));
		CBicubicGridSubset* pRestrictedSamplePointSet = dynamic_cast<CBicubicGridSubset*>(RestrictedSamplePointSet.get());
		int TotalSize = AllIdxPairSet.size();

		double SampleRatio = 1.0 * NumK / TotalSize;
//#pragma omp parallel for
		for (int i=0; i<TotalSize; ++i)
		{
			if (ICP_RAND(0.0, 1.0) < SampleRatio)
			{
				RestrictedIdxPairSet.push_back(AllIdxPairSet[i]);
				RestrictedPntSet.push_back(AllPntSet[i]);
				RestrictedNormSet.push_back(AllNormSet[i]);
			}
		}
		pRestrictedSamplePointSet->swapPointSet(RestrictedPntSet, RestrictedNormSet, RestrictedIdxPairSet);

		return RestrictedSamplePointSet;
	}
	else
	{
		return m_pAllCenterPointSet;
	}
}