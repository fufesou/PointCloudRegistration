#include "CorrespondenceRejectionLCSNS.h"
#include <time.h>
#include <utility>
#include <Eigen/Core>
#include <Eigen/LU>
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "CommonLCSNSSubset.hpp"
#include "Bicubic.h"
#include "LCSNSType.h"
#include "ControlMatrixData.h"

// #ifdef _DEBUG
// #include <fstream>
// #endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceRejectionLCSNS> TheCreator(CCorrespondenceRejectionLCSNS::getClassSig());


CCorrespondenceRejectionLCSNS::CCorrespondenceRejectionLCSNS()
{
	PTR_CONTROL_PARAMS->setIfNotExist(CCorrespondenceRejectionLCSNS::getKeyCurvatureConstraintThreshold(), 0.1);
	__parseConfig();
}

CCorrespondenceRejectionLCSNS::~CCorrespondenceRejectionLCSNS()
{

}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceRejectionLCSNS::_rejectInvalidPairsV( const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs )
{
	if (m_DoCurvatureConstraint)
	{
		__conv2OWorldCoordsWithCurvConstraint(vAllCorrespondencePairs, voValidCorrespondencePairs);
	}
	else
	{
		__conv2OWorldCoordsWithoutCurvConstraint(vAllCorrespondencePairs, voValidCorrespondencePairs);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNS::__computePrincipalCurvatureSet( 
	const std::pair<double, double>& vUVSet, const std::pair<int, int>& vIndexPairSet, 
	const ControlPoints& vControlPoints, std::pair<double, double>& voPrincipalCurvature ) const
{
	Eigen::Matrix4d	CtrlPoints[3];
	int i = vIndexPairSet.first;
	int k = vIndexPairSet.second;

	CtrlPoints[0] <<
		vControlPoints(i, k  )(0, 0), vControlPoints(i+1, k  )(0, 0), vControlPoints(i+2, k  )(0, 0), vControlPoints(i+3, k  )(0, 0),
		vControlPoints(i, k+1)(0, 0), vControlPoints(i+1, k+1)(0, 0), vControlPoints(i+2, k+1)(0, 0), vControlPoints(i+3, k+1)(0, 0),
		vControlPoints(i, k+2)(0, 0), vControlPoints(i+1, k+2)(0, 0), vControlPoints(i+2, k+2)(0, 0), vControlPoints(i+3, k+2)(0, 0),
		vControlPoints(i, k+3)(0, 0), vControlPoints(i+1, k+3)(0, 0), vControlPoints(i+2, k+3)(0, 0), vControlPoints(i+3, k+3)(0, 0);

	CtrlPoints[1] <<
		vControlPoints(i, k  )(1, 0), vControlPoints(i+1, k  )(1, 0), vControlPoints(i+2, k  )(1, 0), vControlPoints(i+3, k  )(1, 0),
		vControlPoints(i, k+1)(1, 0), vControlPoints(i+1, k+1)(1, 0), vControlPoints(i+2, k+1)(1, 0), vControlPoints(i+3, k+1)(1, 0),
		vControlPoints(i, k+2)(1, 0), vControlPoints(i+1, k+2)(1, 0), vControlPoints(i+2, k+2)(1, 0), vControlPoints(i+3, k+2)(1, 0),
		vControlPoints(i, k+3)(1, 0), vControlPoints(i+1, k+3)(1, 0), vControlPoints(i+2, k+3)(1, 0), vControlPoints(i+3, k+3)(1, 0);

	CtrlPoints[2] <<
		vControlPoints(i, k  )(2, 0), vControlPoints(i+1, k  )(2, 0), vControlPoints(i+2, k  )(2, 0), vControlPoints(i+3, k  )(2, 0),
		vControlPoints(i, k+1)(2, 0), vControlPoints(i+1, k+1)(2, 0), vControlPoints(i+2, k+1)(2, 0), vControlPoints(i+3, k+1)(2, 0),
		vControlPoints(i, k+2)(2, 0), vControlPoints(i+1, k+2)(2, 0), vControlPoints(i+2, k+2)(2, 0), vControlPoints(i+3, k+2)(2, 0),
		vControlPoints(i, k+3)(2, 0), vControlPoints(i+1, k+3)(2, 0), vControlPoints(i+2, k+3)(2, 0), vControlPoints(i+3, k+3)(2, 0);

	Bicubic::CBicubic BicubicObj;
	BicubicObj.computePrincipleCurvatures(vUVSet.first, vUVSet.second, CtrlPoints, voPrincipalCurvature);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNS::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoCurvatureConstraint(), m_DoCurvatureConstraint);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNS::__conv2OWorldCoordsWithCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{
	_ASSERT((vAllCorrespondencePairs.first).get() && (vAllCorrespondencePairs.second).get());

	CCommonLCSNSSubset<PtrControlPointsSet>* pAllSourceCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>( vAllCorrespondencePairs.first.get() );
	_ASSERT(pAllSourceCorPointSet);
	CCommonLCSNSSubset<PtrControlPointsSet>* pAllTargetCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>( vAllCorrespondencePairs.second.get() );
	_ASSERT(pAllTargetCorPointSet);

	const VecColVector3& AllSourcePointSet = pAllSourceCorPointSet->getPointSet();
	const VecColVector3& AllTargetPointSet = pAllTargetCorPointSet->getPointSet();
	const PtrControlPointsSet& AllSourceControlPoints = pAllSourceCorPointSet->getControlPointsSet();
	const PtrControlPointsSet& AllTargetControlPoints = pAllTargetCorPointSet->getControlPointsSet();
	const std::vector<std::pair<int, int> >& AllSourceIndexPairSet = pAllSourceCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AllTargetIndexPairSet = pAllTargetCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& AllSourceUVSet = pAllSourceCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AllTargetUVSet = pAllTargetCorPointSet->getUVSet();

	std::pair<double, double> SrcPrincipalCurvature, TgtPrincipalCurvature;

	VecColVector3 ValidSourcePointSet;
	VecColVector3 ValidTargetPointSet;

	double Threshold;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(CCorrespondenceRejectionLCSNS::getKeyCurvatureConstraintThreshold(), Threshold);
	_ASSERT(DumpRes);

	const SCtrolMatrixData* pCtrlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();

// #ifdef _DEBUG
// 	std::string CurvatureRatioFile;
// 	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CCorrespondenceRejectionLCSNS::getKeyCurvatureRatioFile(), CurvatureRatioFile);
// 	_ASSERT(DumpRes);
// 
// 	std::ofstream OutFile(CurvatureRatioFile.c_str());
// #endif

	int SizePairedPnts = AllSourcePointSet.size();

	//#pragma parallel for
	for (int i=0; i<SizePairedPnts; ++i)
	{
		__computePrincipalCurvatureSet(AllSourceUVSet[i], AllSourceIndexPairSet[i], (*AllSourceControlPoints)[i].second, SrcPrincipalCurvature);
		__computePrincipalCurvatureSet(AllTargetUVSet[i], AllTargetIndexPairSet[i], (*AllTargetControlPoints)[i].second, TgtPrincipalCurvature);

		const ColVector3& CurrentSourcePointSet = AllSourcePointSet[i];
		const ColVector3& CurrentTargetPointSet = AllTargetPointSet[i];

		unsigned int CurSampleIdx = (*AllSourceControlPoints)[i].first;

		Eigen::Matrix3d CurMatLoca2World = pCtrlMatData->VecCtrlMatData[CurSampleIdx].MatWorld2Local.inverse();
		const ColVector3& CurOrigin = pCtrlMatData->VecCtrlMatData[CurSampleIdx].Original;

		double CurRatio = fabs(fabs(SrcPrincipalCurvature.first - TgtPrincipalCurvature.first) - fabs(SrcPrincipalCurvature.second - TgtPrincipalCurvature.second)) /
			(fabs(SrcPrincipalCurvature.first - TgtPrincipalCurvature.first) + fabs(SrcPrincipalCurvature.second - TgtPrincipalCurvature.second));

// #ifdef _DEBUG
// 		OutFile << CurRatio << std::endl;
// #endif

		if ( CurRatio <= Threshold)
		{
			ValidSourcePointSet.push_back(CurMatLoca2World*CurrentSourcePointSet + CurOrigin);
			ValidTargetPointSet.push_back(CurMatLoca2World*CurrentTargetPointSet + CurOrigin);
		}
	}

// #ifdef _DEBUG
// 	OutFile.close();
// #endif

	voValidCorrespondencePairs = std::make_pair(makeSharedPtr(new CPntNormSubset()), makeSharedPtr(new CPntNormSubset()));
	CPntNormSubset* pCorSourcePointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.first.get());
	CPntNormSubset* pCorTargetPointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.second.get());
	pCorSourcePointCloudet->swapPointSet(ValidSourcePointSet);
	pCorTargetPointCloudet->swapPointSet(ValidTargetPointSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNS::__conv2OWorldCoordsWithoutCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{
	_ASSERT((vAllCorrespondencePairs.first).get() && (vAllCorrespondencePairs.second).get());

	CCommonLCSNSSubset<PtrControlPointsSet>* pAllSourceCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>( vAllCorrespondencePairs.first.get() );
	_ASSERT(pAllSourceCorPointSet);
	CCommonLCSNSSubset<PtrControlPointsSet>* pAllTargetCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>( vAllCorrespondencePairs.second.get() );
	_ASSERT(pAllTargetCorPointSet);

	const VecColVector3& AllSourcePointSet = pAllSourceCorPointSet->getPointSet();
	const VecColVector3& AllTargetPointSet = pAllTargetCorPointSet->getPointSet();
	const PtrControlPointsSet& AllSourceControlPoints = pAllSourceCorPointSet->getControlPointsSet();

	VecColVector3 ValidSourcePointSet;
	VecColVector3 ValidTargetPointSet;
	const SCtrolMatrixData* pCtrlMatData = hiveCommon::CSingleton<SCtrolMatrixData>::getInstance();

	int SizePairedPnts = AllSourcePointSet.size();
	//#pragma parallel for
	for (int i=0; i<SizePairedPnts; ++i)
	{
		unsigned int CurSampleIdx = (*AllSourceControlPoints)[i].first;
		Eigen::Matrix3d CurMatLoca2World = pCtrlMatData->VecCtrlMatData[CurSampleIdx].MatWorld2Local.inverse();
		const ColVector3& CurOrigin = pCtrlMatData->VecCtrlMatData[CurSampleIdx].Original;
		ValidSourcePointSet.push_back(CurMatLoca2World*AllSourcePointSet[i] + CurOrigin);
		ValidTargetPointSet.push_back(CurMatLoca2World*AllTargetPointSet[i] + CurOrigin);
	}

	voValidCorrespondencePairs = std::make_pair(makeSharedPtr(new CPntNormSubset()), makeSharedPtr(new CPntNormSubset()));
	CPntNormSubset* pCorSourcePointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.first.get());
	CPntNormSubset* pCorTargetPointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.second.get());
	pCorSourcePointCloudet->swapPointSet(ValidSourcePointSet);
	pCorTargetPointCloudet->swapPointSet(ValidTargetPointSet);
}
