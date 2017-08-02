#include "CorrespondenceRejectionLCSNSClosest.h"
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

hiveCommon::CProductFactory<CCorrespondenceRejectionLCSNSClosest> TheCreator(CCorrespondenceRejectionLCSNSClosest::getClassSig());


CCorrespondenceRejectionLCSNSClosest::CCorrespondenceRejectionLCSNSClosest()
{
	PTR_CONTROL_PARAMS->setIfNotExist(CCorrespondenceRejectionLCSNSClosest::getKeyCurvatureConstraintThreshold(), 0.1);
	__parseConfig();
}

CCorrespondenceRejectionLCSNSClosest::~CCorrespondenceRejectionLCSNSClosest()
{

}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceRejectionLCSNSClosest::_rejectInvalidPairsV( const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs )
{
	if (m_DoCurvatureConstraint)
	{
		__applyCurvConstraint(vAllCorrespondencePairs, voValidCorrespondencePairs);
	}
	else
	{
		voValidCorrespondencePairs = vAllCorrespondencePairs;
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNSClosest::__computePrincipalCurvatureSet( 
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
void hiveRegistration::CCorrespondenceRejectionLCSNSClosest::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoCurvatureConstraint(), m_DoCurvatureConstraint);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionLCSNSClosest::__applyCurvConstraint(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{
	_ASSERT((vAllCorrespondencePairs.first).get() && (vAllCorrespondencePairs.second).get());

	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pAllSourceCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>( vAllCorrespondencePairs.first.get() );
	_ASSERT(pAllSourceCorPointSet);
	CCommonLCSNSSubset<PtrSimpleCtrlPntSet>* pAllTargetCorPointSet = dynamic_cast<CCommonLCSNSSubset<PtrSimpleCtrlPntSet>*>( vAllCorrespondencePairs.second.get() );
	_ASSERT(pAllTargetCorPointSet);

	const VecColVector3& AllSourcePointSet = pAllSourceCorPointSet->getPointSet();
	const VecColVector3& AllTargetPointSet = pAllTargetCorPointSet->getPointSet();
	const PtrSimpleCtrlPntSet& AllSourceControlPoints = pAllSourceCorPointSet->getControlPointsSet();
	const PtrSimpleCtrlPntSet& AllTargetControlPoints = pAllTargetCorPointSet->getControlPointsSet();
	const std::vector<std::pair<int, int> >& AllSourceIndexPairSet = pAllSourceCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AllTargetIndexPairSet = pAllTargetCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& AllSourceUVSet = pAllSourceCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AllTargetUVSet = pAllTargetCorPointSet->getUVSet();

	std::pair<double, double> SrcPrincipalCurvature, TgtPrincipalCurvature;

	VecColVector3 ValidSourcePointSet;
	VecColVector3 ValidTargetPointSet;

	double Threshold;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(CCorrespondenceRejectionLCSNSClosest::getKeyCurvatureConstraintThreshold(), Threshold);
	_ASSERT(DumpRes);

// #ifdef _DEBUG
// 	std::string CurvatureRatioFile;
// 	DumpRes = PTR_CONTROL_PARAMS->dumpValue(CCorrespondenceRejectionLCSNSClosest::getKeyCurvatureRatioFile(), CurvatureRatioFile);
// 	_ASSERT(DumpRes);
// 
// 	std::ofstream OutFile(CurvatureRatioFile.c_str());
// #endif

	int SizePairedPnts = AllSourcePointSet.size();

	//#pragma parallel for
	for (int i=0; i<SizePairedPnts; ++i)
	{
		__computePrincipalCurvatureSet(AllSourceUVSet[i], AllSourceIndexPairSet[i], (*AllSourceControlPoints)[i], SrcPrincipalCurvature);
		__computePrincipalCurvatureSet(AllTargetUVSet[i], AllTargetIndexPairSet[i], (*AllTargetControlPoints)[i], TgtPrincipalCurvature);

		const ColVector3& CurrentSourcePointSet = AllSourcePointSet[i];
		const ColVector3& CurrentTargetPointSet = AllTargetPointSet[i];

		double CurRatio = fabs(fabs(SrcPrincipalCurvature.first - TgtPrincipalCurvature.first) - fabs(SrcPrincipalCurvature.second - TgtPrincipalCurvature.second)) /
			(fabs(SrcPrincipalCurvature.first - TgtPrincipalCurvature.first) + fabs(SrcPrincipalCurvature.second - TgtPrincipalCurvature.second));

// #ifdef _DEBUG
// 		OutFile << CurRatio << std::endl;
// #endif

		if ( CurRatio <= Threshold)
		{
			ValidSourcePointSet.push_back(CurrentSourcePointSet);
			ValidTargetPointSet.push_back(CurrentTargetPointSet);
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