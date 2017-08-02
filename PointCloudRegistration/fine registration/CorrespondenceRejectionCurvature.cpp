#include "CorrespondenceRejectionCurvature.h"
#include <time.h>
#include <utility>
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloudSubset.h"
#include "Bicubic.h"

#ifdef _DEBUG
#include "TestUnitity.h"
#endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceRejectionCurvature> TheCreator(CCorrespondenceRejectionCurvature::getClassSig());

CCorrespondenceRejectionCurvature::CCorrespondenceRejectionCurvature()
{
	PTR_CONTROL_PARAMS->setIfNotExist(CCorrespondenceRejectionCurvature::getKeyCurvatureConstraintThreshold(), 0.1);
}

CCorrespondenceRejectionCurvature::~CCorrespondenceRejectionCurvature()
{

}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceRejectionCurvature::_rejectInvalidPairsV( const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs )
{
	_ASSERT((vAllCorrespondencePairs.first).get() && (vAllCorrespondencePairs.second).get());

	if (!m_DoDoCurvatureConstraint)
	{
		voValidCorrespondencePairs = vAllCorrespondencePairs;
		return;
	}

	CBicubicGridSubset* pAllSourceCorPointSetRegister = dynamic_cast<CBicubicGridSubset*>( vAllCorrespondencePairs.first.get() );
	_ASSERT(pAllSourceCorPointSetRegister);
	CBicubicGridSubset* pAllTargetCorPointSetRegister = dynamic_cast<CBicubicGridSubset*>( vAllCorrespondencePairs.second.get() );
	_ASSERT(pAllTargetCorPointSetRegister);

	const VecColVector3& AllSourcePointSet = pAllSourceCorPointSetRegister->getPointSet();
	const VecColVector3& AllSourceNormalSet = pAllSourceCorPointSetRegister->getNormalSet();
	const MatrixControlPoints& AllSourceControlPoints = pAllSourceCorPointSetRegister->getControlPoints();
	const std::vector<std::pair<double, double> >& AllSourceUVSet = pAllSourceCorPointSetRegister->getUVSet();
	const std::vector<std::pair<int, int> >& AllSurfaceIndexPairSet = pAllSourceCorPointSetRegister->getIndexPairSet();

	const VecColVector3& AllTargetPointSet	= pAllTargetCorPointSetRegister->getPointSet();
	const VecColVector3& AllTargetNormalSet = pAllTargetCorPointSetRegister->getNormalSet();
	const MatrixControlPoints& AllTargetControlPoints = pAllTargetCorPointSetRegister->getControlPoints();
	const std::vector<std::pair<double, double> >& AllTargetUVSet = pAllTargetCorPointSetRegister->getUVSet();
	const std::vector<std::pair<int, int> >& AllTargetIndexPairSet = pAllTargetCorPointSetRegister->getIndexPairSet();
	
	std::vector<std::pair<double, double> > SourcePrincipalCurvatureSet, TargetPrincipalCurvatureSet;
	__computePrincipalCurvatureSet(AllSourceUVSet, AllSurfaceIndexPairSet, AllSourceControlPoints, SourcePrincipalCurvatureSet);
	__computePrincipalCurvatureSet(AllTargetUVSet, AllTargetIndexPairSet, AllTargetControlPoints, TargetPrincipalCurvatureSet);

	VecColVector3 ValidSourcePointSet;
	VecColVector3 ValidTargetPointSet;
	VecColVector3 ValidSourceNormalSet;
	VecColVector3 ValidTargetNormalSet;

	for (unsigned int i=0; i<AllSourcePointSet.size(); ++i)
	{
		if ( fabs(fabs(SourcePrincipalCurvatureSet[i].first - TargetPrincipalCurvatureSet[i].first) - fabs(SourcePrincipalCurvatureSet[i].second - TargetPrincipalCurvatureSet[i].second)) /
			(fabs(SourcePrincipalCurvatureSet[i].first - TargetPrincipalCurvatureSet[i].first) + fabs(SourcePrincipalCurvatureSet[i].second - TargetPrincipalCurvatureSet[i].second)) 
			<= m_CurvatureConstraintThreshold )
		{
			ValidSourcePointSet.push_back(AllSourcePointSet[i]);
			ValidSourceNormalSet.push_back(AllSourceNormalSet[i]);
			ValidTargetPointSet.push_back(AllTargetPointSet[i]);
			ValidTargetNormalSet.push_back(AllTargetNormalSet[i]);
		}
	}

	voValidCorrespondencePairs = std::make_pair(makeSharedPtr(new CPntNormSubset()), makeSharedPtr(new CPntNormSubset()));
	CPntNormSubset* pCorSourcePointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.first.get());
	_ASSERT(pCorSourcePointCloudet);
	CPntNormSubset* pCorTargetPointCloudet = dynamic_cast<CPntNormSubset*>(voValidCorrespondencePairs.second.get());
	_ASSERT(pCorTargetPointCloudet);

#ifdef _DEBUG
	static int s_Int = 0;
	++s_Int;
	static char FileName[255];
	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\Valid\\%d_src.ply", s_Int);
	export2Ply(FileName, ValidSourcePointSet);
	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Mid\\Valid\\%d_tgt.ply", s_Int);
	export2Ply(FileName, ValidTargetPointSet);
#endif

	pCorSourcePointCloudet->swapPointSet(ValidSourcePointSet);
	pCorSourcePointCloudet->swapNormalSet(ValidSourceNormalSet);
	pCorTargetPointCloudet->swapPointSet(ValidTargetPointSet);
	pCorTargetPointCloudet->swapNormalSet(ValidTargetNormalSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionCurvature::__computePrincipalCurvatureSet( 
	const std::vector<std::pair<double, double> >& vUVSet, const std::vector<std::pair<int, int> >& vSurfaceIndexPairSet, 
	const MatrixControlPoints &vControlPoints, std::vector<std::pair<double, double> >& voPrincipalCurvatureSet ) const
{
	voPrincipalCurvatureSet.clear();
	Eigen::Matrix4d	CtrlPoints[3];
	int RowIndex, ColIndex;
	std::pair<double, double> PrincipleCurvatures;

	Bicubic::CBicubic BicubicObj;

	for (std::vector<std::pair<double, double>>::size_type i=0; i!=vUVSet.size(); ++i)
	{
		RowIndex = vSurfaceIndexPairSet[i].first;
		ColIndex = vSurfaceIndexPairSet[i].second;

		if ( !vControlPoints(RowIndex,	ColIndex  ).empty()	&& !vControlPoints(RowIndex+1, ColIndex  ).empty() && !vControlPoints(RowIndex+2, ColIndex  ).empty() && !vControlPoints(RowIndex+3, ColIndex  ).empty() &&
			 !vControlPoints(RowIndex,	ColIndex+1).empty()	&& !vControlPoints(RowIndex+1, ColIndex+1).empty() && !vControlPoints(RowIndex+2, ColIndex+1).empty() && !vControlPoints(RowIndex+3, ColIndex+1).empty() &&
			 !vControlPoints(RowIndex,	ColIndex+2).empty()	&& !vControlPoints(RowIndex+1, ColIndex+2).empty() && !vControlPoints(RowIndex+2, ColIndex+2).empty() && !vControlPoints(RowIndex+3, ColIndex+2).empty() &&
			 !vControlPoints(RowIndex,	ColIndex+3).empty()	&& !vControlPoints(RowIndex+1, ColIndex+3).empty() && !vControlPoints(RowIndex+2, ColIndex+3).empty() && !vControlPoints(RowIndex+3, ColIndex+3).empty() )		// FIXME: 这个判断还需要吗
		{
			CtrlPoints[0] <<
				vControlPoints(RowIndex, ColIndex  )[0](0, 0), vControlPoints(RowIndex+1, ColIndex  )[0](0, 0), vControlPoints(RowIndex+2, ColIndex  )[0](0, 0), vControlPoints(RowIndex+3, ColIndex  )[0](0, 0),
				vControlPoints(RowIndex, ColIndex+1)[0](0, 0), vControlPoints(RowIndex+1, ColIndex+1)[0](0, 0), vControlPoints(RowIndex+2, ColIndex+1)[0](0, 0), vControlPoints(RowIndex+3, ColIndex+1)[0](0, 0),
				vControlPoints(RowIndex, ColIndex+2)[0](0, 0), vControlPoints(RowIndex+1, ColIndex+2)[0](0, 0), vControlPoints(RowIndex+2, ColIndex+2)[0](0, 0), vControlPoints(RowIndex+3, ColIndex+2)[0](0, 0),	// FIXME
				vControlPoints(RowIndex, ColIndex+3)[0](0, 0), vControlPoints(RowIndex+1, ColIndex+3)[0](0, 0), vControlPoints(RowIndex+2, ColIndex+3)[0](0, 0), vControlPoints(RowIndex+3, ColIndex+3)[0](0, 0);

			CtrlPoints[1] <<
				vControlPoints(RowIndex, ColIndex  )[0](1, 0), vControlPoints(RowIndex+1, ColIndex  )[0](1, 0), vControlPoints(RowIndex+2, ColIndex  )[0](1, 0), vControlPoints(RowIndex+3, ColIndex  )[0](1, 0),
				vControlPoints(RowIndex, ColIndex+1)[0](1, 0), vControlPoints(RowIndex+1, ColIndex+1)[0](1, 0), vControlPoints(RowIndex+2, ColIndex+1)[0](1, 0), vControlPoints(RowIndex+3, ColIndex+1)[0](1, 0),
				vControlPoints(RowIndex, ColIndex+2)[0](1, 0), vControlPoints(RowIndex+1, ColIndex+2)[0](1, 0), vControlPoints(RowIndex+2, ColIndex+2)[0](1, 0), vControlPoints(RowIndex+3, ColIndex+2)[0](1, 0),
				vControlPoints(RowIndex, ColIndex+3)[0](1, 0), vControlPoints(RowIndex+1, ColIndex+3)[0](1, 0), vControlPoints(RowIndex+2, ColIndex+3)[0](1, 0), vControlPoints(RowIndex+3, ColIndex+3)[0](1, 0);

			CtrlPoints[2] <<
				vControlPoints(RowIndex, ColIndex  )[0](2, 0), vControlPoints(RowIndex+1, ColIndex  )[0](2, 0), vControlPoints(RowIndex+2, ColIndex  )[0](2, 0), vControlPoints(RowIndex+3, ColIndex  )[0](2, 0),
				vControlPoints(RowIndex, ColIndex+1)[0](2, 0), vControlPoints(RowIndex+1, ColIndex+1)[0](2, 0), vControlPoints(RowIndex+2, ColIndex+1)[0](2, 0), vControlPoints(RowIndex+3, ColIndex+1)[0](2, 0),
				vControlPoints(RowIndex, ColIndex+2)[0](2, 0), vControlPoints(RowIndex+1, ColIndex+2)[0](2, 0), vControlPoints(RowIndex+2, ColIndex+2)[0](2, 0), vControlPoints(RowIndex+3, ColIndex+2)[0](2, 0),
				vControlPoints(RowIndex, ColIndex+3)[0](2, 0), vControlPoints(RowIndex+1, ColIndex+3)[0](2, 0), vControlPoints(RowIndex+2, ColIndex+3)[0](2, 0), vControlPoints(RowIndex+3, ColIndex+3)[0](2, 0);

			BicubicObj.computePrincipleCurvatures(vUVSet[i].first, vUVSet[i].second, CtrlPoints, PrincipleCurvatures);
			voPrincipalCurvatureSet.push_back(PrincipleCurvatures);
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceRejectionCurvature::__parseCfg( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCurvatureConstraintThreshold(), m_CurvatureConstraintThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyDoCurvatureConstraint(), m_DoDoCurvatureConstraint);
	_ASSERT(DumpRes);
}
