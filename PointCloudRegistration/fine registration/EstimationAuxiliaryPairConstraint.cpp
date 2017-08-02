#include "EstimationAuxiliaryPairConstraint.h"
#include <time.h>
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloudSubset.h"
#include "PairEstimationMethodNormal2Plane.h"
#include "ControlPointsData.h"
#include "Bicubic.h"

using namespace hiveRegistration;

CEstimationAuxiliaryPairConstraint::CEstimationAuxiliaryPairConstraint()
: m_pPairEstimationNormal2Plane(boost::shared_ptr<CPairEstimationNormal2Plane>(new CPairEstimationNormal2Plane))
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyPairThreshold(), 0.1);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyAuxiliarySearchRadius(), 0.05);
}

CEstimationAuxiliaryPairConstraint::~CEstimationAuxiliaryPairConstraint()
{
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::doAuxiliaryPairConstraint(const CorrespondencePairSet& vInitCorPairSet, CorrespondencePairSet& voCorrespondencePointPairSet)
{
	const CBicubicGridSubset* pInitTgtCorPointSet =
		dynamic_cast<const CBicubicGridSubset*>((vInitCorPairSet.first).get());
	_ASSERT(pInitTgtCorPointSet);

	__setInitCorrespondencePairSet(vInitCorPairSet);
	__setUpAuxiliaryCorrespondencePairSet(pInitTgtCorPointSet->getControlPoints());
	__doAuxiliaryPairConstraint(voCorrespondencePointPairSet);
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::__doAuxiliaryPairConstraint(CorrespondencePairSet& voCorrespondencePointPairSet)
{
	const CBicubicGridSubset* pInitTgtCorPointSet = dynamic_cast<const CBicubicGridSubset*>(m_pInitCorrespondencePairSet.first.get());
	const CBicubicGridSubset* pInitSrcCorPointSet = dynamic_cast<const CBicubicGridSubset*>(m_pInitCorrespondencePairSet.second.get());
	const CBicubicGridSubset* pAuxiTgtCorPointSet = dynamic_cast<const CBicubicGridSubset*>(m_pAuxiCorrespondencePairSet.first.get());
	const CBicubicGridSubset* pAuxiSrcCorPointSet = dynamic_cast<const CBicubicGridSubset*>(m_pAuxiCorrespondencePairSet.second.get());

	const VecColVector3& InitSrcPointSet = pInitSrcCorPointSet->getPointSet();
	const VecColVector3& InitTgtPointSet = pInitTgtCorPointSet->getPointSet();
	const VecColVector3& AuxiSrcPointSet = pAuxiSrcCorPointSet->getPointSet();
	const VecColVector3& AuxiTgtPointSet = pAuxiTgtCorPointSet->getPointSet();
	const std::vector<std::pair<double, double> >& InitSrcUVSet = pInitSrcCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& InitTgtUVSet = pInitTgtCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AuxiSrcUVSet = pAuxiSrcCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AuxiTgtUVSet = pAuxiTgtCorPointSet->getUVSet();
	const std::vector<std::pair<int, int> >& InitSourceIndexPairSet = pInitSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& InitTargetIndexPairSet = pInitTgtCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AuxiSourceIndexPairSet = pAuxiSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AuxiTargetIndexPairSet = pAuxiTgtCorPointSet->getIndexPairSet();

	std::vector<std::pair<double, double> > SrcUVSet;
	std::vector<std::pair<double, double> > TgtUVSet;
	std::vector<std::pair<int, int> > SrcIndexPairSet;
	std::vector<std::pair<int, int> > TgtIndexPairSet;

	double Threshold;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyPairThreshold(), Threshold);
	_ASSERT(DumpRes);

	for (VecColVector3::size_type i=0; i<m_MapInitAuxi.size(); ++i)
	{
		const unsigned int TmpInitPointIndex = m_MapInitAuxi[i];
		const unsigned int TmpAuxiPointIndex = i;

		const double NormPiPii = (InitSrcPointSet[TmpInitPointIndex] - AuxiSrcPointSet[TmpAuxiPointIndex]).norm();
		const double NormQiQii = (InitTgtPointSet[TmpInitPointIndex] - AuxiTgtPointSet[TmpAuxiPointIndex]).norm();
		const bool MeetConstraint = ((fabs(NormPiPii - NormQiQii) / (NormPiPii + NormQiQii)) <= Threshold);

		if ( MeetConstraint )		// FIXME: 这个地方控制点都来自InitCorrecepondencePointSet了，就不再判断筛选Auxilliary的uv了
		{
			SrcUVSet.push_back(InitTgtUVSet[TmpInitPointIndex]);
			TgtUVSet.push_back(InitTgtUVSet[TmpInitPointIndex]);
			SrcIndexPairSet.push_back(InitSourceIndexPairSet[TmpInitPointIndex]);
			TgtIndexPairSet.push_back(InitTargetIndexPairSet[TmpInitPointIndex]);
		}
	}

	CBicubicGridSubset* pSourceCorPointSet = new CBicubicGridSubset(pInitSrcCorPointSet->getControlPoints());
	CBicubicGridSubset* pTargetCorPointSet = new CBicubicGridSubset(pInitTgtCorPointSet->getControlPoints());
	pSourceCorPointSet->swapUVSet(SrcIndexPairSet, SrcUVSet);
	pTargetCorPointSet->swapUVSet(TgtIndexPairSet, TgtUVSet);
	voCorrespondencePointPairSet.first = makeSharedPtr(pSourceCorPointSet);
	voCorrespondencePointPairSet.second = makeSharedPtr(pTargetCorPointSet);
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::__setInitCorrespondencePairSet(const CorrespondencePairSet& vInitCorPairSet)
{
	m_pInitCorrespondencePairSet = vInitCorPairSet;
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::__setUpAuxiliaryCorrespondencePairSet( const MatrixControlPoints& vTgtPoints )
{
	CorrespondencePointSet AuxiliarySourceCorrespondencePointSet;
	__setUpAuxiliarySourceCorrespondencePointSet(AuxiliarySourceCorrespondencePointSet);
	__pairAuxiliaryCorrespondencePairSet(AuxiliarySourceCorrespondencePointSet, vTgtPoints);
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::__setUpAuxiliarySourceCorrespondencePointSet(
	CorrespondencePointSet& voAuxiSrcCorrespondencePointSet)
{
	double AuxiliarySearchRadius;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyAuxiliarySearchRadius(), AuxiliarySearchRadius);
	_ASSERT(DumpRes);

	const CBicubicGridSubset* pInitSrcCorPointSet =
		dynamic_cast<const CBicubicGridSubset*>((m_pInitCorrespondencePairSet.second).get());
	_ASSERT(pInitSrcCorPointSet);

	const std::vector<std::pair<double, double> >& UVSet = pInitSrcCorPointSet->getUVSet();
	const MatrixControlPoints& ControlPoints = pInitSrcCorPointSet->getControlPoints();
	std::vector<std::pair<int, int> > SurfaceIndexPairSet = pInitSrcCorPointSet->getIndexPairSet();
	
	std::vector<std::pair<double, double> > NewUVSet;
	Bicubic::CBicubic BiCubicObj;

	for (std::vector<std::pair<double, double> >::size_type i=0; i!=UVSet.size(); ++i)
	{
		NewUVSet.push_back( std::make_pair( UVSet[i].first+ICP_RAND(-AuxiliarySearchRadius, AuxiliarySearchRadius), 
											UVSet[i].second+ICP_RAND(-AuxiliarySearchRadius, AuxiliarySearchRadius) ) );
	}

	voAuxiSrcCorrespondencePointSet = makeSharedPtr(new CBicubicGridSubset(ControlPoints));
	CBicubicGridSubset* pAuxilTgtCorrespondencePointSet = dynamic_cast<CBicubicGridSubset*>(voAuxiSrcCorrespondencePointSet.get());
	pAuxilTgtCorrespondencePointSet->swapUVSet(SurfaceIndexPairSet, NewUVSet);
}

//*********************************************************************************
//FUNCTION:
void CEstimationAuxiliaryPairConstraint::__pairAuxiliaryCorrespondencePairSet(
	const CorrespondencePointSet& vAuxiliaryTargetCorrespondencePointSet, 
	const MatrixControlPoints& vTgtPoints)
{
	m_MapInitAuxi.clear();
	m_pPairEstimationNormal2Plane->determineCorrespondencePairSet(vAuxiliaryTargetCorrespondencePointSet, vTgtPoints, m_pAuxiCorrespondencePairSet, m_MapInitAuxi);
}