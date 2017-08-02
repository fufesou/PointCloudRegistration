#include "AuxiliaryConstraintLCSNS.h"
#include <time.h>
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "CommonLCSNSSubset.hpp"
#include "NormalShootingLCSNS.h"
#include "ControlPointsData.h"

// #ifdef _DEBUG
// #include <fstream>
// #endif

using namespace hiveRegistration;

CAuxiliaryConstraintLCSNS::CAuxiliaryConstraintLCSNS()
: m_pPairEstimationNormal2Plane(boost::shared_ptr<CNormalShootingLCSNS>(new CNormalShootingLCSNS))
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyPairThreshold(), 0.1);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyAuxiliarySearchRadius(), 0.05);

	__parseConfig();
}

CAuxiliaryConstraintLCSNS::~CAuxiliaryConstraintLCSNS()
{
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::doAuxiliaryPairConstraint(const CorrespondencePairSet& vInitCorPairSet, CorrespondencePairSet& voCorrespondencePointPairSet)
{
	__setInitCorrespondencePairSet(vInitCorPairSet);
	__setUpAuxiliaryCorrespondencePairSet();
	__doAuxiliaryPairConstraint(voCorrespondencePointPairSet);
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::__doAuxiliaryPairConstraint(CorrespondencePairSet& voCorrespondencePointPairSet)
{
	const CCommonLCSNSSubset<PtrControlPointsSet>* pInitSrcCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(m_pInitCorrespondencePairSet.first.get());
	const CCommonLCSNSSubset<PtrControlPointsSet>* pInitTgtCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(m_pInitCorrespondencePairSet.second.get());
	const CCommonLCSNSSubset<PtrControlPointsSet>* pAuxiSrcCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(m_pAuxiCorrespondencePairSet.first.get());
	const CCommonLCSNSSubset<PtrControlPointsSet>* pAuxiTgtCorPointSet = dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>(m_pAuxiCorrespondencePairSet.second.get());

	const PtrControlPointsSet& InitSrcCtrlPntsSet = pInitSrcCorPointSet->getControlPointsSet();
	const PtrControlPointsSet& InitTgtCtrlPntsSet = pInitTgtCorPointSet->getControlPointsSet();
	const VecColVector3& InitSrcPointSet = pInitSrcCorPointSet->getPointSet();
	const VecColVector3& InitTgtPointSet = pInitTgtCorPointSet->getPointSet();
	const VecColVector3& AuxiSrcPointSet = pAuxiSrcCorPointSet->getPointSet();
	const VecColVector3& AuxiTgtPointSet = pAuxiTgtCorPointSet->getPointSet();
	const VecColVector3& InitSrcNormalSet = pInitSrcCorPointSet->getNormalSet();
	const VecColVector3& InitTgtNormalSet = pInitTgtCorPointSet->getNormalSet();
	const VecColVector3& AuxiSrcNormalSet = pAuxiSrcCorPointSet->getNormalSet();
	const VecColVector3& AuxiTgtNormalSet = pAuxiTgtCorPointSet->getNormalSet();
	const std::vector<std::pair<int, int> >& InitSrcSurfIdxSet = pInitSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& InitTgtSurfIdxSet = pInitTgtCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AuxiSrcSurfIdxSet = pAuxiSrcCorPointSet->getIndexPairSet();
	const std::vector<std::pair<int, int> >& AuxiTgtSurfIdxSet = pAuxiTgtCorPointSet->getIndexPairSet();
	const std::vector<std::pair<double, double> >& InitSrcUVSet = pInitSrcCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& InitTgtUVSet = pInitTgtCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AuxiSrcUVSet = pAuxiSrcCorPointSet->getUVSet();
	const std::vector<std::pair<double, double> >& AuxiTgtUVSet = pAuxiTgtCorPointSet->getUVSet();

	PtrControlPointsSet SrcCtrlPntsSet(new ControlPointsSet);
	PtrControlPointsSet TgtCtrlPntsSet(new ControlPointsSet);
	VecColVector3 SrcPointSet;
	VecColVector3 TgtPointSet;
	VecColVector3 SrcNormalSet;
	VecColVector3 TgtNormalSet;
	std::vector<std::pair<int, int> > SrcSurfIdxSet;
	std::vector<std::pair<int, int> > TgtSurfIdxSet;
	std::vector<std::pair<double, double> > SrcUVSet;
	std::vector<std::pair<double, double> > TgtUVSet;

	int SizeMatchedSurfs = m_MapInitAuxi.size();

// #ifdef _DEBUG
// 	std::ofstream OutFile("TestData\\output\\CorPointPairs\\FineReg_Mid\\DistPiPii_QiQii.txt");
// #endif

//#pragma parallel for
	for (int i=0; i<m_MapInitAuxi.size(); ++i)
	{
		const ColVector3& CurrentInitSrcPoint = InitSrcPointSet[ m_MapInitAuxi[i] ];
		const ColVector3& CurrentInitTgtPoint = InitTgtPointSet[ m_MapInitAuxi[i] ];
		const ColVector3& CurrentAuxiSrcPoint = AuxiSrcPointSet[i];
		const ColVector3& CurrentAuxiTgtPoint = AuxiTgtPointSet[i];

		const double NormPiPii = (CurrentInitSrcPoint - CurrentAuxiSrcPoint).norm();
		const double NormQiQii = (CurrentInitTgtPoint - CurrentAuxiTgtPoint).norm();
		double TmpRatio = (fabs(NormPiPii - NormQiQii) / (NormPiPii + NormQiQii));

// #ifdef _DEBUG
// 		OutFile << TmpRatio << std::endl;
// #endif

		if ( TmpRatio < m_PairThreshold )
		{
			unsigned int CurInitIdx = m_MapInitAuxi[i];

			if ((rand() & 1) == 0)
			{
				SrcPointSet.push_back(CurrentInitSrcPoint);
				TgtPointSet.push_back(CurrentInitTgtPoint);
				SrcNormalSet.push_back(InitSrcNormalSet[CurInitIdx]);
				TgtNormalSet.push_back(InitTgtNormalSet[CurInitIdx]);
				SrcSurfIdxSet.push_back(InitSrcSurfIdxSet[CurInitIdx]);
				TgtSurfIdxSet.push_back(InitTgtSurfIdxSet[CurInitIdx]);
				SrcUVSet.push_back(InitSrcUVSet[CurInitIdx]);
				TgtUVSet.push_back(InitTgtUVSet[CurInitIdx]);
			} 
			else
			{
				SrcPointSet.push_back(CurrentAuxiSrcPoint);
				TgtPointSet.push_back(CurrentAuxiTgtPoint);
				SrcNormalSet.push_back(AuxiSrcNormalSet[i]);
				TgtNormalSet.push_back(AuxiTgtNormalSet[i]);
				SrcSurfIdxSet.push_back(AuxiSrcSurfIdxSet[i]);
				TgtSurfIdxSet.push_back(AuxiTgtSurfIdxSet[i]);
				SrcUVSet.push_back(AuxiSrcUVSet[i]);
				TgtUVSet.push_back(AuxiTgtUVSet[i]);
			}

			SrcCtrlPntsSet->push_back((*InitSrcCtrlPntsSet)[ CurInitIdx ]);
			TgtCtrlPntsSet->push_back((*InitTgtCtrlPntsSet)[ CurInitIdx ]);
		}
	}

// #ifdef _DEBUG
// 	OutFile.close();
// #endif
	
	CCommonLCSNSSubset<PtrControlPointsSet>* pSourceCorPointSet = new CCommonLCSNSSubset<PtrControlPointsSet>(SrcCtrlPntsSet);
	CCommonLCSNSSubset<PtrControlPointsSet>* pTargetCorPointSet = new CCommonLCSNSSubset<PtrControlPointsSet>(TgtCtrlPntsSet);
	pSourceCorPointSet->swapPointSet(SrcPointSet, SrcNormalSet, SrcSurfIdxSet, SrcUVSet);
	pTargetCorPointSet->swapPointSet(TgtPointSet, TgtNormalSet, TgtSurfIdxSet, TgtUVSet);
	voCorrespondencePointPairSet.first  = makeSharedPtr(pSourceCorPointSet);
	voCorrespondencePointPairSet.second = makeSharedPtr(pTargetCorPointSet);
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::__setInitCorrespondencePairSet(const CorrespondencePairSet& vInitCorPairSet)
{
	m_pInitCorrespondencePairSet = vInitCorPairSet;
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::__setUpAuxiliaryCorrespondencePairSet()
{
	CorrespondencePointSet AuxiliarySourceCorrespondencePointSet;
	__setUpAuxiliarySourceCorrespondencePointSet(AuxiliarySourceCorrespondencePointSet);
	__pairAuxiliaryCorrespondencePairSet(AuxiliarySourceCorrespondencePointSet);
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::__setUpAuxiliarySourceCorrespondencePointSet(
	CorrespondencePointSet& voAuxiSrcCorrespondencePointSet)
{
	const CCommonLCSNSSubset<PtrControlPointsSet>* pInitSrcCorPointSet =
		dynamic_cast<const CCommonLCSNSSubset<PtrControlPointsSet>*>((m_pInitCorrespondencePairSet.first).get());
	_ASSERT(pInitSrcCorPointSet);

	const PtrControlPointsSet& CtrlPntsSet = pInitSrcCorPointSet->getControlPointsSet();
	std::vector<std::pair<int, int> > IndexPairSet = pInitSrcCorPointSet->getIndexPairSet();
	std::vector<std::pair<double, double> > UVSet = pInitSrcCorPointSet->getUVSet();
	
	for (std::vector<std::pair<double, double> >::iterator Itr=UVSet.begin(); Itr!=UVSet.end(); ++Itr)
	{
		Itr->first  += ICP_RAND(-m_AuxiSreachRadius, m_AuxiSreachRadius);
		Itr->second += ICP_RAND(-m_AuxiSreachRadius, m_AuxiSreachRadius);
		if (Itr->first < 0) Itr->first = 0.00001;
		if (Itr->first > 1.0) Itr->first = 0.999999;
		if (Itr->second < 0) Itr->second = 0.00001;
		if (Itr->second > 1.0) Itr->second = 0.999999;
	}

	voAuxiSrcCorrespondencePointSet = makeSharedPtr(new CCommonLCSNSSubset<PtrControlPointsSet>(CtrlPntsSet));
	CCommonLCSNSSubset<PtrControlPointsSet>* pAuxilTgtCorrespondencePointSet = dynamic_cast<CCommonLCSNSSubset<PtrControlPointsSet>*>(voAuxiSrcCorrespondencePointSet.get());
	pAuxilTgtCorrespondencePointSet->swapUVSet(IndexPairSet, UVSet);
}

//*********************************************************************************
//FUNCTION:
void CAuxiliaryConstraintLCSNS::__pairAuxiliaryCorrespondencePairSet( const CorrespondencePointSet& vAuxiliarySourceCorrespondencePointSet )
{
	m_MapInitAuxi.clear();
	m_pPairEstimationNormal2Plane->determineCorrespondencePairSet(vAuxiliarySourceCorrespondencePointSet, m_pInitCorrespondencePairSet.second, m_pAuxiCorrespondencePairSet, m_MapInitAuxi);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CAuxiliaryConstraintLCSNS::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyPairThreshold(), m_PairThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyAuxiliarySearchRadius(), m_AuxiSreachRadius);
	_ASSERT(DumpRes);
}
