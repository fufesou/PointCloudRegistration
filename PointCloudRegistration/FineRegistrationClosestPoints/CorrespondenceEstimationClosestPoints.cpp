#include "CorrespondenceEstimationClosestPoints.h"
#include <vector>
#include "ProductFactory.h"
#include "KNNSearch.h"
#include "PointCloud.h"
#include "ProductFactoryData.h"
#include "ICPMacros.h"
#include "ControlParameters.h"
#include "ICPType.h"
#include "PointCloudSubset.h"
#include "UniqueData.h"

#ifdef _DEBUG
#include <fstream>
namespace
{
	std::ofstream OutFileNVD("TestData\\output\\CorPointPairs\\NormalVectorDist.txt");
	std::ofstream OutFileND("TestData\\output\\CorPointPairs\\NormalDeviation.txt");
}
#endif

using namespace hiveRegistration;

hiveCommon::CProductFactory<CCorrespondenceEstimationClosestPoints> TheCreator(CCorrespondenceEstimationClosestPoints::getClassSig());

CCorrespondenceEstimationClosestPoints::CCorrespondenceEstimationClosestPoints(void)
: m_pPointCloudKDTree(boost::shared_ptr<hiveCommon::CKNNSearch>(new hiveCommon::CKNNSearch))
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNumNeighbors(), 4);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNormalDeviationThreshold(), 0.85);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNormalVectorDistThreshold(), 0.0125);

	__parseConfig();

	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationClosestPoints::_setPointCloudV(const hivePointCloud::CPointCloud* vSourcePointCloud, const hivePointCloud::CPointCloud* vTargetPointCloud)
{
	_ASSERT(vSourcePointCloud && vTargetPointCloud);
	m_pSourcePointCloud = vSourcePointCloud;

	if (m_pTargetPointCloud != vTargetPointCloud)
	{
		m_pTargetPointCloud = vTargetPointCloud;
		__updateKDTree();
	}
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationClosestPoints::_determineCorrespondencesV(const CorrespondencePointSet& vSourceCorrespondencePointSet, const Eigen::Matrix3d& vR, const ColVector3& vT, CorrespondencePairSet& vioCorrespondencePairs)
{
	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	m_UniqueSquareDist = pUniqueData->getUniqSquareDist();

	std::vector<unsigned int> NeighborSet(m_NumNeighbors);
	const VecColVector3& SrcCorPointSet = vSourceCorrespondencePointSet->getPointSet();
	const VecColVector3& SrcCorNormalSet = vSourceCorrespondencePointSet->getNormalSet();

	VecColVector3 PairedSrcPointSet;
	VecColVector3 PairedTgtPointSet;
	VecColVector3 PairedSrcNormalSet;
	VecColVector3 PairedTgtNormalSet;

	const double* pTgtPos = m_pTargetPointCloud->getPosPointer();
	const double* pTgtNormal = m_pTargetPointCloud->getNormalPointer();

	const int SizeSrcCorPointSet = vSourceCorrespondencePointSet->getNumPoint();

	hivePointCloud::CPointCloud TmpPointCloud = *m_pSourcePointCloud;
	TmpPointCloud.transform(vR.data(), vT.data());
	const double* pTmpPos = TmpPointCloud.getPosPointer();

	hiveCommon::CKNNSearch KNN;
	KNN.initKNNSearch(TmpPointCloud.getPosPointer(), TmpPointCloud.getNumPoints(), 3, 1);
	std::vector<unsigned int> TmpNeighborSet(1);

	double Pi2NeibSquaredDistThreshold = m_UniqueSquareDist*m_NeighbourDistFactorThreshold;
	double Pi2QiSquaredDistThreshold = m_UniqueSquareDist*m_SquareDistFactorThreshold;

	for (int i=0; i<SizeSrcCorPointSet; ++i)
	{
		ColVector3 CurSrcPoint = vR*(SrcCorPointSet[i]) + vT;
		ColVector3 CurSrcNormal = vR * SrcCorNormalSet[i];
		NeighborSet.clear();
		m_pPointCloudKDTree->executeKNN(CurSrcPoint.data(), NeighborSet);

		ColVector3 TargetNormal(pTgtNormal[NeighborSet.back()*3], pTgtNormal[NeighborSet.back()*3 + 1], pTgtNormal[NeighborSet.back()*3 + 2]);
		ColVector3 TargetPoint(pTgtPos[NeighborSet.back()*3], pTgtPos[NeighborSet.back()*3 + 1], pTgtPos[NeighborSet.back()*3 + 2]);

		KNN.executeKNN(TargetPoint.data(), TmpNeighborSet);
		ColVector3 TmpPoint(pTmpPos[TmpNeighborSet.back()*3], pTmpPos[TmpNeighborSet.back()*3 + 1], pTmpPos[TmpNeighborSet.back()*3 + 2]);

		if ( ((CurSrcPoint - TargetPoint).squaredNorm() < Pi2QiSquaredDistThreshold) && 
			 ((CurSrcPoint - TmpPoint).squaredNorm() < Pi2NeibSquaredDistThreshold) )
		{
			PairedSrcPointSet.push_back(CurSrcPoint);
			PairedSrcNormalSet.push_back(CurSrcNormal);
			PairedTgtPointSet.push_back(TargetPoint);
			PairedTgtNormalSet.push_back(TargetNormal);
		}
		//__insertFitPairs(CurSrcPoint, CurSrcNormal, NeighborSet, PairedSrcPointSet, PairedSrcNormalSet, PairedTgtPointSet, PairedTgtNormalSet);
	}

	vioCorrespondencePairs.first = makeSharedPtr(new CPntNormSubset);
	vioCorrespondencePairs.second = makeSharedPtr(new CPntNormSubset);

	CPntNormSubset* pSrcCorPointSet = dynamic_cast<CPntNormSubset*>(vioCorrespondencePairs.first.get());
	CPntNormSubset* pTgtCorPointSet = dynamic_cast<CPntNormSubset*>(vioCorrespondencePairs.second.get());
	pSrcCorPointSet->swapPointSet(PairedSrcPointSet);
	pSrcCorPointSet->swapNormalSet(PairedSrcNormalSet);
	pTgtCorPointSet->swapPointSet(PairedTgtPointSet);
	pTgtCorPointSet->swapNormalSet(PairedTgtNormalSet);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationClosestPoints::__updateKDTree()
{
	m_pPointCloudKDTree->initKNNSearch(m_pTargetPointCloud->getPosPointer(), m_pTargetPointCloud->getNumPoints(), 3, m_NumNeighbors);
}

//*********************************************************************************
//FUNCTION:
void CCorrespondenceEstimationClosestPoints::__insertFitPairs(
	ColVector3& vSrcCorPoint, ColVector3& vSrcCorNormal, const std::vector<unsigned int>& vNeighbors, 
	VecColVector3& vioPairedSrcPointSet, VecColVector3& vioPairedSrcNormalSet, 
	VecColVector3& vioPairedTgtPointSet, VecColVector3& vioPairedTgtNormalSet)
{
	const int NoNormalCompare = 1;

	const double* pPos = m_pTargetPointCloud->getPosPointer();
	const double* pNormal = m_pTargetPointCloud->getNormalPointer();

	ColVector3 TmpTgtNormal;
	ColVector3 TmpTgtPoint;
	for (std::vector<unsigned int>::const_iterator citr = vNeighbors.begin(); citr!=vNeighbors.end(); ++citr)
	{
		TmpTgtNormal << pPos[(*citr)*3], pPos[(*citr)*3 + 1], pPos[(*citr)*3 + 2];
		TmpTgtNormal << pNormal[(*citr)*3], pNormal[(*citr)*3 + 1], pNormal[(*citr)*3 + 2];

		TmpTgtNormal.normalize();
		ColVector3 TmpVec = TmpTgtPoint - vSrcCorPoint;
		TmpVec.normalize();

		double NormalVectorDist = fabs(TmpVec.squaredNorm() - fabs(TmpVec.dot(TmpTgtNormal)));

#ifdef _DEBUG
		OutFileNVD << NormalVectorDist << std::endl;
#endif

		if ( NormalVectorDist < m_NormalVectorDistThreshold )
		{
			vSrcCorNormal.normalize();
			double NormalDeviation = fabs(TmpTgtNormal.dot(vSrcCorNormal));

#ifdef _DEBUG
			OutFileND << NormalDeviation << std::endl;
#endif

			if ( NoNormalCompare || NormalDeviation > m_NormalDeviationThreshold)
			{
				vioPairedTgtPointSet.push_back(TmpTgtPoint);
				vioPairedTgtNormalSet.push_back(TmpTgtNormal);
				vioPairedSrcPointSet.push_back(vSrcCorPoint);
				vioPairedSrcNormalSet.push_back(vSrcCorNormal);

				return;
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CCorrespondenceEstimationClosestPoints::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNormalDeviationThreshold(), m_NormalDeviationThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNormalVectorDistThreshold(), m_NormalVectorDistThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumNeighbors(), m_NumNeighbors);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySquareDistFactorThreshold(), m_SquareDistFactorThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNeighbourDistFactorThreshold(), m_NeighbourDistFactorThreshold);
	_ASSERT(DumpRes);
}
