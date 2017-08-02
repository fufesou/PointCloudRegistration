#include "RejectionCentroidDist.h"
#include <cmath>
#include <vector>
#include "KNNSearch.h"
#include "ProductFactoryData.h"
#include "ICPMacros.h"
#include "RegMath.h"
#include "ControlParameters.h"
#include "PointCloudSubset.h"

using namespace hiveRegistration;

hiveRegistration::CRejectionCentroidDist::CRejectionCentroidDist(void)
: m_DefaultNumNeighbors(25)
, m_DefaultMaxSearchRadius(8.0)
, m_DefaultCriticalDistFactor(1.0)
{

}

//*********************************************************************************
//FUNCTION:
void CRejectionCentroidDist::_rejectInvalidPairsV(const CorrespondencePairSet& vAllCorrespondencePairs, CorrespondencePairSet& voValidCorrespondencePairs)
{
	const unsigned int SrcPntSetSize = vAllCorrespondencePairs.first->getPointSet().size();
	const unsigned int TgtPntSetSize = vAllCorrespondencePairs.second->getPointSet().size();
	_ASSERT(SrcPntSetSize > 0 && SrcPntSetSize == TgtPntSetSize);

	__initKNN();

	std::vector<double> SquaredDistSet;
	__comDiffSquaredDist2CentroidVec(vAllCorrespondencePairs, SquaredDistSet);

	const double MeanDist = computeMeanValue(SquaredDistSet.begin(), SquaredDistSet.end());
	const double StandardVariance = computeStandardVariance(SquaredDistSet.begin(), SquaredDistSet.end(), MeanDist, POPULATION);

	__reject(vAllCorrespondencePairs, SquaredDistSet, MeanDist, StandardVariance, voValidCorrespondencePairs);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__initKNN()
{
	int NumNeighbors = m_DefaultNumNeighbors;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxSearchNum(), NumNeighbors);

	_ASSERT(m_pSourcePointCloud && m_pTargetPointCloud);

	if (!m_pSrcKNN.get())
	{
		m_pSrcKNN = boost::shared_ptr<hiveCommon::CKNNSearch>(new hiveCommon::CKNNSearch);
		m_pSrcKNN->initKNNSearch(m_pSourcePointCloud->getPosPointer(), m_pSourcePointCloud->getNumPoints(), 3, NumNeighbors);
	}

	if (!m_pTgtKNN.get())
	{
		m_pTgtKNN = boost::shared_ptr<hiveCommon::CKNNSearch>(new hiveCommon::CKNNSearch);
		m_pTgtKNN->initKNNSearch(m_pTargetPointCloud->getPosPointer(), m_pTargetPointCloud->getNumPoints(), 3, NumNeighbors);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__comDiffSquaredDist2CentroidVec(const CorrespondencePairSet& vAllCorrespondencePairs, std::vector<double>& voDiffSquaredDistVec)
{
	const VecColVector3& SrcPntSet = vAllCorrespondencePairs.first->getPointSet();
	const VecColVector3& TgtPntSet = vAllCorrespondencePairs.second->getPointSet();

	voDiffSquaredDistVec.reserve(SrcPntSet.size());
	voDiffSquaredDistVec.clear();

	for (int i=0; i<SrcPntSet.size(); ++i)
	{
		double TgtSquaredDist = __comSquaredDist2Centroid(TgtPntSet[i].data(), m_pTargetPointCloud, m_pTgtKNN);
		double SrcSquaredDist = __comSquaredDist2Centroid(SrcPntSet[i].data(), m_pSourcePointCloud, m_pSrcKNN);

		voDiffSquaredDistVec.push_back(std::abs(TgtSquaredDist - SrcSquaredDist));
	}
}

double hiveRegistration::CRejectionCentroidDist::__comSquaredDist2Centroid(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN)
{
	std::vector<unsigned int> Neighbors;
	double Centroid[3];
	__searchNeighbors(vTgtPnt, vPointCloud, vKNN, Neighbors);
	__computeCentroid(vTgtPnt, vPointCloud, Neighbors, Centroid);

	return 
		(vTgtPnt[0] - Centroid[0])*(vTgtPnt[0] - Centroid[0]) + 
		(vTgtPnt[1] - Centroid[1])*(vTgtPnt[1] - Centroid[1]) + 
		(vTgtPnt[2] - Centroid[2])*(vTgtPnt[2] - Centroid[2]);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__computeCentroid(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, const std::vector<unsigned int>& vNeighbors, double* voCentroid)
{
	_ASSERT(vTgtPnt && vPointCloud && voCentroid);

	const double* pPos = vPointCloud->getPosPointer();
	_ASSERT(pPos);

	voCentroid[0] = vTgtPnt[0];
	voCentroid[1] = vTgtPnt[1];
	voCentroid[2] = vTgtPnt[2];
	for (std::vector<unsigned int>::const_iterator citr=vNeighbors.begin(); citr!=vNeighbors.end(); ++citr)
	{
		const double* pNewPnt = pPos + 3*(*citr);
		voCentroid[0] += pNewPnt[0];
		voCentroid[1] += pNewPnt[1];
		voCentroid[2] += pNewPnt[2];
	}
	voCentroid[0] /= (vNeighbors.size() + 1);
	voCentroid[0] /= (vNeighbors.size() + 1);
	voCentroid[0] /= (vNeighbors.size() + 1);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__reject(const CorrespondencePairSet& vAllCorPairs, const std::vector<double>& vDistSet, double vMeanDist, double vStdVariance, CorrespondencePairSet& voValidCorPairs) const
{
	const VecColVector3& AllSrcPointSet = vAllCorPairs.first->getPointSet();
	const VecColVector3& AllTgtPointSet = vAllCorPairs.second->getPointSet();

	_ASSERT(AllSrcPointSet.size() > 0 && AllSrcPointSet.size() == AllTgtPointSet.size());

	VecColVector3 ValidSrcPointSet;
	VecColVector3 ValidTgtPointSet;

	double CriticalDistFactor;
	double DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCriticalDistFactor(), CriticalDistFactor);
	_ASSERT(DumpRes);

	const double Tolerance = CriticalDistFactor * vStdVariance;
	for (unsigned int i=0; i<AllSrcPointSet.size(); i++)
	{
		if( std::abs(vDistSet[i]-vMeanDist) <= Tolerance )
		{
			ValidSrcPointSet.push_back(AllSrcPointSet[i]);
			ValidTgtPointSet.push_back(AllTgtPointSet[i]);
		}
	}

	voValidCorPairs.first = makeSharedPtr(new CPntNormSubset);
	voValidCorPairs.second = makeSharedPtr(new CPntNormSubset);
	CPntNormSubset* pCorFirstPointSet = dynamic_cast<CPntNormSubset*>((voValidCorPairs.first).get());
	_ASSERT(pCorFirstPointSet);
	CPntNormSubset* pCorSecondPointSet = dynamic_cast<CPntNormSubset*>((voValidCorPairs.second).get());
	_ASSERT(pCorSecondPointSet);

	pCorFirstPointSet->swapPointSet(ValidSrcPointSet);
	pCorSecondPointSet->swapPointSet(ValidTgtPointSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__searchNeighbors(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, std::vector<unsigned int>& voNeighbors)
{
	__searchDesignedNumNeighbors(vTgtPnt, vKNN, voNeighbors);
	__searchDesignedRadiusNeighbors(vTgtPnt, vPointCloud, voNeighbors);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__searchDesignedRadiusNeighbors(const double* vTgtPnt, const hivePointCloud::CPointCloud* vPointCloud, std::vector<unsigned int>& vioNeighbors)
{
	double MaxSearchDist = m_DefaultMaxSearchRadius;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxSearchDist(), MaxSearchDist);
	_ASSERT(DumpRes);

	DistCmp CmpPred(vTgtPnt, vPointCloud);
	std::vector<unsigned int>::iterator Lower = std::lower_bound(vioNeighbors.begin(), vioNeighbors.end(), MaxSearchDist, CmpPred);
	vioNeighbors.erase(Lower, vioNeighbors.end());

	//_ASSERT(vioNeighbors.size() > 5);		//should the assertion be placed here?
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CRejectionCentroidDist::__searchDesignedNumNeighbors(const double* vTgtPnt, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, std::vector<unsigned int>& voNeighbors)
{
	int KNNNumNeighbor = m_DefaultNumNeighbors;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMaxSearchNum(), KNNNumNeighbor);
	_ASSERT(DumpRes);

	voNeighbors.reserve(KNNNumNeighbor);
	voNeighbors.clear();
	vKNN->executeKNN(vTgtPnt, voNeighbors);
}

hiveRegistration::CRejectionCentroidDist::DistCmp::DistCmp(const double* vPnt, const hivePointCloud::CPointCloud* vPointCloud)
: Ptr2Pnt(vPnt)
, Ptr2PntCldPos(vPointCloud->getPosPointer())
{

}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CRejectionCentroidDist::DistCmp::operator()(unsigned int vIdx, double vDesignedDist)
{
	double NewPnt[3] = { Ptr2PntCldPos[vIdx*3 + 0], Ptr2PntCldPos[vIdx*3 + 1], Ptr2PntCldPos[vIdx*3 + 2] };
	return (
		(NewPnt[0] - Ptr2Pnt[0]) * (NewPnt[0] - Ptr2Pnt[0]) + 
		(NewPnt[1] - Ptr2Pnt[1]) * (NewPnt[1] - Ptr2Pnt[1]) + 
		(NewPnt[2] - Ptr2Pnt[2]) * (NewPnt[2] - Ptr2Pnt[2])
		) < vDesignedDist;
}
