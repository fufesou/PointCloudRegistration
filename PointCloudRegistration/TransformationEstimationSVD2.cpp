#include "TransformationEstimationSVD2.h"
#include <Eigen/src/LU/Determinant.h>
#include <Eigen/SVD>
#include "ProductFactory.h"
#include "PointCloudSubset.h"
#include "RegUtilityFunctions.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CTransformationEstimationSVD2> TheCreator(CTransformationEstimationSVD2::getClassSig());

CTransformationEstimationSVD2::CTransformationEstimationSVD2()
{
	_letFactoryReleaseProduct();
}

void CTransformationEstimationSVD2::_computeRotationAndTranslationV(const CorrespondencePairSet& vCorrespondencePairs, Eigen::Matrix3d& vioR, ColVector3& vioT)
{
	_ASSERT(vCorrespondencePairs.first.get() && vCorrespondencePairs.second.get());

	const VecColVector3& SourcePointSet = vCorrespondencePairs.first->getPointSet();
	const VecColVector3& TargetPointSet = vCorrespondencePairs.second->getPointSet();

	Eigen::Matrix3d CurR;
	ColVector3 CurT;
	computeRotationAndTranslation(SourcePointSet, TargetPointSet, CurR, CurT);

	accumulateTransformation(vioR, vioT, CurR, CurT);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CTransformationEstimationSVD2::computeRotationAndTranslation(const VecColVector3& vPntSetA, const VecColVector3& vPntSetB, Eigen::Matrix3d& voR, ColVector3& voT)
{
	_ASSERT(vPntSetA.size() != 0 && vPntSetA.size() == vPntSetB.size());

	std::pair<ColVector3, ColVector3> CentroidPair;
	compute3DCentroid(vPntSetA, CentroidPair.first);
	compute3DCentroid(vPntSetB, CentroidPair.second);

	CorrespondencePairSet DemeanedCorPairSet;
	__deMean(vPntSetA, vPntSetB, CentroidPair, DemeanedCorPairSet);

	Eigen::Matrix<double, 3, Eigen::Dynamic> SrcDemeanMat;
	Eigen::Matrix<double, 3, Eigen::Dynamic> TgtDemeanMat;
	__buildPointPairMatrix(DemeanedCorPairSet, SrcDemeanMat, TgtDemeanMat);

	__computeRelativeRotationAndTranslation(SrcDemeanMat, TgtDemeanMat, CentroidPair, voR, voT);
}

//*********************************************************************************
//FUNCTION:
void CTransformationEstimationSVD2::__deMean(const VecColVector3& vSrcPointSet, const VecColVector3& vTgtPointSet, const std::pair<ColVector3, ColVector3>& vCentroidPair, CorrespondencePairSet& voDemeanedCorPairSet)
{
	VecColVector3 DemeanedSourcePointSet(vSrcPointSet.size());
	VecColVector3 DemeanedTargetPointSet(vTgtPointSet.size());

	const int PointSetSize = vSrcPointSet.size();

#pragma omp parallel for
	for (int i=0; i<PointSetSize; ++i)
	{
		DemeanedSourcePointSet[i] = vSrcPointSet[i] - vCentroidPair.first;
		DemeanedTargetPointSet[i] = vTgtPointSet[i] - vCentroidPair.second;
	}

	CPntNormSubset* pDemeanedSrcPointSet = new CPntNormSubset;
	CPntNormSubset* pDemeanedTgtPointSet = new CPntNormSubset;
	pDemeanedSrcPointSet->swapPointSet(DemeanedSourcePointSet);
	pDemeanedTgtPointSet->swapPointSet(DemeanedTargetPointSet);
	voDemeanedCorPairSet.first = makeSharedPtr(pDemeanedSrcPointSet);
	voDemeanedCorPairSet.second = makeSharedPtr(pDemeanedTgtPointSet);
}

//*********************************************************************************
//FUNCTION:
void CTransformationEstimationSVD2::__buildPointPairMatrix(const CorrespondencePairSet& vCorPairSet, Eigen::Matrix<double, 3, Eigen::Dynamic>& voPointMatA, Eigen::Matrix<double, 3, Eigen::Dynamic>& voPointMatB)
{
	const VecColVector3& PointSetA = vCorPairSet.first->getPointSet();
	const VecColVector3& PointSetB = vCorPairSet.second->getPointSet();
	const int SizePointA = PointSetA.size();

	_ASSERT(SizePointA != 0 && SizePointA == PointSetB.size());

	voPointMatA = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, SizePointA);
	voPointMatB = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, SizePointA);

#pragma omp parallel for
	for (int i=0; i<SizePointA; ++i)
	{
		voPointMatA.block<3, 1>(0, i) = PointSetA[i];
		voPointMatB.block<3, 1>(0, i) = PointSetB[i];
	}
}

//*********************************************************************************
//FUNCTION:
void CTransformationEstimationSVD2::__computeRelativeRotationAndTranslation(
	const Eigen::Matrix<double, 3, Eigen::Dynamic>& vSrcDemeanMat, const Eigen::Matrix<double, 3, Eigen::Dynamic>& vTgtDemeanMat, 
	const std::pair<ColVector3, ColVector3>& vCentroidPair, 
	Eigen::Matrix3d& voCurR, ColVector3& voCurT)
{
	Eigen::Matrix<double, 3, 3> H = vSrcDemeanMat * vTgtDemeanMat.transpose ();
	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > SVD(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 3, 3> U = SVD.matrixU();
	Eigen::Matrix<double, 3, 3> V = SVD.matrixV();

	if (U.determinant () * V.determinant () < 0)
	{
		V(0, 2) *= -1;
		V(1, 2) *= -1;
		V(2, 2) *= -1;
	}

	voCurR = V * U.transpose ();
	voCurT = vCentroidPair.second - voCurR * vCentroidPair.first;
}
