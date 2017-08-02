#include "PairEstimationMethodNormal2Plane.h"
#include <cmath>
#include "ICPMacros.h"
#include "ControlParameters.h"
#include "ProductFactoryData.h"
#include "KNNSearch.h"
#include "PointCloudSubset.h"
#include "SquareCellsInitialization.h"
#include "ControlPointsData.h"
#include "Bicubic.h"
#include ".\\RaySurfIntersection\\Intersection.h"
#include "UniqueData.h"

using namespace hiveRegistration;

CPairEstimationNormal2Plane::CPairEstimationNormal2Plane()
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySurfacePatchSize(), int(3));
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyCorPointsBuildingType(), int(0));

	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
void CPairEstimationNormal2Plane::determineCorrespondencePairSet(
	const CorrespondencePointSet& vTgtCorPointSet, const hivePointCloud::CPointCloud* vSourcePointCloud, 
	const Eigen::Matrix3d& vR, const ColVector3& vT, 
	CorrespondencePairSet& voCorPairSet )
{
	_ASSERT(vTgtCorPointSet.get() && vSourcePointCloud);

	const SControlPointsMatrixInfo* pTgtMatrix = hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance();

	MatrixControlPoints SrcCtlPoints;

	int CorPointsBuildingType;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCorPointsBuildingType(), CorPointsBuildingType);
	_ASSERT(DumpRes);

	if (0 == CorPointsBuildingType)
	{
		__computeControlPointsByMatrix(vSourcePointCloud, vR, vT, SrcCtlPoints);
	}
	else
	{
		__computeControlPointsByClosestPoint(vTgtCorPointSet, vSourcePointCloud, vR, vT, SrcCtlPoints);
	}

	const int SizeCtrlMatrixRow = pTgtMatrix->NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = pTgtMatrix->NumTotalRowAndCols.second;

	const CBicubicGridSubset* pCorPointSet = dynamic_cast<const CBicubicGridSubset*>(vTgtCorPointSet.get());

	const VecColVector3& CorPointSet = pCorPointSet->getPointSet();
	const VecColVector3& CorNormalSet = pCorPointSet->getNormalSet();
	const std::vector<std::pair<int, int> >& CorIndexPairSet = pCorPointSet->getIndexPairSet();
	const MatrixControlPoints& OriPoints = pCorPointSet->getControlPoints();
	
	std::vector<std::pair<int, int> > PairedRayIndexPairSet;
	std::vector<std::pair<int, int> > PairedSurfIndexPairSet;

	bool IsIntersected = false;
	SRegionIndexRange RegionIndexRange;
	std::pair<int, int> IntersectedIndexPair;

	std::vector<std::pair<double, double> > RayUVSet;
	std::vector<std::pair<double, double> > SurfUVSet;
	ColVector2 OtherUV;

	__computeMaxRayMarchLength(*pTgtMatrix);

	for (std::vector<std::pair<int, int> >::size_type i=0; i!=CorIndexPairSet.size(); ++i)
	{
		__computePatchIndexRange(CorIndexPairSet[i].first, SizeCtrlMatrixRow-3, RegionIndexRange.RowBegin, RegionIndexRange.RowEnd);
		__computePatchIndexRange(CorIndexPairSet[i].second, SizeCtrlMatrixCol-3, RegionIndexRange.ColBegin, RegionIndexRange.ColEnd);
		__computeIntersectPointWithEnoughSurfacePatchs(RegionIndexRange, CorPointSet[i] - CorNormalSet[i]*m_MaxRayMarchLength, CorNormalSet[i], SrcCtlPoints, IntersectedIndexPair, OtherUV, IsIntersected);

		if (IsIntersected)
		{
			PairedRayIndexPairSet.push_back(CorIndexPairSet[i]);
			PairedSurfIndexPairSet.push_back(IntersectedIndexPair);
			RayUVSet.push_back( std::pair<double, double>(0.5, 0.5) );
			SurfUVSet.push_back( std::pair<double, double>(OtherUV(0, 0), OtherUV(1, 0)) );
		}
	}

	voCorPairSet.first = makeSharedPtr(new CBicubicGridSubset(OriPoints));
	voCorPairSet.second = makeSharedPtr(new CBicubicGridSubset(SrcCtlPoints));
	CBicubicGridSubset* pPairedRayCorrespondence = dynamic_cast<CBicubicGridSubset*>(voCorPairSet.first.get());
	CBicubicGridSubset* pPairedSurfCorrespondence = dynamic_cast<CBicubicGridSubset*>(voCorPairSet.second.get());
	pPairedRayCorrespondence->swapUVSet(PairedRayIndexPairSet, RayUVSet);
	pPairedSurfCorrespondence->swapUVSet(PairedSurfIndexPairSet, SurfUVSet);
}

//*********************************************************************************
//FUNCTION:
void CPairEstimationNormal2Plane::__computeMaxRayMarchLength(const SControlPointsMatrixInfo &vMatrix)
{
	const RegionDim3& SurfacesRegion = vMatrix.Range;
	m_MaxRayMarchLength = std::sqrt(
		std::pow(SurfacesRegion(0, 1) - SurfacesRegion(0, 0), 2) + 
		std::pow(SurfacesRegion(1, 1) - SurfacesRegion(1, 0), 2) + 
		std::pow(SurfacesRegion(2, 1) - SurfacesRegion(2, 0), 2)
		);
}

//*********************************************************************************
//FUNCTION:
void CPairEstimationNormal2Plane::__computePatchIndexRange(const int vCenterIndex, const int vTotal, int& voBegin, int& vEnd) const
{
	_ASSERT(vCenterIndex>=0 && vCenterIndex<vTotal);

	const int SurfacePathSpan = m_SurfacePathSize>>1;

	if (vCenterIndex < SurfacePathSpan)
	{
		voBegin = 0;
		vEnd = vCenterIndex + SurfacePathSpan + 1;
	}
	else if (vCenterIndex > (vTotal - SurfacePathSpan - 1))
	{
		voBegin = vCenterIndex - SurfacePathSpan;
		vEnd = vTotal;
	}
	else
	{
		voBegin = vCenterIndex - SurfacePathSpan;
		vEnd = vCenterIndex + SurfacePathSpan + 1;
	}
}

//*********************************************************************************
//FUNCTION:
void CPairEstimationNormal2Plane::__computeIntersectPointWithEnoughSurfacePatchs(
	const SRegionIndexRange& vIndexRange, 
	const ColVector3 vRayOrigin, const ColVector3 vDir, 
	const hiveRegistration::MatrixControlPoints &vControlPoints,
	std::pair<int, int>& voIdxPair, hiveRegistration::ColVector2& voUV, bool& voIsIntersected)
{
	Eigen::Matrix4d	CtrlPoints[3];
	
	voIsIntersected = false;

	for (int i=vIndexRange.RowBegin; i<vIndexRange.RowEnd; ++i)
	{
		for (int k=vIndexRange.ColBegin; k<vIndexRange.ColEnd; ++k)
		{
			if ( !vControlPoints(i,	k  ).empty() && !vControlPoints(i+1, k  ).empty() && !vControlPoints(i+2, k  ).empty() && !vControlPoints(i+3, k  ).empty() &&
				 !vControlPoints(i,	k+1).empty() && !vControlPoints(i+1, k+1).empty() && !vControlPoints(i+2, k+1).empty() && !vControlPoints(i+3, k+1).empty() &&
				 !vControlPoints(i,	k+2).empty() && !vControlPoints(i+1, k+2).empty() && !vControlPoints(i+2, k+2).empty() && !vControlPoints(i+3, k+2).empty() &&
				 !vControlPoints(i,	k+3).empty() && !vControlPoints(i+1, k+3).empty() && !vControlPoints(i+2, k+3).empty() && !vControlPoints(i+3, k+3).empty() )
			{
				CtrlPoints[0] <<
					vControlPoints(i, k  )[0](0, 0), vControlPoints(i+1, k  )[0](0, 0), vControlPoints(i+2, k  )[0](0, 0), vControlPoints(i+3, k  )[0](0, 0),
					vControlPoints(i, k+1)[0](0, 0), vControlPoints(i+1, k+1)[0](0, 0), vControlPoints(i+2, k+1)[0](0, 0), vControlPoints(i+3, k+1)[0](0, 0),
					vControlPoints(i, k+2)[0](0, 0), vControlPoints(i+1, k+2)[0](0, 0), vControlPoints(i+2, k+2)[0](0, 0), vControlPoints(i+3, k+2)[0](0, 0),
					vControlPoints(i, k+3)[0](0, 0), vControlPoints(i+1, k+3)[0](0, 0), vControlPoints(i+2, k+3)[0](0, 0), vControlPoints(i+3, k+3)[0](0, 0);

				CtrlPoints[1] <<
					vControlPoints(i, k  )[0](1, 0), vControlPoints(i+1, k  )[0](1, 0), vControlPoints(i+2, k  )[0](1, 0), vControlPoints(i+3, k  )[0](1, 0),
					vControlPoints(i, k+1)[0](1, 0), vControlPoints(i+1, k+1)[0](1, 0), vControlPoints(i+2, k+1)[0](1, 0), vControlPoints(i+3, k+1)[0](1, 0),
					vControlPoints(i, k+2)[0](1, 0), vControlPoints(i+1, k+2)[0](1, 0), vControlPoints(i+2, k+2)[0](1, 0), vControlPoints(i+3, k+2)[0](1, 0),
					vControlPoints(i, k+3)[0](1, 0), vControlPoints(i+1, k+3)[0](1, 0), vControlPoints(i+2, k+3)[0](1, 0), vControlPoints(i+3, k+3)[0](1, 0);

				CtrlPoints[2] <<
					vControlPoints(i, k  )[0](2, 0), vControlPoints(i+1, k  )[0](2, 0), vControlPoints(i+2, k  )[0](2, 0), vControlPoints(i+3, k  )[0](2, 0),
					vControlPoints(i, k+1)[0](2, 0), vControlPoints(i+1, k+1)[0](2, 0), vControlPoints(i+2, k+1)[0](2, 0), vControlPoints(i+3, k+1)[0](2, 0),
					vControlPoints(i, k+2)[0](2, 0), vControlPoints(i+1, k+2)[0](2, 0), vControlPoints(i+2, k+2)[0](2, 0), vControlPoints(i+3, k+2)[0](2, 0),
					vControlPoints(i, k+3)[0](2, 0), vControlPoints(i+1, k+3)[0](2, 0), vControlPoints(i+2, k+3)[0](2, 0), vControlPoints(i+3, k+3)[0](2, 0);

				voIsIntersected = intersect(CtrlPoints, vRayOrigin, vDir, voUV);
				
				if (voIsIntersected)
				{
					voIdxPair.first = i;
					voIdxPair.second = k;
					return;
				}
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__computeControlPointsByMatrix( const hivePointCloud::CPointCloud* vSourcePointCloud, const Eigen::Matrix3d& vR, const ColVector3& vT, MatrixControlPoints& voMatrixControlPoints )
{
	Eigen::Matrix<double, 3, Eigen::Dynamic> Points;
	Points.resize(3, vSourcePointCloud->getNumPoints());
	::memcpy(Points.data(), vSourcePointCloud->getPosPointer(), vSourcePointCloud->getNumPoints() * 3 * sizeof(double));
	for (unsigned int i=0; i<vSourcePointCloud->getNumPoints(); ++i)
	{
		const ColVector3& TmpPnt = Points.block(0, i, 3, 1);
		Points.block(0, i, 3, 1) = vR * Points.block(0, i, 3, 1) + vT;
	}
	__genControlPointsMatrix(Points, voMatrixControlPoints);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__genControlPointsMatrix( const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, MatrixControlPoints& voControlPoints ) const
{
	__initControlPoints(vPoints, voControlPoints);
	__removeRedundantPoints(voControlPoints);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__removeRedundantPoints( MatrixControlPoints& vioControlPoints ) const
{
	const int SizeCtrlMatrixRow = hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance()->NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance()->NumTotalRowAndCols.second;

	for (int i=0; i<SizeCtrlMatrixRow; ++i)
	{
		for (int k=0; k<SizeCtrlMatrixCol; ++k)
		{
			if (vioControlPoints(i, k).size() > 1)
			{
				int RandomIndex = rand()%vioControlPoints(i, k).size();
				ColVector3 TmpPoint = vioControlPoints(i, k)[RandomIndex];
				vioControlPoints(i, k).swap(VecColVector3(1, TmpPoint));
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__initControlPoints( const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPoints, MatrixControlPoints& voControlPoints ) const
{
	const SControlPointsMatrixInfo& TgtMatrixInfo = *hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance();

	register double UnitLength0 = TgtMatrixInfo.SizeEachBlock.first;
	register double UnitLength1 = TgtMatrixInfo.SizeEachBlock.second;
	voControlPoints.resize(TgtMatrixInfo.NumTotalRowAndCols.first, TgtMatrixInfo.NumTotalRowAndCols.second);

	const int NumPoint = vPoints.cols();
	//#pragma omp parallel for
	for (int i=0; i<NumPoint; ++i)
	{
		const ColVector3& Point = vPoints.block(0, i, 3, 1);
		const int CellIndex0 = (Point(0, 0) - TgtMatrixInfo.Range(0, 0)) / UnitLength0;
		const int CellIndex1 = (Point(1, 0) - TgtMatrixInfo.Range(1, 0)) / UnitLength1;

		if (CellIndex0 > 0 && CellIndex0 < TgtMatrixInfo.NumTotalRowAndCols.first && CellIndex1 > 0 && CellIndex1 < TgtMatrixInfo.NumTotalRowAndCols.second)
		{
			voControlPoints(CellIndex0, CellIndex1).push_back(Point);
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::determineCorrespondencePairSet( 
	const CorrespondencePointSet& vSrcCorPointSet, const MatrixControlPoints &vTgtPoints, 
	CorrespondencePairSet& voCorPairSet, std::vector<unsigned int>& voIntersectedPointSet )
{
	_ASSERT(vSrcCorPointSet.get());

	const SControlPointsMatrixInfo& SrcMatrix = *hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance();

	const int SizeCtrlMatrixRow = SrcMatrix.NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = SrcMatrix.NumTotalRowAndCols.second;

	voIntersectedPointSet.clear();

	const CBicubicGridSubset* pCorPointSetRegister = dynamic_cast<const CBicubicGridSubset*>(vSrcCorPointSet.get());

	const VecColVector3& CorPointSet					     = pCorPointSetRegister->getPointSet();
	const VecColVector3& CorNormalSet                        = pCorPointSetRegister->getNormalSet();
	const MatrixControlPoints& SrcPoints					 = pCorPointSetRegister->getControlPoints();
	const std::vector<std::pair<double, double> >& CorUVSet	 = pCorPointSetRegister->getUVSet();
	const std::vector<std::pair<int, int> >& CorIndexPairSet = pCorPointSetRegister->getIndexPairSet();

	std::vector<std::pair<int, int> > PairedRayIdxPairSet;
	std::vector<std::pair<int, int> > PairedSurfIdxPairSet;

	bool IsIntersected = false;
	SRegionIndexRange RegionIndexRange;
	std::pair<int, int> IntersectedIndexPair;

	std::vector<std::pair<double, double> > PairedRayUVSet;
	std::vector<std::pair<double, double> > PairedSurfUVSet;
	ColVector2 OtherUV;

	__computeMaxRayMarchLength(SrcMatrix);

	for (unsigned i=0; i<CorIndexPairSet.size(); ++i)
	{
		__computePatchIndexRange(CorIndexPairSet[i].first, SizeCtrlMatrixRow-3, RegionIndexRange.RowBegin, RegionIndexRange.RowEnd);
		__computePatchIndexRange(CorIndexPairSet[i].second, SizeCtrlMatrixCol-3, RegionIndexRange.ColBegin, RegionIndexRange.ColEnd);
		__computeIntersectPointWithEnoughSurfacePatchs(RegionIndexRange, (CorPointSet[i] - CorNormalSet[i]*m_MaxRayMarchLength), CorNormalSet[i], vTgtPoints, IntersectedIndexPair, OtherUV, IsIntersected);

		if (IsIntersected)
		{
			PairedRayIdxPairSet.push_back(CorIndexPairSet[i]);
			PairedSurfIdxPairSet.push_back(IntersectedIndexPair);
			PairedRayUVSet.push_back(CorUVSet[i]);
			PairedSurfUVSet.push_back( std::make_pair(OtherUV(0, 0), OtherUV(1, 0)) );

			voIntersectedPointSet.push_back(i);
		}
	}

	voCorPairSet.first = makeSharedPtr(new CBicubicGridSubset(SrcPoints));
	voCorPairSet.second = makeSharedPtr(new CBicubicGridSubset(vTgtPoints));
	CBicubicGridSubset* pPairedIntersectCorrespondence = dynamic_cast<CBicubicGridSubset*>((voCorPairSet.first).get());
	CBicubicGridSubset* pPairedOtherCorrespondence = dynamic_cast<CBicubicGridSubset*>((voCorPairSet.second).get());
	pPairedIntersectCorrespondence->swapUVSet(PairedRayIdxPairSet, PairedRayUVSet);
	pPairedOtherCorrespondence->swapUVSet(PairedSurfIdxPairSet, PairedSurfUVSet);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__computeControlPointsByClosestPoint( const CorrespondencePointSet& vTargetCorrespondencePointSet, const hivePointCloud::CPointCloud* vSourcePointCloud, const Eigen::Matrix3d& vR, const ColVector3& vT, MatrixControlPoints& voMatrixControlPoints )
{
	const SControlPointsMatrixInfo& TgtMatrixInfo = *hiveCommon::CSingleton<SControlPointsMatrixInfo>::getInstance();
	const int SizeCtrlMatrixRow = TgtMatrixInfo.NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = TgtMatrixInfo.NumTotalRowAndCols.second;
	register double UnitLength0 = TgtMatrixInfo.SizeEachBlock.first;
	register double UnitLength1 = TgtMatrixInfo.SizeEachBlock.second;

	voMatrixControlPoints.resize(SizeCtrlMatrixRow, SizeCtrlMatrixCol);

	hivePointCloud::CPointCloud TmpPointCloud = *vSourcePointCloud;
	TmpPointCloud.transform(vR.data(), vT.data());
	const double* pTmpPos = TmpPointCloud.getPosPointer();

	hiveCommon::CKNNSearch KNN;
	KNN.initKNNSearch(TmpPointCloud.getPosPointer(), TmpPointCloud.getNumPoints(), 3, 1);
	std::vector<unsigned int> TmpNeighborSet(1);

	const CBicubicGridSubset* pCorPointSet = dynamic_cast<const CBicubicGridSubset*>(vTargetCorrespondencePointSet.get());
	_ASSERT(pCorPointSet);
	
	const MatrixControlPoints& MatCtrlPnts = pCorPointSet->getControlPoints();

	CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	double UnitSquareDist = pUniqueData->getUniqSquareDist();
	double SquareDistThreshold = m_PairSquareDistFactor * UnitSquareDist;

	int ClosestCount = 0;
	int LoopCount = 0;

	for (unsigned int i=0; i!=MatCtrlPnts.rows(); ++i)
	{
		for (unsigned int k=0; k!=MatCtrlPnts.cols(); ++k)
		{
			if (!MatCtrlPnts(i, k).empty())
			{
				KNN.executeKNN(MatCtrlPnts(i, k)[0].data(), TmpNeighborSet);
				ColVector3 TmpPoint(pTmpPos[TmpNeighborSet.back()*3], pTmpPos[TmpNeighborSet.back()*3 + 1], pTmpPos[TmpNeighborSet.back()*3 + 2]);

				double TmpSquareDist = (MatCtrlPnts(i, k)[0] - TmpPoint).squaredNorm();

				++LoopCount;
				if (TmpSquareDist < SquareDistThreshold)
				{
					voMatrixControlPoints(i, k).push_back(TmpPoint);
					++ClosestCount;
				}
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CPairEstimationNormal2Plane::__parseConfig( void )
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySurfacePatchSize(), m_SurfacePathSize);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyPairSquareDistFactor(), m_PairSquareDistFactor);
	_ASSERT(DumpRes);
}
