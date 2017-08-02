#include "SamplerExtremeCurvature.h"
#include <cfloat>
#include <vector>
#include <boost/foreach.hpp>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloud.h"
#include "PointCloudSubset.h"
#include "KNNSearch.h"
#include "RegUtilityFunctions.h"
#include "UniqueData.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CSamplerExtremeCurvature> TheCreator(CSamplerExtremeCurvature::getClassSig());


hiveRegistration::CSamplerExtremeCurvature::CSamplerExtremeCurvature(void)
{
	_letFactoryReleaseProduct();
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerExtremeCurvature::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyExtremePointNumNeib(), m_SampleNumNeib);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyExtremePointDistFactor(), m_SampleDistFactor);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CSamplerExtremeCurvature::__isExtremeCurvaturePoint(const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIdx, const std::vector<unsigned>& vNeibs) const
{
	const double* pGssCurvature = vPointCloud.getGssCurvaturePointer();
	const double CurCurvature   = *(pGssCurvature + vIdx);
	const double SquareDistThre = m_UnitSquareDist * m_SampleDistFactor;

	if (CurCurvature < 0)
	{
		BOOST_FOREACH (unsigned int NeighborIndex, vNeibs)
		{
			if (*(pGssCurvature + NeighborIndex) < CurCurvature)		
			{
				return false;
			}
		}
	}
	else
	{
		BOOST_FOREACH (unsigned int NeighborIndex, vNeibs)
		{
			if (*(pGssCurvature + NeighborIndex) > CurCurvature)	
			{
				return false;
			}
		}
	}

	return true;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CSamplerExtremeCurvature::__isExtremeCurvaturePointWithinDist(const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIdx, const std::vector<unsigned>& vNeibs) const
{
	const double* pGssCurvature = vPointCloud.getGssCurvaturePointer();
	const double CurCurvature   = *(pGssCurvature + vIdx);
	const double SquareDistThre = m_UnitSquareDist * m_SampleDistFactor;

	if (CurCurvature < 0)
	{
		BOOST_FOREACH (unsigned int NeighborIndex, vNeibs)
		{
			if (comSquaredDist(vPointCloud.getPosPointer(), vIdx, vPointCloud.getPosPointer(), NeighborIndex, 3) > SquareDistThre) break;
			if (*(pGssCurvature + NeighborIndex) < CurCurvature)		
			{
				return false;
			}
		}
	}
	else
	{
		BOOST_FOREACH (unsigned int NeighborIndex, vNeibs)
		{
			if (comSquaredDist(vPointCloud.getPosPointer(), vIdx, vPointCloud.getPosPointer(), NeighborIndex, 3) > SquareDistThre) break;
			if (*(pGssCurvature + NeighborIndex) > CurCurvature)	
			{
				return false;
			}
		}
	}

	return true;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerExtremeCurvature::__firstSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& voSubset) const
{
	const double* pPos = vPointCloud.getPosPointer();
	const unsigned NumPnts = vPointCloud.getNumPoints();

	hiveCommon::CKNNSearch TmpKNN;
	TmpKNN.initKNNSearch(pPos, NumPnts, 3, m_FirstSampleNumNeib);

	voSubset.clear();
	for (unsigned int i=0; i<NumPnts; ++i)
	{
		std::vector<unsigned> Neibs;
		TmpKNN.executeKNN(pPos + i*3, Neibs);
		if (__isExtremeCurvaturePoint(vPointCloud, i, Neibs)) voSubset.push_back(i);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerExtremeCurvature::__secondSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& vioSubset) const
{
	const double* pPos = vPointCloud.getPosPointer();
	Eigen::Matrix<double, 3, Eigen::Dynamic> SubsetPoints;
	SubsetPoints.resize(3, vioSubset.size());
	for (unsigned int i=0; i<vioSubset.size(); ++i)
	{
		SubsetPoints(0, i) = pPos[vioSubset[i]*3    ];
		SubsetPoints(1, i) = pPos[vioSubset[i]*3 + 1];
		SubsetPoints(2, i) = pPos[vioSubset[i]*3 + 2];
	}

	hiveCommon::CKNNSearch TmpKNN;
	TmpKNN.initKNNSearch(SubsetPoints.data(), SubsetPoints.cols(), 3, m_SecondSampleNumNeib);

	std::vector<unsigned> TmpSubset;
	for (std::vector<unsigned>::const_iterator CItr=vioSubset.begin(); CItr!=vioSubset.end(); ++CItr)
	{
		std::vector<unsigned> Neibs;
		TmpKNN.executeKNN(pPos + (*CItr)*3, Neibs);
		if (__isExtremeCurvaturePointWithinDist(vPointCloud, *CItr, Neibs)) TmpSubset.push_back(*CItr);
	}

	vioSubset.swap(TmpSubset);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerExtremeCurvature::__init(void)
{
	const CUniqueData* pUniqueData = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()));
	_ASSERT(pUniqueData);
	m_UnitSquareDist = pUniqueData->getUniqSquareDist();

	m_FirstSampleNumNeib = std::sqrt(1.0 * m_SampleNumNeib);
	m_SecondSampleNumNeib = (1.0 * m_SampleNumNeib / m_FirstSampleNumNeib) + 0.99;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerExtremeCurvature::__simpleSample(const hivePointCloud::CPointCloud& vPointCloud, std::vector<unsigned>& voSubset) const
{
	const double* pPos = vPointCloud.getPosPointer();
	const unsigned NumPnts = vPointCloud.getNumPoints();

	hiveCommon::CKNNSearch TmpKNN;
	TmpKNN.initKNNSearch(pPos, NumPnts, 3, m_SampleNumNeib);

	voSubset.clear();
	for (unsigned int i=0; i<NumPnts; ++i)
	{
		std::vector<unsigned> Neibs;
		TmpKNN.executeKNN(pPos + i*3, Neibs);
		if (__isExtremeCurvaturePointWithinDist(vPointCloud, i, Neibs)) voSubset.push_back(i);
	}
}

//*********************************************************************************
//FUNCTION:
CorrespondencePointSet CSamplerExtremeCurvature::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	__init();

	std::vector<unsigned int> IdxSubset;
	__firstSample(vPointCloud, IdxSubset);
	__secondSample(vPointCloud, IdxSubset);
	//__simpleSample(vPointCloud, IdxSubset);

	const unsigned int MinCorPointNum = 10;
	_ASSERT(IdxSubset.size() >= MinCorPointNum);
	return makeSharedPtr(new CIndexSubset(vPointCloud, IdxSubset, true));
}