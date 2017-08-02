#include "SamplerExtremeCurvature.h"
#include <cfloat>
#include <vector>
#include <boost/foreach.hpp>
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
#include "ICPConstGlobleValue.h"
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
bool hiveRegistration::CSamplerExtremeCurvature::__isMinCurvaturePoint(unsigned int vIdx, const hivePointCloud::CPointCloud& vPointCloud)
{
	const double* pGssCurvature = vPointCloud.getGssCurvaturePointer();
	const double CurCurvature   = *(pGssCurvature + vIdx);
	const double SquareDistThre = m_UnitSquareDist * m_SampleDistFactor;

	const double* pPos = vPointCloud.getPosPointer() + vIdx*3;
	std::vector<unsigned int> Neighbors;
	m_pPointCloudKDTree->executeKNN(pPos, Neighbors);

	if (CurCurvature < 0)
	{
		BOOST_FOREACH (unsigned int NeighborIndex, Neighbors)
		{
			if (comSquaredDist(vPointCloud.getPosPointer(), vIdx, vPointCloud.getPosPointer(), NeighborIndex, 3) > SquareDistThre) break;
			if (*(pGssCurvature + NeighborIndex) < CurCurvature)		
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

//*********************************************************************************
//FUNCTION:
bool CSamplerExtremeCurvature::__isCurvatureExtremePoint(unsigned int vIdx, const hivePointCloud::CPointCloud& vPointCloud)
{
	const double* pGssCurvature = vPointCloud.getGssCurvaturePointer();
	const double CurCurvature   = *(pGssCurvature + vIdx);
	const double SquareDistThre = m_UnitSquareDist * m_SampleDistFactor;

	const double* pPos = vPointCloud.getPosPointer() + vIdx*3;
	std::vector<unsigned int> Neighbors;
	m_pPointCloudKDTree->executeKNN(pPos, Neighbors);

	if (CurCurvature < 0)
	{
		BOOST_FOREACH (unsigned int NeighborIndex, Neighbors)
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
		BOOST_FOREACH (unsigned int NeighborIndex, Neighbors)
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
CorrespondencePointSet CSamplerExtremeCurvature::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	if (!m_pPointCloudKDTree.get())
	{
		__initPointCloudKDTree(vPointCloud);
	}

	m_UnitSquareDist = (dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()))->getUniqSquareDist());

	unsigned int NumPnts = vPointCloud.getNumPoints();
	std::vector<unsigned int> CorIdxSet;

	for (unsigned int i=0; i<NumPnts; ++i)
	{
		if (__isCurvatureExtremePoint(i, vPointCloud))
		{
			CorIdxSet.push_back(i);
		}
		//if (__isMinCurvaturePoint(i, vPointCloud))
		//{
		//	CorIdxSet.push_back(i);
		//}
	}

	const unsigned int MinCorPointNum = 10;
	_ASSERT(CorIdxSet.size() >= MinCorPointNum);

	return makeSharedPtr(new CIndexSubset(vPointCloud, CorIdxSet));
}

//*********************************************************************************
//FUNCTION:
void CSamplerExtremeCurvature::__initPointCloudKDTree(const hivePointCloud::CPointCloud& vPointCloud)
{
	const int Dim = 3;
	m_pPointCloudKDTree = boost::shared_ptr<hiveCommon::CKNNSearch>(new hiveCommon::CKNNSearch);
	m_pPointCloudKDTree->initKNNSearch(vPointCloud.getPosPointer(), vPointCloud.getNumPoints(), Dim, m_SampleNumNeib);
}