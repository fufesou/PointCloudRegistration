#include "UniqueData.h"
#include "ProductFactory.h"
#include "PointCloud.h"
#include "KNNSearch.h"
#include "RegUtilityFunctions.h"

hiveCommon::CProductFactory<hiveRegistration::CUniqueData> TheSrcCreator(hiveRegistration::CUniqueData::getSigSrcUniqueData());
hiveCommon::CProductFactory<hiveRegistration::CUniqueData> TheTgtCreator(hiveRegistration::CUniqueData::getSigTgtUniqueData());

hiveRegistration::CUniqueData::CUniqueData()
: m_UnitSquareDist(-1.0)
{

}

//*********************************************************************************
//FUNCTION:
hiveRegistration::CUniqueData::~CUniqueData()
{

}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CUniqueData::setPointCloud(const hivePointCloud::CPointCloud& vPointCloud)
{
	const double* pPos = vPointCloud.getPosPointer();
	const unsigned int NumPoints = vPointCloud.getNumPoints();
	unsigned int NumSamplePoints = 20;
	unsigned int SampleDist = NumPoints / NumSamplePoints;
	if (SampleDist < 1) SampleDist = 1;

	hiveCommon::CKNNSearch TmpKNN;
	TmpKNN.initKNNSearch(pPos, NumPoints, 3, 4);

	unsigned int SampleItr = 0;
	unsigned int SampleCount = 0;
	double SquaredDist = 0;
	std::vector<unsigned int> VecNeighbors;
	while (SampleItr < NumPoints)
	{
		TmpKNN.executeKNN(pPos + SampleItr*3, VecNeighbors);
		SquaredDist += comSquaredDist(pPos, SampleItr, pPos, VecNeighbors[0]);
		SquaredDist += comSquaredDist(pPos, SampleItr, pPos, VecNeighbors[1]);
		SquaredDist += comSquaredDist(pPos, SampleItr, pPos, VecNeighbors[2]);
		SquaredDist += comSquaredDist(pPos, SampleItr, pPos, VecNeighbors[3]);
		VecNeighbors.clear();
		SampleItr += SampleDist;
		++SampleCount;
	}

	m_UnitSquareDist = SquaredDist / (SampleCount * 4);
}
