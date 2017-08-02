#include "SpinImagesGenerator.h"
#include <cmath>
#include <climits>
#include <algorithm>
#include <string>
#include <boost/assign.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/foreach.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "HiveCommonMicro.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "KNNSearch.h"
#include "RegMath.h"
#include "RegUtilityFunctions.h"
#include "PointCloud.h"
#include "UniqueData.h"

#ifdef _DEBUG
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include "TestUnitity.h"
#endif


using namespace hiveRegistration;


hiveRegistration::CSpinImagesGenerator::CSpinImagesGenerator(const hivePointCloud::CPointCloud& vPointCloud)
: m_pKNN(new hiveCommon::CKNNSearch)
, m_pPointCloud(&vPointCloud)
{
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyBinSize(), 1);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyResFactor(), 0.2);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyTopBeta(), 2);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyImageWidth(), 20);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyImageHeight(), 20);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeySupportAngle(), 180);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyNumKNNNeibs(), 100);
	PTR_CONTROL_PARAMS->setIfNotExist(getKeyBlockThreshold(), 0.2);

	__parseCtrlParams();

	double UnitSquareDist = dynamic_cast<CUniqueData*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CUniqueData::getSigTgtUniqueData()))->getUniqSquareDist();
	m_ResDist = std::sqrt(UnitSquareDist) * m_ResFact;
	m_SquareDistThreshold = m_SquareDistFactor * sqrt(UnitSquareDist);

	m_pKNN->initKNNSearch(vPointCloud.getPosPointer(), vPointCloud.getNumPoints(), 3, m_NumKNNNeibs);
}

hiveRegistration::CSpinImagesGenerator::~CSpinImagesGenerator()
{

}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSpinImagesGenerator::__parseCtrlParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyBinSize(), m_BinSize);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyResFactor(), m_ResFact);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyTopBeta(), m_TopBeta);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyImageWidth(), m_ImageWidth);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyImageHeight(), m_ImageHeight);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySupportAngle(), m_SupportAngle);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyNumKNNNeibs(), m_NumKNNNeibs);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyBlockThreshold(), m_BlockThreshold);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySquareDistFactor(), m_SquareDistFactor);
	_ASSERT(DumpRes);

	m_ImageWidth /= m_BinSize;
	m_ImageHeight /= m_BinSize;
	m_TopBeta /= m_BinSize;

	m_SpinBitMap.setwidth_height(m_ImageWidth, m_ImageHeight, true);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSpinImagesGenerator::genSpinImages(const std::vector<unsigned>& vSeeds, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voSpinImages) const
{
	voSpinImages.resize(m_ImageWidth * m_ImageHeight, vSeeds.size());
	::memset(voSpinImages.data(), 0, sizeof(double) * m_ImageWidth * m_ImageHeight * vSeeds.size());
	Eigen::Matrix<double, Eigen::Dynamic, 1> TmpImge;
	TmpImge.resize(m_ImageWidth * m_ImageHeight, 1);

	for (unsigned i=0; i<vSeeds.size(); ++i)
	{
		genImamge(vSeeds[i], TmpImge);
		voSpinImages.col(i) = TmpImge;
	}

#ifdef _DEBUG
	char FileName[255];
	static int s_IntImgsCount = 0;
	for (unsigned int i=0; i<vSeeds.size(); ++i)
	{
		m_SpinBitMap.clear();
		m_SpinBitMap.import_rgb(voSpinImages.col(i).data(), voSpinImages.col(i).data(), voSpinImages.col(i).data());
		sprintf(FileName, "TestData\\\output\\SpinImages\\%d_%d.bmp", s_IntImgsCount, vSeeds[i]);
		m_SpinBitMap.save_image(FileName);
	}
	++s_IntImgsCount;
#endif
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSpinImagesGenerator::genImamge(unsigned int vCentIdx, Eigen::Matrix<double, Eigen::Dynamic, 1>& voImg) const
{
	_ASSERT(voImg.rows() == m_ImageWidth * m_ImageHeight);
	::memset(voImg.data(), 0x00, sizeof(double) * m_ImageWidth * m_ImageHeight);

	double MaxAlpha = (m_ImageWidth - 1) * m_ResDist * m_BinSize;
	double MaxBeta = (m_ImageHeight - 1) * m_ResDist * m_BinSize;

	std::vector<unsigned> Neibs;
	m_pKNN->executeKNN(m_pPointCloud->getPosPointer() + vCentIdx*3, Neibs);

	for (std::vector<unsigned>::const_iterator citr=Neibs.begin(); citr!=Neibs.end(); ++citr)
	{
		std::pair<double, double> AlphaBetaDist = __comImageDist(vCentIdx, *citr);

		if (AlphaBetaDist.first < m_SquareDistThreshold)
		{
			if (AlphaBetaDist.first < MaxAlpha && AlphaBetaDist.second > 0 && AlphaBetaDist.second < MaxBeta)
			{
				std::pair<unsigned, unsigned> AlphaBetaIdx = __comImageIdxPair(AlphaBetaDist);
				std::pair<double, double> BilinearWeights = __comBilinearWeights(AlphaBetaDist, AlphaBetaIdx);

				static double ColStep = 0.3;

				_ASSERT(AlphaBetaIdx.first < (m_ImageWidth - 1) && AlphaBetaIdx.second < (m_ImageHeight - 1));

				voImg.data()[(AlphaBetaIdx.second + 0)*m_ImageWidth + (AlphaBetaIdx.first + 0)] += ColStep * (1 - BilinearWeights.first)*(1 - BilinearWeights.second);
				voImg.data()[(AlphaBetaIdx.second + 0)*m_ImageWidth + (AlphaBetaIdx.first + 1)] += ColStep * BilinearWeights.first*(1 - BilinearWeights.second);
				voImg.data()[(AlphaBetaIdx.second + 1)*m_ImageWidth + (AlphaBetaIdx.first + 0)] += ColStep * (1 - BilinearWeights.first) * BilinearWeights.second;
				voImg.data()[(AlphaBetaIdx.second + 1)*m_ImageWidth + (AlphaBetaIdx.first + 1)] += ColStep * BilinearWeights.first * BilinearWeights.second;
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
std::pair<double, double> hiveRegistration::CSpinImagesGenerator::__comImageDist(unsigned vCentPoint, unsigned vPixelPoint) const
{
	const double* pPos = m_pPointCloud->getPosPointer();
	const double* pNorm = m_pPointCloud->getNormalPointer();

	ColVector3 CentPos(pPos[vCentPoint*3], pPos[vCentPoint*3 + 1], pPos[vCentPoint*3 + 2]);
	ColVector3 PixelPos(pPos[vPixelPoint*3], pPos[vPixelPoint*3 + 1], pPos[vPixelPoint*3 + 2]);
	ColVector3 CentNorm(pNorm[vCentPoint*3], pNorm[vCentPoint*3 + 1], pNorm[vCentPoint*3 + 2]);
	ColVector3 Cent2Pixel = PixelPos - CentPos;
	CentNorm.normalize();
	double BetaDist = CentNorm.dot(Cent2Pixel);
	double AlphaSquareDist = Cent2Pixel.squaredNorm() - BetaDist*BetaDist;
	return std::make_pair(std::sqrt(AlphaSquareDist), m_TopBeta*m_ResDist*m_BinSize - BetaDist);
}

//*********************************************************************************
//FUNCTION:
std::pair<unsigned, unsigned> hiveRegistration::CSpinImagesGenerator::__comImageIdxPair(const std::pair<double, double>& vDistPair) const
{
	double BinWidth = m_ResDist * m_BinSize;
	unsigned int AlphaIdx = vDistPair.first / BinWidth;
	unsigned int BetaIdx = vDistPair.second / BinWidth;
	return std::make_pair(AlphaIdx, BetaIdx);
}

//*********************************************************************************
//FUNCTION:
std::pair<double, double> hiveRegistration::CSpinImagesGenerator::__comBilinearWeights(const std::pair<double, double>& vDistPair, const std::pair<unsigned, unsigned>& vIdxPair) const
{
	double BinWidth = m_ResDist * m_BinSize;
	double AlphaWeight = vDistPair.first / BinWidth - vIdxPair.first;
	double BetaWeight = vDistPair.second / BinWidth - vIdxPair.second;
	return std::make_pair(AlphaWeight, BetaWeight);
}