#include "LCPTriangle.h"
#include <algorithm>
#include <fstream>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloud.h"
#include "PointCloudSubset.h"
#include "BaseSampler.h"
#include "TestUnitity.h"
#include "TransformationEstimationSVD2.h"


//#ifdef _DEBUG
namespace
{
	static unsigned gs_BestFitPairs[6];

	void exportCurrentFitInfo(unsigned vCongruentCount, double vFitRatio)
	{
		std::ofstream FitInfoFile("TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\FitInfo.txt");

		FitInfoFile << " BestFittedPairs: " << std::endl;
		FitInfoFile << gs_BestFitPairs[0] << " " << gs_BestFitPairs[3] << std::endl;
		FitInfoFile << gs_BestFitPairs[1] << " " << gs_BestFitPairs[4] << std::endl;
		FitInfoFile << gs_BestFitPairs[2] << " " << gs_BestFitPairs[5] << std::endl;
		FitInfoFile << "Best Congruent Count : \n" << vCongruentCount << std::endl;
		FitInfoFile << "Fit Ratio : \n" << vFitRatio << std::endl;
		FitInfoFile.close();
	}
}
//#endif


namespace
{
	struct CmpPairedPntGroup
	{
		bool operator()(const std::pair<unsigned, std::vector<unsigned> >& vL, const std::pair<unsigned, std::vector<unsigned> >& vR)
		{
			return vL.second.size() < vR.second.size();
		}
	};

	struct CmpCongruentIdx 
	{
		CmpCongruentIdx(const std::vector<unsigned>& vVecCongruent) : m_VecCongruent(vVecCongruent) {}
		const std::vector<unsigned>& m_VecCongruent;
		bool operator()(unsigned vL, unsigned vR)
		{
			return m_VecCongruent[vL] > m_VecCongruent[vR];
		}
	};
}

hiveRegistration::CLCPTriangle::CLCPTriangle(const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud)
: m_LoopPointCloud(vLoopPointCloud)
, m_MatchPointCloud(vMatchPointCloud)
{
	__parseParams();

	m_MatchKNN.initKNNSearch(m_MatchPointCloud.getPosPointer(), m_MatchPointCloud.getNumPoints(), 3, 1);
	CBaseSampler* pSampler = dynamic_cast<CBaseSampler*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(m_SampleStrID));
	_ASSERT(pSampler);
	m_LoopSamplePointSet = pSampler->samplePointCloud(m_LoopPointCloud);
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CLCPTriangle::comRotationAndTranslation(
	std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, 
	double vUnitSquareDist, 
	Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
	m_UnitSquareDist = vUnitSquareDist;

	std::sort(vInitPairedPoints.begin(), vInitPairedPoints.end(), CmpPairedPntGroup());

	double MinTriEdeLen = m_TriangleEdgeFactor * m_TriangleEdgeFactor * m_UnitSquareDist;

	Eigen::Matrix3d CurR;
	Eigen::Vector3d CurT;
	unsigned BestCongruentCount = 0;
	unsigned CurCongruentCount = 0;
	unsigned EarlyReturnThreshold = m_EarlyReturnRatio * m_LoopSamplePointSet->getPointSet().size();
	unsigned AcceptThreshold = m_AcceptMatchRatio * m_LoopSamplePointSet->getPointSet().size();
	for (unsigned i=0; i<vInitPairedPoints.size() - 2; ++i)
	{
		for (unsigned k=i+1; k<vInitPairedPoints.size() - 1; ++k)
		{
			double LoopDistAB = comSquaredDist(m_LoopPointCloud.getPosPointer(), vInitPairedPoints[i].first, m_LoopPointCloud.getPosPointer(), vInitPairedPoints[k].first, 3);
			if (LoopDistAB < MinTriEdeLen)
			{
				continue;
			}

			for (unsigned j=k+1; j<vInitPairedPoints.size(); ++j)
			{
				CurCongruentCount = __comOptimalRT(vInitPairedPoints, i, k, j, CurR, CurT);
				if (CurCongruentCount > BestCongruentCount)
				{
					voR = CurR;
					voT = CurT;
					BestCongruentCount = CurCongruentCount;

					if (BestCongruentCount > EarlyReturnThreshold)
					{
//#ifdef _DEBUG
						double FitRatio = computeCurrentFitRatio(vInitPairedPoints, voR, voT, m_LoopPointCloud, m_MatchPointCloud, (m_CongruentFactor) * m_UnitSquareDist);
						exportCurrentFitInfo(BestCongruentCount, FitRatio);
//#endif
						return true;
					}
				}
			}
		}
	}

//#ifdef _DEBUG
	double FitRatio = computeCurrentFitRatio(vInitPairedPoints, voR, voT, m_LoopPointCloud, m_MatchPointCloud, m_CongruentFactor * m_UnitSquareDist);
	exportCurrentFitInfo(BestCongruentCount, FitRatio);
//#endif

	return (BestCongruentCount > AcceptThreshold);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__parseParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCongruentFactor(), m_CongruentFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyEarlyReturnRatio(), m_EarlyReturnRatio);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyAcceptMatchRatio(), m_AcceptMatchRatio);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySimTriFactor(), m_SimilarTriangleFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyTriangleEdgeFactor(), m_TriangleEdgeFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySamplerStrID(), m_SampleStrID);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__comRTWith3PntPairs(const std::pair<unsigned, unsigned> vPntPairs[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT) const
{
#ifdef _DEBUG
	std::ofstream TrianglesFile("TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\PairedTriangles.txt", std::ios::app);
	TrianglesFile << vPntPairs[0].first << " " << vPntPairs[0].second << std::endl;
	TrianglesFile << vPntPairs[1].first << " " << vPntPairs[1].second << std::endl;
	TrianglesFile << vPntPairs[2].first << " " << vPntPairs[2].second << std::endl;
	TrianglesFile << std::endl;
	TrianglesFile.close();
#endif

	const double* pLoopPos = m_LoopPointCloud.getPosPointer();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();

	VecColVector3 LoopPntSet;
	VecColVector3 MatchPntSet;
	for (unsigned i=0; i<3; ++i)
	{
		LoopPntSet.push_back(ColVector3(pLoopPos[vPntPairs[i].first*3 + 0], pLoopPos[vPntPairs[i].first*3 + 1], pLoopPos[vPntPairs[i].first*3 + 2]));
		MatchPntSet.push_back(ColVector3(pMatchPos[vPntPairs[i].second*3 + 0], pMatchPos[vPntPairs[i].second*3 + 1], pMatchPos[vPntPairs[i].second*3 + 2]));
	}
	CTransformationEstimationSVD2* pTESVD = dynamic_cast<CTransformationEstimationSVD2*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CTransformationEstimationSVD2::getClassSig()));
	pTESVD->computeRotationAndTranslation(LoopPntSet, MatchPntSet, voR, voT);

//#ifdef _DEBUG
//	double SquareDist = (voR * LoopPntSet[0] + voT - MatchPntSet[0]).squaredNorm();
//	_ASSERT(SquareDist < m_UnitSquareDist * m_SimilarTriangleFactor);
//	SquareDist = (voR * LoopPntSet[1] + voT - MatchPntSet[1]).squaredNorm();
//	_ASSERT(SquareDist < m_UnitSquareDist * m_SimilarTriangleFactor);
//	SquareDist = (voR * LoopPntSet[2] + voT - MatchPntSet[2]).squaredNorm();
//	_ASSERT(SquareDist < m_UnitSquareDist * m_SimilarTriangleFactor);
//#endif
}

//*********************************************************************************
//FUNCTION:
unsigned hiveRegistration::CLCPTriangle::__findNumPntsInTolerance(const Eigen::Matrix3d& vR, const Eigen::Vector3d& vT)
{
	unsigned CongruentCount = 0;
	double CongruentDist = m_CongruentFactor * m_UnitSquareDist;
	const VecColVector3& LoopSamplePointSet = m_LoopSamplePointSet->getPointSet();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();
	for (VecColVector3::const_iterator CItr=LoopSamplePointSet.begin(); CItr!=LoopSamplePointSet.end(); ++CItr)
	{
		const Eigen::Vector3d NewLoopPnt = vR * (*CItr) + vT;
		std::vector<unsigned> Neibs;
		m_MatchKNN.executeKNN(NewLoopPnt.data(), Neibs);
		double SquareDist = comSquaredDist(NewLoopPnt.data(), 0, pMatchPos, Neibs.front(), 3);
		CongruentCount += (SquareDist < CongruentDist);
	}

	return CongruentCount;
}

//*********************************************************************************
//FUNCTION:
unsigned hiveRegistration::CLCPTriangle::__comOptimalRT(
	const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, 
	unsigned vI, unsigned vK, unsigned vJ, 
	Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
	unsigned BestCongruentCount = 0;
	unsigned CurCongruentCount = 0;

	Eigen::Matrix3d CurR;
	Eigen::Vector3d CurT;

	const double* pLoopPos = m_LoopPointCloud.getPosPointer();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();

	double TriDistTol = m_SimilarTriangleFactor * m_SimilarTriangleFactor * m_UnitSquareDist;
	double MinTriEdeLen = m_TriangleEdgeFactor * m_TriangleEdgeFactor * m_UnitSquareDist;

	unsigned LoopPnt[3] = { vInitPairedPoints[vI].first, vInitPairedPoints[vK].first, vInitPairedPoints[vJ].first };

	double LoopDistAB = comSquaredDist(pLoopPos, vInitPairedPoints[vI].first, pLoopPos, vInitPairedPoints[vK].first, 3);
	double LoopDistBC = comSquaredDist(pLoopPos, vInitPairedPoints[vK].first, pLoopPos, vInitPairedPoints[vJ].first, 3);
	double LoopDistAC = comSquaredDist(pLoopPos, vInitPairedPoints[vI].first, pLoopPos, vInitPairedPoints[vJ].first, 3);

	if (LoopDistAC < MinTriEdeLen || LoopDistBC < MinTriEdeLen)
	{
		return 0;
	}

	const std::vector<unsigned>& CandI = vInitPairedPoints[vI].second;
	const std::vector<unsigned>& CandK = vInitPairedPoints[vK].second;
	const std::vector<unsigned>& CandJ = vInitPairedPoints[vJ].second;

	for (std::vector<unsigned>::const_iterator CItrI=CandI.begin(); CItrI!=CandI.end(); ++CItrI)
	{
		for (std::vector<unsigned>::const_iterator CItrK=CandK.begin(); CItrK!=CandK.end(); ++CItrK)
		{
			double MatchDistAB = comSquaredDist(pMatchPos, *CItrI, pMatchPos, *CItrK, 3);
			if (fabs(MatchDistAB - LoopDistAB) < TriDistTol)
			{
				for (std::vector<unsigned>::const_iterator CItrJ=CandJ.begin(); CItrJ!=CandJ.end(); ++CItrJ)
				{
					double MatchDistAC = comSquaredDist(pMatchPos, *CItrI, pMatchPos, *CItrJ, 3);
					if (fabs(MatchDistAC - LoopDistAC) < TriDistTol)
					{
						double MatchDistBC = comSquaredDist(pMatchPos, *CItrK, pMatchPos, *CItrJ, 3);
						if (fabs(MatchDistBC - LoopDistBC) < TriDistTol)
						{
							std::pair<unsigned, unsigned> PntPairs[3];
							PntPairs[0].first = vInitPairedPoints[vI].first; PntPairs[0].second = *CItrI;
							PntPairs[1].first = vInitPairedPoints[vK].first; PntPairs[1].second = *CItrK;
							PntPairs[2].first = vInitPairedPoints[vJ].first; PntPairs[2].second = *CItrJ;

							__comRTWith3PntPairs(PntPairs, CurR, CurT);
							CurCongruentCount = __findNumPntsInTolerance(CurR, CurT);

							if (CurCongruentCount > BestCongruentCount)
							{
								BestCongruentCount = CurCongruentCount;
								voR = CurR;
								voT = CurT;

//#ifdef _DEBUG
								gs_BestFitPairs[0] = PntPairs[0].first; gs_BestFitPairs[3] = PntPairs[0].second;
								gs_BestFitPairs[1] = PntPairs[1].first; gs_BestFitPairs[4] = PntPairs[1].second;
								gs_BestFitPairs[2] = PntPairs[2].first; gs_BestFitPairs[5] = PntPairs[2].second;
//#endif
							}
						}
					}
				}
			}
		}
	}

	return BestCongruentCount;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CLCPTriangle::comRTWithLastElement(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, double vUnitSquareDist, Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
	if (vInitPairedPoints.size() < 3) return false;

	m_UnitSquareDist = vUnitSquareDist;

	double MinTriEdeLen = m_TriangleEdgeFactor * m_TriangleEdgeFactor * m_UnitSquareDist;

	Eigen::Matrix3d CurR;
	Eigen::Vector3d CurT;
	unsigned BestCongruentCount = 0;
	unsigned CurCongruentCount = 0;
	unsigned EarlyReturnThreshold = m_EarlyReturnRatio * m_LoopSamplePointSet->getPointSet().size();
	unsigned AcceptThreshold = m_AcceptMatchRatio * m_LoopSamplePointSet->getPointSet().size();
	unsigned int LastIdx = vInitPairedPoints.size() - 1;
	for (unsigned i=0; i<vInitPairedPoints.size() - 2; ++i)
	{
		for (unsigned k=i+1; k<vInitPairedPoints.size() - 1; ++k)
		{
			double LoopDistAB = comSquaredDist(m_LoopPointCloud.getPosPointer(), vInitPairedPoints[i].first, m_LoopPointCloud.getPosPointer(), vInitPairedPoints[k].first, 3);
			if (LoopDistAB < MinTriEdeLen)
			{
				continue;
			}

			CurCongruentCount = __comOptimalRT(vInitPairedPoints, i, k, LastIdx, CurR, CurT);
			if (CurCongruentCount > BestCongruentCount)
			{
				voR = CurR;
				voT = CurT;
				BestCongruentCount = CurCongruentCount;

				if (BestCongruentCount > EarlyReturnThreshold)
				{
//#ifdef _DEBUG
					double FitRatio = computeCurrentFitRatio(vInitPairedPoints, voR, voT, m_LoopPointCloud, m_MatchPointCloud, (m_CongruentFactor) * m_UnitSquareDist);
					exportCurrentFitInfo(BestCongruentCount, FitRatio);
//#endif

					return true;
				}
			}
		}
	}

//#ifdef _DEBUG
	double FitRatio = computeCurrentFitRatio(vInitPairedPoints, voR, voT, m_LoopPointCloud, m_MatchPointCloud, m_CongruentFactor * m_UnitSquareDist);
	exportCurrentFitInfo(BestCongruentCount, FitRatio);
//#endif

	return (BestCongruentCount > AcceptThreshold);
}