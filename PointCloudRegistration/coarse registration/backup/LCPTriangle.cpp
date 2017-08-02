#include "LCPTriangle.h"
#include <algorithm>
#include <Eigen/LU>
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloud.h"
#include "TestUnitity.h"
#include "TransformationEstimationSVD2.h"

#ifdef _DEBUG
#include <fstream>

namespace
{
	static unsigned gs_BestFitPairs[6];

	struct CmpPairedPntGroup
	{
		bool operator()(const std::pair<unsigned, std::vector<unsigned> >& vL, const std::pair<unsigned, std::vector<unsigned> >& vR)
		{
			return vL.second.size() < vR.second.size();
		}
	};
}
#endif


hiveRegistration::CLCPTriangle::CLCPTriangle(const hivePointCloud::CPointCloud& vLoopPointCloud, const hivePointCloud::CPointCloud& vMatchPointCloud, const std::vector<unsigned>& vLoopSamplePnts, const std::vector<unsigned>& vMatchSamplePnts)
: m_LoopPointCloud(vLoopPointCloud)
, m_MatchPointCloud(vMatchPointCloud)
, m_LoopPnts(vLoopSamplePnts)
, m_MatchPnts(vMatchSamplePnts)
{
	__parseParams();

	m_LoopKNN.initKNNSearch(m_LoopPointCloud.getPosPointer(), m_LoopPointCloud.getNumPoints(), 3, 1);
	m_MatchKNN.initKNNSearch(m_MatchPointCloud.getPosPointer(), m_MatchPointCloud.getNumPoints(), 3, 1);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::comRotationAndTranslation(std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPntGroup, double vUnitSquareDist, Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
#ifdef _DEBUG
	std::ofstream BestFitPntPairs("TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\BestFitPairs.txt");
#endif

	std::sort(vInitPairedPntGroup.begin(), vInitPairedPntGroup.end(), CmpPairedPntGroup());

	m_UnitSquareDist = vUnitSquareDist;

	Eigen::Matrix3d CurR;
	Eigen::Vector3d CurT;
	unsigned PreCongruentCount = 0;
	unsigned CurCongruentCount = 0;
	unsigned CurgruentCountThreshold = m_MatchRatio * m_LoopPnts.size();

	double MinTriEdgeLen = m_TriangleEdgeFactor * m_UnitSquareDist;

	std::pair<unsigned, std::vector<unsigned> > TriPntGroup[3];
	for (unsigned i=0; i<vInitPairedPntGroup.size() - 2; ++i)
	{
		for (unsigned k=i+1; k<vInitPairedPntGroup.size() - 1; ++k)
		{
			std::pair<unsigned, std::vector<unsigned> > TmpPntPair[2];
			TmpPntPair[0].first = vInitPairedPntGroup[i].first;
			TmpPntPair[1].first = vInitPairedPntGroup[k].first;
			__findEdgeVertexSet(vInitPairedPntGroup[i], vInitPairedPntGroup[k], TmpPntPair);

			for (unsigned j=k+1; j<vInitPairedPntGroup.size(); ++j)
			{
				__findEdgeVertexSet(TmpPntPair, vInitPairedPntGroup[j], TriPntGroup);

				if (!TriPntGroup[0].second.empty())
				{
					TriPntGroup[0].first = vInitPairedPntGroup[i].first; 
					TriPntGroup[1].first = vInitPairedPntGroup[k].first; 
					TriPntGroup[2].first = vInitPairedPntGroup[j].first;

					CurCongruentCount = __comOptimalRT(TriPntGroup, CurR, CurT);
					if (CurCongruentCount > PreCongruentCount)
					{
						voR = CurR;
						voT = CurT;

						PreCongruentCount = CurCongruentCount;
					}
					if (CurCongruentCount > CurgruentCountThreshold)
					{
#ifdef _DEBUG
						BestFitPntPairs << gs_BestFitPairs[0] << " " << gs_BestFitPairs[3] << std::endl;
						BestFitPntPairs << gs_BestFitPairs[1] << " " << gs_BestFitPairs[4] << std::endl;
						BestFitPntPairs << gs_BestFitPairs[2] << " " << gs_BestFitPairs[5] << std::endl;
						BestFitPntPairs << PreCongruentCount << std::endl;
						BestFitPntPairs.close();
#endif
						return;
					}
				}
			}
		}
	}

#ifdef _DEBUG
	BestFitPntPairs << gs_BestFitPairs[0] << " " << gs_BestFitPairs[3] << std::endl;
	BestFitPntPairs << gs_BestFitPairs[1] << " " << gs_BestFitPairs[4] << std::endl;
	BestFitPntPairs << gs_BestFitPairs[2] << " " << gs_BestFitPairs[5] << std::endl;
	BestFitPntPairs << PreCongruentCount << std::endl;
	BestFitPntPairs.close();
#endif

}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__parseParams(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyCongruentFactor(), m_CongruentFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyMatchRatio(), m_MatchRatio);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySimTriFactor(), m_SimilarTriangleFactor);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyTriangleEdgeFactor(), m_TriangleEdgeFactor);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__comRTWith3PntPairs(const std::pair<unsigned, unsigned> vPntPairs[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT) const
{
#ifdef _DEBUG
	std::ofstream OutFileTri("TestData\\output\\CorPointPairs\\CoarseReg_Mid\\all\\BestFitPairs.txt", std::ios::app );
	OutFileTri << vPntPairs[0].first << " " << vPntPairs[0].second << std::endl;
	OutFileTri << vPntPairs[1].first << " " << vPntPairs[1].second << std::endl;
	OutFileTri << vPntPairs[2].first << " " << vPntPairs[2].second << std::endl;
	OutFileTri << std::endl;
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
//	_ASSERT(SquareDist < m_UnitSquareDist * 10.0);
//	SquareDist = (voR * LoopPntSet[1] + voT - MatchPntSet[1]).squaredNorm();
//	_ASSERT(SquareDist < m_UnitSquareDist * 10.0);
//	SquareDist = (voR * LoopPntSet[2] + voT - MatchPntSet[2]).squaredNorm();
//	_ASSERT(SquareDist < m_UnitSquareDist * 10.0);
//#endif
}

//*********************************************************************************
//FUNCTION:
unsigned hiveRegistration::CLCPTriangle::__findNumPntsInTolerance(const Eigen::Matrix3d& vR, const Eigen::Vector3d& vT)
{
	unsigned CongruentCount = 0;
	double CongruentDist = m_CongruentFactor * m_UnitSquareDist;
	const double* pLoopPos = m_LoopPointCloud.getPosPointer();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();
	std::vector<unsigned> Neibs;
	Eigen::Vector3d NewLoopPnt;
	for (std::vector<unsigned>::const_iterator citr=m_LoopPnts.begin(); citr!=m_LoopPnts.end(); ++citr)
	{
		NewLoopPnt = vR * Eigen::Vector3d(pLoopPos[(*citr) * 3 + 0], pLoopPos[(*citr) * 3 + 1], pLoopPos[(*citr) * 3 + 2]) + vT;
		m_MatchKNN.executeKNN(NewLoopPnt.data(), Neibs);
		double SquareDist = comSquaredDist(NewLoopPnt.data(), 0, const_cast<double*>(pMatchPos), Neibs.front(), 3);
		CongruentCount += (SquareDist < CongruentDist);
	}

	return CongruentCount;
}

//*********************************************************************************
//FUNCTION:
unsigned hiveRegistration::CLCPTriangle::__comOptimalRT(const std::pair<unsigned, std::vector<unsigned> > vTriPntGroup[3], Eigen::Matrix3d& voR, Eigen::Vector3d& voT)
{
	unsigned PreCongruentCount = 0;
	unsigned CurCongruentCount = 0;
	unsigned CurgruentCountThreshold = m_MatchRatio * m_LoopPnts.size();

	Eigen::Matrix3d CurR;
	Eigen::Vector3d CurT;

	std::pair<unsigned, unsigned> PntPairs[3];
	PntPairs[0].first = vTriPntGroup[0].first;
	PntPairs[1].first = vTriPntGroup[1].first;
	PntPairs[2].first = vTriPntGroup[2].first;

	for (std::vector<unsigned>::const_iterator CItrA=vTriPntGroup[0].second.begin(); CItrA!=vTriPntGroup[0].second.end(); ++CItrA)
	{
		for (std::vector<unsigned>::const_iterator CItrB=vTriPntGroup[0].second.begin(); CItrB!=vTriPntGroup[0].second.end(); ++CItrB)
		{
			for (std::vector<unsigned>::const_iterator CItrC=vTriPntGroup[0].second.begin(); CItrC!=vTriPntGroup[0].second.end(); ++CItrC)
			{
				PntPairs[0].second = *CItrA;
				PntPairs[1].second = *CItrB;
				PntPairs[2].second = *CItrC;

				__comRTWith3PntPairs(PntPairs, CurR, CurT);
				CurCongruentCount = __findNumPntsInTolerance(CurR, CurT);

				if (CurCongruentCount > PreCongruentCount)
				{
#ifdef _DEBUG
					gs_BestFitPairs[0] = PntPairs[0].first; gs_BestFitPairs[3] = PntPairs[0].second;
					gs_BestFitPairs[1] = PntPairs[1].first; gs_BestFitPairs[4] = PntPairs[1].second;
					gs_BestFitPairs[2] = PntPairs[2].first; gs_BestFitPairs[5] = PntPairs[2].second;
#endif

					voR = CurR;
					voT = CurT;
					PreCongruentCount = CurCongruentCount;
				}
				if (CurCongruentCount > CurgruentCountThreshold)
				{
					return CurCongruentCount;
				}
			}
		}
	}

	return PreCongruentCount;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__findEdgeVertexSet(const std::pair<unsigned, std::vector<unsigned> > vPrePairedAB[2], const std::pair<unsigned, std::vector<unsigned> >& vPntGroupC, std::pair<unsigned, std::vector<unsigned> > voTriPntGroupPair[3]) const
{
	if (vPrePairedAB[0].second.empty())
	{
		return;
	}

	voTriPntGroupPair[0].second.clear();
	voTriPntGroupPair[1].second.clear();
	voTriPntGroupPair[2].second.clear();

	const double* pLoopPos = m_LoopPointCloud.getPosPointer();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();

	double TriDistTol = m_SimilarTriangleFactor * m_UnitSquareDist;
	double MinTriEdgeLen = m_TriangleEdgeFactor * m_UnitSquareDist;

	double LoopDistAC = comSquaredDist(pLoopPos, vPrePairedAB[0].first, pLoopPos, vPntGroupC.first, 3);
	double LoopDistBC = comSquaredDist(pLoopPos, vPrePairedAB[1].first, pLoopPos, vPntGroupC.first, 3);
	if (LoopDistAC < MinTriEdgeLen)
	{
		return;
	}

	for (std::vector<unsigned>::const_iterator CItrA=vPrePairedAB[0].second.begin(), CItrB=vPrePairedAB[1].second.begin(); CItrA!=vPrePairedAB[0].second.end(); ++CItrA, ++CItrB)
	{
		for (std::vector<unsigned>::const_iterator CItrC=vPntGroupC.second.begin(); CItrC!=vPntGroupC.second.end(); ++CItrC)
		{
			double MatchDistAC = comSquaredDist(pMatchPos, *CItrA, pMatchPos, *CItrC, 3);
			if (fabs(MatchDistAC - LoopDistAC) < TriDistTol)
			{
				double MatchDistBC = comSquaredDist(pMatchPos, *CItrB, pMatchPos, *CItrC, 3);
				if (fabs(MatchDistBC - LoopDistAC) < TriDistTol)
				{
					voTriPntGroupPair[0].second.push_back(*CItrA);
					voTriPntGroupPair[1].second.push_back(*CItrB);
					voTriPntGroupPair[2].second.push_back(*CItrC);
				}
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CLCPTriangle::__findEdgeVertexSet(const std::pair<unsigned, std::vector<unsigned> >& vPntGroupA, const std::pair<unsigned, std::vector<unsigned> >& vPntGroupB, std::pair<unsigned, std::vector<unsigned> > voPrePairedAB[2]) const
{
	voPrePairedAB[0].second.clear();
	voPrePairedAB[1].second.clear();

	const double* pLoopPos = m_LoopPointCloud.getPosPointer();
	const double* pMatchPos = m_MatchPointCloud.getPosPointer();

	double TriDistTol = m_SimilarTriangleFactor * m_UnitSquareDist;
	double MinTriEdgeLen = m_TriangleEdgeFactor * m_UnitSquareDist;

	double LoopDistAB = comSquaredDist(pLoopPos, vPntGroupA.first, pLoopPos, vPntGroupB.first, 3);
	if (LoopDistAB < MinTriEdgeLen)
	{
		return;
	}

	// FIXME: OpenMP have no effect here, because the sizes of the vectors are small

	for (std::vector<unsigned>::const_iterator CItrA=vPntGroupA.second.begin(); CItrA!=vPntGroupA.second.end(); ++CItrA)
	{
		for (std::vector<unsigned>::const_iterator CItrB=vPntGroupB.second.begin(); CItrB!=vPntGroupB.second.end(); ++CItrB)
		{
			double MatchDistAB = comSquaredDist(pMatchPos, *CItrA, pMatchPos, *CItrB, 3);
			if (fabs(MatchDistAB - LoopDistAB) < TriDistTol)
			{
				voPrePairedAB[0].second.push_back(*CItrA);
				voPrePairedAB[1].second.push_back(*CItrB);
			}
		}
	}
}
