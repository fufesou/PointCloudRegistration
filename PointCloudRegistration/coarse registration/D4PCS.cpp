#include "D4PCS.h"
#include <boost/typeof/typeof.hpp>
#include <Eigen/Geometry>
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "ICPType.h"
#include "BaseSampler.h"
#include "KNNSearch.h"
#include "PointCloudSubset.h"
#include "RegUtilityFunctions.h"
#include "TransformationEstimationSVD2.h"

#ifdef _DEBUG
#include "TestUnitity.h"
#endif


hiveRegistration::CD4PCS::CD4PCS()
{
	__parseConfig();
}

hiveRegistration::CD4PCS::~CD4PCS()
{

}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CD4PCS::coarseFit(const hivePointCloud::CPointCloud& vSrcPntCld, const hivePointCloud::CPointCloud& vTgtPntCld, Eigen::Matrix3d& voRot, Eigen::Vector3d& voTra)
{
	__init(vSrcPntCld, vTgtPntCld);

#ifdef _DEBUG
	export2Ply("TestData\\output\\CorPointPairs\\CoarseReg\\SubsetSrc.ply", m_SrcPntSubset->getPointSet());
	export2Ply("TestData\\output\\CorPointPairs\\CoarseReg\\Subsettgt.ply", m_TgtPntSubset->getPointSet());
#endif

	bool IsEdgeLenOK = false;

	for (int i=0; i<m_Param.NumLoop; ++i)
	{
		if (__selectCoplanarBase() && __computeR1R2())
		{
			__selectFeetPnts();

#ifdef _DEBUG
			char FileName[255];
			VecColVector3 BasePoints(m_FourPnts, m_FourPnts + 4);
			sprintf(FileName, "TestData\\output\\CorPointPairs\\CoarseReg\\BasePoints_%d.ply", i);
			export2Ply(FileName, BasePoints);
			VecColVector3 FeetPointsSet;
			FeetPointsSet.insert(FeetPointsSet.end(), m_FeetPnts[0].begin(), m_FeetPnts[0].end());
			FeetPointsSet.insert(FeetPointsSet.end(), m_FeetPnts[1].begin(), m_FeetPnts[1].end());
			FeetPointsSet.insert(FeetPointsSet.end(), m_FeetPnts[2].begin(), m_FeetPnts[2].end());
			FeetPointsSet.insert(FeetPointsSet.end(), m_FeetPnts[3].begin(), m_FeetPnts[3].end());
			sprintf(FileName, "TestData\\output\\CorPointPairs\\CoarseReg\\FeetPoints_%d.ply", i);
			export2Ply(FileName, FeetPointsSet);
#endif

			if (__findCongruent())
			{
				break;
			}
		}
	}
	if (m_CandSet.empty()) return false;

	int BestScore = -1;
	unsigned int BestIdx = 0;
	for (unsigned int i=0; i<m_CandSet.size(); ++i)
	{
		__testAlignment(m_CandSet[i]);
		if (m_CandSet[i].Score > BestScore)
		{
			BestScore = m_CandSet[i].Score;
			BestIdx = i;
		}
	}

	voRot = m_CandSet[BestIdx].R;
	voTra = m_CandSet[BestIdx].T;

	return true;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__init(const hivePointCloud::CPointCloud& vSrcPntCld, const hivePointCloud::CPointCloud& vTgtPntCld)
{
	m_pSrcPntCld = &vSrcPntCld;
	m_pTgtPntCld = &vTgtPntCld;

	const double* pSrcPos = vSrcPntCld.getPosPointer();
	const double* pSrcNorm = vSrcPntCld.getNormalPointer();
	for (unsigned int i=0; i<vSrcPntCld.getNumPoints(); ++i)
	{
		m_SrcPntSet.push_back(ColVector3(pSrcPos[i*3], pSrcPos[i*3 + 1], pSrcPos[i*3 + 2]));
		m_SrcNormalSet.push_back(ColVector3(pSrcNorm[i*3], pSrcNorm[i*3 + 1], pSrcNorm[i*3 + 2]));
	}
	const double* pTgtPos = vTgtPntCld.getPosPointer();
	const double* pTgtNorm = vTgtPntCld.getNormalPointer();
	for (unsigned int i=0; i<vTgtPntCld.getNumPoints(); ++i)
	{
		m_TgtPntSet.push_back(ColVector3(pTgtPos[i*3], pTgtPos[i*3 + 1], pTgtPos[i*3 + 2]));
		m_TgtNormalSet.push_back(ColVector3(pTgtNorm[i*3], pTgtNorm[i*3 + 1], pTgtNorm[i*3 + 2]));
	}
	
	Eigen::Matrix<double, 3, 2> PntCldRegion;
	computeRegion<3>(m_pSrcPntCld->getPosPointer(), m_pSrcPntCld->getNumPoints(), PntCldRegion);
	Eigen::Vector3d SideLen = PntCldRegion.col(1) - PntCldRegion.col(0);
	double MaxSideLen = (SideLen(0, 0) > SideLen(1, 0)) ?
		(SideLen(0, 0) > SideLen(2, 0) ? SideLen(0, 0) : SideLen(2, 0)) : 
		(SideLen(1, 0) > SideLen(2, 0) ? SideLen(1, 0) : SideLen(2, 0));
	m_FourPntsSide = MaxSideLen * m_Param.OverlapRatio;

	__doSample();
	__constructKNN();
	__comUnitDist();
	__computeEdgeLen(m_FourPntsSide*1.4, m_FourPntsSide*1.4);

	m_Param.ApproximationLevel *= m_SampleUnitDist;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__doSample(void)
{
	CBaseSampler* pSampler = dynamic_cast<CBaseSampler*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(m_Param.SampleStrID));
	m_SrcPntSubset = pSampler->samplePointCloud(*m_pSrcPntCld);
	m_TgtPntSubset = pSampler->samplePointCloud(*m_pTgtPntCld);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__parseConfig()
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyApproximationLever(), m_Param.ApproximationLevel);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyFeetSize(), m_Param.FeetSize);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyOverlapRatio(), m_Param.OverlapRatio);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySampleStrID(), m_Param.SampleStrID);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyScoreAln(), m_Param.ScoreAln);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeyScoreFeet(), m_Param.ScoreFeet);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
double hiveRegistration::CD4PCS::__comUnitSquareDist(const VecColVector3& vPntSet, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN) const
{
	_ASSERT(vKNN.get());

	double Avg = 0.0;
	unsigned int NumSample = 100;

	std::vector<unsigned int> Neibs;
	for (unsigned int i=0; i<NumSample; ++i)
	{
		int TmpIdx = rand()/(float)RAND_MAX * vPntSet.size() - 1;
		vKNN->executeKNN(vPntSet[TmpIdx].data(), Neibs);
		Avg += (vPntSet[Neibs[0]] - vPntSet[Neibs[1]]).squaredNorm();
	}
	Avg /= NumSample;

	return Avg;
}

//*********************************************************************************
//FUNCTION:
double hiveRegistration::CD4PCS::__comUnitSquareDist(const double* vPos, unsigned int vNumPnts, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN)
{
	_ASSERT(vKNN.get());
	double Avg = 0.0;
	unsigned int NumSample = 100;

	std::vector<unsigned int> Neibs;
	for (unsigned int i=0; i<NumSample; ++i)
	{
		unsigned int TmpIdx = rand()/(float)RAND_MAX * vNumPnts - 1;
		vKNN->executeKNN(vPos + i*3, Neibs);
		Avg += comSquaredDist(vPos, i, vPos, Neibs[1]);
	}
	Avg /= NumSample;

	return Avg;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CD4PCS::__selectCoplanarBase(void)
{
	double Delta = m_FourPntsSide * 0.1;
	unsigned int IdxPnt[4];
	const VecColVector3& SrcSubsetPnts = m_SrcPntSubset->getPointSet();
	IdxPnt[0] = ICP_RAND(0.0, 1.0) * (SrcSubsetPnts.size() - 2);
	m_FourPnts[0] = SrcSubsetPnts[IdxPnt[0]];

	unsigned int i=0;
	for (; i<SrcSubsetPnts.size(); ++i)
	{
		double TmpDist = (SrcSubsetPnts[i] - m_FourPnts[0]).norm();
		if ((TmpDist < m_FourPntsSide + Delta) && (TmpDist > m_FourPntsSide - Delta))
		{
			m_FourPnts[1] = SrcSubsetPnts[i];
			break;
		}
	}
	if (SrcSubsetPnts.size() == i) return false;

	unsigned int BestIdx = -1;
	double BestCosAgl = std::numeric_limits<double>::max();
	for (i=0; i<SrcSubsetPnts.size(); ++i)
	{
		unsigned int TmpIdx = ICP_RAND(0.0, 1.0) * (SrcSubsetPnts.size() - 1);
		double TmpDist = (SrcSubsetPnts[i] - m_FourPnts[1]).norm();
		if ((TmpDist < m_FourPntsSide + Delta) && (TmpDist > m_FourPntsSide - Delta))
		{
			ColVector3 TmpVec01 = m_FourPnts[0] - m_FourPnts[1];
			ColVector3 TmpVec12 = SrcSubsetPnts[i] - m_FourPnts[1];
			TmpVec01.normalize();
			TmpVec12.normalize();
			double TmpCorAgl = fabs(TmpVec12.dot(TmpVec01));

			if (TmpCorAgl < BestCosAgl)
			{
				BestCosAgl = TmpCorAgl;
				BestIdx = i;
			}
		}
	}
	if (-1 == BestIdx) return false;
	m_FourPnts[2] = SrcSubsetPnts[BestIdx];

	ColVector3 Vec01 = m_FourPnts[0] - m_FourPnts[1];
	ColVector3 Vec21 = m_FourPnts[2] - m_FourPnts[1];
	ColVector3 Tmp4thPnt = m_FourPnts[1] + Vec01 + Vec21;

	std::vector<unsigned> Neibs;
	m_SrcSubsetKNN->executeKNN(Tmp4thPnt.data(), Neibs);
	BestIdx = -1;
	BestCosAgl = std::numeric_limits<double>::max();
	double DistThreshold = Delta * 4.0;
	Vec01.normalize();
	Vec21.normalize();
	ColVector3 Normal2Plane = Vec01.cross(Vec21);
	for (unsigned int i=0; i<Neibs.size(); ++i)
	{
		double TmpDist = (Tmp4thPnt - SrcSubsetPnts[Neibs[i]]).norm();
		if (TmpDist < DistThreshold)
		{
			double TmpCosAgl = fabs((SrcSubsetPnts[Neibs[i]] - m_FourPnts[1]).dot(Normal2Plane));
			if (TmpCosAgl < BestCosAgl)
			{
				BestCosAgl = TmpCosAgl;
				BestIdx = Neibs[i];
			}
		}
	}
	if (-1 != BestIdx)
	{
		m_FourPnts[3] = SrcSubsetPnts[BestIdx];
		return true;
	}
	return false;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__comUnitDist(void)
{
	m_PntCldUnitDist = __comUnitSquareDist(m_pSrcPntCld->getPosPointer(), m_pSrcPntCld->getNumPoints(), m_SrcPntCldKNN);
	m_SampleUnitDist = __comUnitSquareDist(m_SrcPntSubset->getPointSet(), m_SrcSubsetKNN);

	m_PntCldUnitDist = std::sqrt(m_PntCldUnitDist);
	m_SampleUnitDist = std::sqrt(m_SampleUnitDist);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__constructKNN(void)
{
	m_SrcPntCldKNN.reset(new hiveCommon::CKNNSearch);
	m_SrcPntCldKNN->initKNNSearch(m_pSrcPntCld->getPosPointer(), m_pSrcPntCld->getNumPoints(), 3, 20);

	m_TgtPntCldKNN.reset(new hiveCommon::CKNNSearch);
	m_TgtPntCldKNN->initKNNSearch(m_pTgtPntCld->getPosPointer(), m_pTgtPntCld->getNumPoints(), 3, 20);

	m_SrcSubsetKNN.reset(new hiveCommon::CKNNSearch);
	Eigen::Matrix<double, 3, Eigen::Dynamic> TmpSrcSubset;
	const VecColVector3& SrcSubset = m_SrcPntSubset->getPointSet();
	TmpSrcSubset.resize(3, SrcSubset.size());
	for (unsigned int i=0; i<SrcSubset.size(); ++i)
	{
		TmpSrcSubset.col(i) = SrcSubset[i];
	}
	m_SrcSubsetKNN->initKNNSearch(TmpSrcSubset.data(), TmpSrcSubset.cols(), 3, m_Param.FeetSize);

	m_TgtSubsetKNN.reset(new hiveCommon::CKNNSearch);
	Eigen::Matrix<double, 3, Eigen::Dynamic> TmpTgtSubset;
	const VecColVector3& TgtSubset = m_TgtPntSubset->getPointSet();
	TmpTgtSubset.resize(3, TgtSubset.size());
	for (unsigned int i=0; i<TgtSubset.size(); ++i)
	{
		TmpTgtSubset.col(i) = TgtSubset[i];
	}
	m_SrcSubsetKNN->initKNNSearch(TmpTgtSubset.data(), TmpTgtSubset.cols(), 3, 8);
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CD4PCS::__computeR1R2()
{
	ColVector3 Pnt1, Pnt2;
	ColVector3 LineAVert[2] = { m_FourPnts[0], m_FourPnts[2] };
	ColVector3 LineBVert[2] = { m_FourPnts[1], m_FourPnts[3] };
	__intersectLineLine(LineAVert, LineBVert, Pnt1, Pnt2);
	double R1 = (Pnt1 - m_FourPnts[0]).dot(m_FourPnts[2] - m_FourPnts[0]) / (m_FourPnts[2] - m_FourPnts[0]).squaredNorm();
	double R2 = (Pnt1 - m_FourPnts[1]).dot(m_FourPnts[3] - m_FourPnts[1]) / (m_FourPnts[3] - m_FourPnts[1]).squaredNorm();
	//double r2 = (Pnt2 - m_FourPnts[1]).dot(m_FourPnts[3] - m_FourPnts[1]) / (m_FourPnts[3] - m_FourPnts[1]).squaredNorm();
	m_CurR1R2[0] = R1;
	m_CurR1R2[1] = R2;

	return ((m_FourPnts[0] + (m_FourPnts[2] - m_FourPnts[0]*R1) - (m_FourPnts[1] + (m_FourPnts[3] - m_FourPnts[2])*R2)).norm() > m_Param.ApproximationLevel);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__selectFeetPnts(void)
{
	double Delta = m_FourPntsSide * 0.5;
	std::vector<unsigned int> Neibs;
	const VecColVector3& SrcSubsetPnts = m_SrcPntSubset->getPointSet();
	const VecColVector3& SrcSubsetNormals = m_SrcPntSubset->getNormalSet();
	for (int i=0; i<4; ++i)
	{
		m_SrcSubsetKNN->executeKNN(m_FourPnts[i].data(), Neibs);
		for (std::vector<unsigned int>::const_iterator CItr=Neibs.begin(); CItr!=Neibs.end(); ++CItr)
		{
			if ((m_FourPnts[i] - SrcSubsetPnts[*CItr]).norm() < Delta)
			{
				m_FeetPnts[i].push_back(SrcSubsetPnts[*CItr]);
				m_FeetPntsNormal[i].push_back(SrcSubsetNormals[*CItr]);
			}
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__computeEdgeLen(double vD1, double vD2)
{
	double Delta = m_FourPntsSide * 0.5;
	const VecColVector3& TgtSubsetPnts = m_TgtPntSubset->getPointSet();
	const unsigned int SizeTgtSubset = TgtSubsetPnts.size();
	for (unsigned int i=0; i<SizeTgtSubset; ++i)
	{
		for (unsigned int k=i; k<SizeTgtSubset; ++k)
		{
			double TmpDist = (TgtSubsetPnts[i] - TgtSubsetPnts[k]).norm();
			if ((TmpDist < vD1 + Delta) && (TmpDist > vD1 - Delta))
			{
				m_VecD1Len.push_back(boost::make_tuple(i, k, TmpDist));
				m_VecD1Len.push_back(boost::make_tuple(k, i, TmpDist));
			}

			//if ((TmpDist < vD2 + Delta) && (TmpDist > vD2 - Delta))
			//{
			//	m_VecD2ABLen.push_back(i, k, TmpDist);
			//	m_VecD2BALen.push_back(k, i, TmpDist);
			//}
		}
	}

	std::sort(m_VecD1Len.begin(), m_VecD1Len.end(), CmpLenDist());
	//std::sort(m_VecD2Len.begin(), m_VecD2Len.end(), cmpLenDist);
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CD4PCS::__findCongruent()
{
	double D1 = (m_FourPnts[2] - m_FourPnts[0]).norm();
	double D2 = (m_FourPnts[3] - m_FourPnts[1]).norm();

	BOOST_AUTO(BItr1, std::lower_bound(m_VecD1Len.begin(), m_VecD1Len.end(), D1 - m_Param.ApproximationLevel*2.0, CmpLenDist()));
	BOOST_AUTO(EItr1, std::lower_bound(m_VecD1Len.begin(), m_VecD1Len.end(), D1 + m_Param.ApproximationLevel*2.0, CmpLenDist()));
	BOOST_AUTO(BItr2, std::lower_bound(m_VecD1Len.begin(), m_VecD1Len.end(), D2 - m_Param.ApproximationLevel*2.0, CmpLenDist()));
	BOOST_AUTO(EItr2, std::lower_bound(m_VecD1Len.begin(), m_VecD1Len.end(), D2 + m_Param.ApproximationLevel*2.0, CmpLenDist()));

	if (BItr1 == m_VecD1Len.end()) return false;
	if (BItr2 == m_VecD1Len.end()) return false;

	const VecColVector3& TgtSubsetPnts = m_TgtPntSubset->getPointSet();

	Eigen::Matrix<double, 3, Eigen::Dynamic> R1Pnts;
	R1Pnts.resize(3, EItr1 - BItr1);
	std::vector<boost::tuple<unsigned int, unsigned int, double> >::iterator Itr = BItr1;
	for (int i=0; i<R1Pnts.cols(); ++i)
	{
		R1Pnts.col(i) = TgtSubsetPnts[Itr->get<0>()] + (TgtSubsetPnts[Itr->get<1>()] - TgtSubsetPnts[Itr->get<0>()]) * m_CurR1R2[0];
		++Itr;
	}

	hiveCommon::CKNNSearch R1KNN;
	R1KNN.initKNNSearch(R1Pnts.data(), R1Pnts.cols(), 3, (R1Pnts.cols() < 20) ? R1Pnts.cols() : 20);		// ATTENTION
//	R1KNN.initKNNSearch(R1Pnts.data(), R1Pnts.cols(), 3, R1Pnts.cols());		// ATTENTION

	double Delta = m_Param.ApproximationLevel * 0.1;
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
	std::vector<boost::tuple<unsigned int, unsigned int, double> >::const_iterator CItr;
	unsigned int TmpCount = EItr2 - BItr2;
	std::vector<unsigned int> Neibs;
	for (Itr = BItr2; Itr!=EItr2; ++Itr)
	{
		ColVector3 TmpR2Pnt = TgtSubsetPnts[Itr->get<0>()] + (TgtSubsetPnts[Itr->get<1>()] - TgtSubsetPnts[Itr->get<0>()]) * m_CurR1R2[1];
		R1KNN.executeKNN(TmpR2Pnt.data(), Neibs);

		for (std::vector<unsigned>::const_iterator NeibCItr=Neibs.begin(); NeibCItr!=Neibs.end(); ++NeibCItr)
		{
			unsigned int CurR1Idx = *NeibCItr;
			double DistNeibR2 = (R1Pnts.col(CurR1Idx) - TmpR2Pnt).norm();
			if ( DistNeibR2 < Delta )
			{
				ColVector3 FourPnts[4];
				CItr = BItr1 + CurR1Idx;
				FourPnts[0] = TgtSubsetPnts[CItr->get<0>()];
				FourPnts[2] = TgtSubsetPnts[CItr->get<1>()];

				FourPnts[1] = TgtSubsetPnts[Itr->get<0>()];
				FourPnts[3] = TgtSubsetPnts[Itr->get<1>()];

#ifdef _DEBUG
				VecColVector3 CurFourPnts(FourPnts, FourPnts + 4);
				export2Ply("TestData\\output\\CorPointPairs\\CoarseReg\\CurFourPnts.ply", CurFourPnts);
#endif

				double TmpErr;
				if (__isTransformCongruent(FourPnts, R, T, TmpErr))
				{
					Candidate TmpCand;
					TmpCand.FourPnts[0] = FourPnts[0];
					TmpCand.FourPnts[1] = FourPnts[1];
					TmpCand.FourPnts[2] = FourPnts[2];
					TmpCand.FourPnts[3] = FourPnts[3];
					TmpCand.R = R;
					TmpCand.T = T;

					__evaluateAlignment(TmpCand);

					if (TmpCand.Score > m_Param.ScoreFeet)
					{
						__testAlignment(TmpCand);
						if (TmpCand.Score > m_Param.ScoreAln)
						{
							return true;
						}
					}

				}
			}
		}
	}

	return false;
}

//*********************************************************************************
//FUNCTION:
bool hiveRegistration::CD4PCS::__isTransformCongruent(const ColVector3 vFourPnts[4], Eigen::Matrix3d& voR, Eigen::Vector3d& voT, double& voErr)
{
	VecColVector3 FixPnts, MovePnts;
	for (int i=0; i<4; ++i)
	{
		FixPnts.push_back(m_FourPnts[i]);
		MovePnts.push_back(vFourPnts[i]);
	}
	ColVector3 TmpFixPnt, TmpMovPnt, VecFix10, VecFix20, VecMov10, VecMov20;
	VecFix20 = m_FourPnts[2] - m_FourPnts[0]; VecFix20.normalize();
	VecFix10 = m_FourPnts[1] - m_FourPnts[0]; VecFix10.normalize();
	VecMov20 = vFourPnts[2] - vFourPnts[0]; VecMov20.normalize();
	VecMov10 = vFourPnts[1] - vFourPnts[0]; VecMov10.normalize();
	TmpFixPnt = VecFix20.cross(VecFix10) * (m_FourPnts[2] - m_FourPnts[0]).norm();
	TmpMovPnt = VecMov20.cross(VecMov10) * (vFourPnts[2] - vFourPnts[0]).norm();
	FixPnts.push_back(TmpFixPnt);
	MovePnts.push_back(TmpMovPnt);

	CTransformationEstimationSVD2* pTESVD = dynamic_cast<CTransformationEstimationSVD2*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(CTransformationEstimationSVD2::getClassSig()));
	pTESVD->computeRotationAndTranslation(MovePnts, FixPnts, voR, voT);

	double Err = 0.0;
	for (int i=0; i<4; ++i)
	{
		Err += (voR * MovePnts[i] + voT - FixPnts[i]).squaredNorm();
	}

	voErr = std::sqrt(Err);

	return Err < m_Param.ApproximationLevel * m_Param.ApproximationLevel * 4.0;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__testAlignment(Candidate& vioCand)
{
	int NumDataClose = 0;
	const VecColVector3& SrcSubsetPnt = m_SrcPntSubset->getPointSet();
	const VecColVector3& SrcSubsetNorm = m_SrcPntSubset->getNormalSet();
	for (unsigned int i=0; i<m_SrcPntSubset->getPointSet().size(); ++i)
	{
		NumDataClose += __evaluateSample(vioCand, SrcSubsetPnt[i], SrcSubsetNorm[i], 
			m_TgtPntCldKNN, m_TgtPntSet, m_TgtNormalSet, m_Param.ApproximationLevel, 0.6);
	}
	vioCand.Score = NumDataClose;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CD4PCS::__evaluateAlignment(Candidate& vioCand)
{
	int NumDataClose = 0;
	for (int i=0; i<4; ++i)
	{
		for (unsigned int k=0; k<m_FeetPnts[i].size(); ++k)
		{
			NumDataClose += __evaluateSample(vioCand, m_FeetPnts[i][k], m_FeetPntsNormal[i][k], 
				m_TgtPntCldKNN, m_TgtPntSet, m_TgtNormalSet, m_Param.ApproximationLevel, 0.9);
		}
	}
	vioCand.Score = NumDataClose;
}

//*********************************************************************************
//FUNCTION:
int hiveRegistration::CD4PCS::__evaluateSample(const Candidate& vCand, const ColVector3& vPnt, const ColVector3& vNormal, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, const VecColVector3& vPntSet, const VecColVector3& vNormSet, double vDistThr, double vCosAgl)
{
	const ColVector3 TmpPnt = vCand.R * vPnt + vCand.T;
	const ColVector3 TmpNorm = vCand.R * vNormal;
	std::vector<unsigned int> Neibs;
	vKNN->executeKNN(TmpPnt.data(), Neibs);
	double SquareDist = comSquaredDist(TmpPnt.data(), 0, vPntSet[Neibs[0]].data(), 0, 3);
	if (SquareDist < vDistThr)
	{
		if (TmpNorm.dot(vNormSet[Neibs[0]]) < vCosAgl)
		{
			return 1;
		}
		else
		{
			return -1;	// ATTENTION
		}
	}

	return 0;		// ATTENTION
}