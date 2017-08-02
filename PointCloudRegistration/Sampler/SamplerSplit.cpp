#include "SamplerSplit.h"
#include <vector>
#include "HiveCommonMicro.h"
#include "ICPMacros.h"
#include "ProductFactory.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "PointCloudSubset.h"
#include "RegUtilityFunctions.h"


hiveCommon::CProductFactory<hiveRegistration::CSamplerSplit> TheCreator(hiveRegistration::CSamplerSplit::getClassSig());

//*********************************************************************************
//FUNCTION:
hiveRegistration::CSamplerSplit::CSamplerSplit(void)
: m_UseCompactRegion(false)
{
	_letFactoryReleaseProduct();
	__parseConfig();
}

//*********************************************************************************
//FUNCTION:
hiveRegistration::CorrespondencePointSet hiveRegistration::CSamplerSplit::_doSampleV(const hivePointCloud::CPointCloud& vPointCloud)
{
	__init(vPointCloud);

	VecSplitNode SplitNodeSet;
	VecSplitNode NextSplitNodeSet;
	__constructRoot(SplitNodeSet);

#ifdef _DEBUG
	unsigned int NumLoop = 0;
#endif

	while (SplitNodeSet.size())
	{
		__doSample(SplitNodeSet);
		__split(SplitNodeSet, NextSplitNodeSet);
		SplitNodeSet.swap(NextSplitNodeSet);

#ifdef _DEBUG
		_ASSERT (++NumLoop < 1000);
#endif
	}

	return makeSharedPtr(new CIndexSubset(*m_pPointCloud, m_SampleIndexSet));
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__parseConfig(void)
{
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getKeySplitNodeMinNumPnt(), m_NodeMinNumPnt);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__init(const hivePointCloud::CPointCloud& vPointCloud)
{
	m_pPointCloud = &vPointCloud;
	m_SampleIndexSet.clear();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__constructRoot(VecSplitNode& voSplitNodes)
{
	voSplitNodes.clear();

#ifdef max
#undef max
#endif

	SplitNode RootNode;
	computeRegion<3>(m_pPointCloud->getPosPointer(), m_pPointCloud->getNumPoints(), RootNode.NodeRegion);
	for (unsigned int i=0; i<m_pPointCloud->getNumPoints(); ++i)
	{
		RootNode.ContainedPnts.push_back(i);
	}
	voSplitNodes.push_back(RootNode);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__split(const VecSplitNode& vNodes2Split, VecSplitNode& voSplitNodes)
{
	voSplitNodes.clear();

	const double* pPos = m_pPointCloud->getPosPointer();

	for (VecSplitNode::const_iterator NodeItr=vNodes2Split.begin(); NodeItr!=vNodes2Split.end(); ++NodeItr)
	{
		ColVector3 Cent = (NodeItr->NodeRegion.col(0) + NodeItr->NodeRegion.col(1)) * 0.5;
		SplitNode TmpNode[8];

		__init8SplitNode(NodeItr->NodeRegion, Cent, TmpNode);

		for (std::vector<unsigned>::const_iterator PntItr=NodeItr->ContainedPnts.begin(); PntItr!=NodeItr->ContainedPnts.end(); ++PntItr)
		{
			unsigned I = (pPos[(*PntItr)*3 + 0] > Cent(0, 0)) ? 1 : 0;
			unsigned J = (pPos[(*PntItr)*3 + 1] > Cent(1, 0)) ? 1 : 0;
			unsigned K = (pPos[(*PntItr)*3 + 2] > Cent(2, 0)) ? 1 : 0;
			TmpNode[K*4 + J*2 + I].ContainedPnts.push_back(*PntItr);
		}
		for (unsigned i=0; i<8; ++i)
		{
			if (!TmpNode[i].ContainedPnts.empty()) voSplitNodes.push_back(TmpNode[i]);
		}
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__doSample(VecSplitNode& vioSplitNodes)
{
	VecSplitNode TmpSplitNodeSet;

	for (VecSplitNode::iterator NodeItr=vioSplitNodes.begin(); NodeItr!=vioSplitNodes.end(); ++NodeItr)
	{
		if (NodeItr->ContainedPnts.size() < m_NodeMinNumPnt)
		{
			ColVector3 Centroid;
			if (m_UseCompactRegion)
			{
				__computeRegionAndCentroid(NodeItr->ContainedPnts, NodeItr->NodeRegion, Centroid);
			}
			else
			{
				__computeCentroid(NodeItr->ContainedPnts, Centroid);
			}
			unsigned SampleIndex = __findPointNearestCentroid(NodeItr->ContainedPnts, Centroid);
			m_SampleIndexSet.push_back(SampleIndex);
		}
		else
		{
			if (m_UseCompactRegion) __updateRegion(NodeItr->ContainedPnts, NodeItr->NodeRegion);
			TmpSplitNodeSet.push_back(*NodeItr);
		}
	}

	vioSplitNodes.swap(TmpSplitNodeSet);
}

//*********************************************************************************
//FUNCTION:
unsigned hiveRegistration::CSamplerSplit::__findPointNearestCentroid(const std::vector<unsigned>& vPointSet,const ColVector3& vCentroid) const
{
	const double* pPos = m_pPointCloud->getPosPointer();
	unsigned SampleIndex = 0xffffffff;
	double NearseSquareDist = std::numeric_limits<double>::max();

	for (std::vector<unsigned>::const_iterator PntItr=vPointSet.begin(); PntItr!=vPointSet.end(); ++PntItr)
	{
		double SquareDist = comSquaredDist(pPos, *PntItr, vCentroid.data(), unsigned(0), 3);
		if (SquareDist < NearseSquareDist)
		{
			NearseSquareDist = SquareDist;
			SampleIndex = *PntItr;
		}
	}

	return SampleIndex;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__computeRegionAndCentroid(const std::vector<unsigned>& vPointSet, RegionDim3& vioRegion, ColVector3& voCentroid) const
{
	const double* pPos = m_pPointCloud->getPosPointer();
	voCentroid.setZero();
	for (std::vector<unsigned>::const_iterator PntItr=vPointSet.begin(); PntItr!=vPointSet.end(); ++PntItr)
	{
		ColVector3 Pnt(pPos[(*PntItr)*3 + 0], pPos[(*PntItr)*3 + 1], pPos[(*PntItr)*3 + 2]);
		_HIVE_SIMPLE_IF(Pnt(0, 0) < vioRegion(0, 0), vioRegion(0, 0) = Pnt(0, 0));
		_HIVE_SIMPLE_IF(Pnt(1, 0) < vioRegion(1, 0), vioRegion(1, 0) = Pnt(1, 0));
		_HIVE_SIMPLE_IF(Pnt(2, 0) < vioRegion(2, 0), vioRegion(2, 0) = Pnt(2, 0));
		_HIVE_SIMPLE_IF(Pnt(0, 0) > vioRegion(0, 1), vioRegion(0, 1) = Pnt(0, 0));
		_HIVE_SIMPLE_IF(Pnt(1, 0) > vioRegion(1, 1), vioRegion(1, 1) = Pnt(1, 0));
		_HIVE_SIMPLE_IF(Pnt(2, 0) > vioRegion(2, 1), vioRegion(2, 1) = Pnt(2, 0));
		voCentroid += Pnt;
	}
	voCentroid /= vPointSet.size();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__updateRegion(const std::vector<unsigned>& vPointSet, RegionDim3& vioRegion) const
{
	const double* pPos = m_pPointCloud->getPosPointer();
	for (std::vector<unsigned>::const_iterator PntItr=vPointSet.begin(); PntItr!=vPointSet.end(); ++PntItr)
	{
		ColVector3 Pnt(pPos[(*PntItr)*3 + 0], pPos[(*PntItr)*3 + 1], pPos[(*PntItr)*3 + 2]);
		_HIVE_SIMPLE_IF(Pnt(0, 0) < vioRegion(0, 0), vioRegion(0, 0) = Pnt(0, 0));
		_HIVE_SIMPLE_IF(Pnt(1, 0) < vioRegion(1, 0), vioRegion(1, 0) = Pnt(1, 0));
		_HIVE_SIMPLE_IF(Pnt(2, 0) < vioRegion(2, 0), vioRegion(2, 0) = Pnt(2, 0));
		_HIVE_SIMPLE_IF(Pnt(0, 0) > vioRegion(0, 1), vioRegion(0, 1) = Pnt(0, 0));
		_HIVE_SIMPLE_IF(Pnt(1, 0) > vioRegion(1, 1), vioRegion(1, 1) = Pnt(1, 0));
		_HIVE_SIMPLE_IF(Pnt(2, 0) > vioRegion(2, 1), vioRegion(2, 1) = Pnt(2, 0));
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__computeCentroid(const std::vector<unsigned>& vPointSet, ColVector3& voCentroid) const
{
	const double* pPos = m_pPointCloud->getPosPointer();
	voCentroid.setZero();
	for (std::vector<unsigned>::const_iterator PntItr=vPointSet.begin(); PntItr!=vPointSet.end(); ++PntItr)
	{
		ColVector3 Pnt(pPos[(*PntItr)*3 + 0], pPos[(*PntItr)*3 + 1], pPos[(*PntItr)*3 + 2]);
		voCentroid += Pnt;
	}
	voCentroid /= vPointSet.size();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSamplerSplit::__init8SplitNode(const RegionDim3& vParentRegion, const ColVector3& vCent, SplitNode voSplitNode[8]) const
{
	Eigen::Matrix3d Tmp3Pnt;
	Tmp3Pnt.col(0) = vParentRegion.col(0);
	Tmp3Pnt.col(1) = vCent;
	Tmp3Pnt.col(2) = vParentRegion.col(1);

	// down front left
	voSplitNode[0].NodeRegion.col(0) << Tmp3Pnt(0, 0), Tmp3Pnt(1, 0), Tmp3Pnt(2, 0);
	voSplitNode[0].NodeRegion.col(1) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 1), Tmp3Pnt(2, 1);

	// down front right
	voSplitNode[1].NodeRegion.col(0) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 0), Tmp3Pnt(2, 0);
	voSplitNode[1].NodeRegion.col(1) << Tmp3Pnt(0, 2), Tmp3Pnt(1, 1), Tmp3Pnt(2, 1);

	// down back left
	voSplitNode[2].NodeRegion.col(0) << Tmp3Pnt(0, 0), Tmp3Pnt(1, 1), Tmp3Pnt(2, 0);
	voSplitNode[2].NodeRegion.col(1) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 2), Tmp3Pnt(2, 1);

	// down back right
	voSplitNode[3].NodeRegion.col(0) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 1), Tmp3Pnt(2, 0);
	voSplitNode[3].NodeRegion.col(1) << Tmp3Pnt(0, 2), Tmp3Pnt(1, 2), Tmp3Pnt(2, 1);

	// up front left
	voSplitNode[4].NodeRegion.col(0) << Tmp3Pnt(0, 0), Tmp3Pnt(1, 0), Tmp3Pnt(2, 1);
	voSplitNode[4].NodeRegion.col(1) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 1), Tmp3Pnt(2, 2);

	// up front right
	voSplitNode[5].NodeRegion.col(0) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 0), Tmp3Pnt(2, 1);
	voSplitNode[5].NodeRegion.col(1) << Tmp3Pnt(0, 2), Tmp3Pnt(1, 1), Tmp3Pnt(2, 2);

	// up back left
	voSplitNode[6].NodeRegion.col(0) << Tmp3Pnt(0, 0), Tmp3Pnt(1, 1), Tmp3Pnt(2, 1);
	voSplitNode[6].NodeRegion.col(1) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 2), Tmp3Pnt(2, 2);

	// up back right
	voSplitNode[7].NodeRegion.col(0) << Tmp3Pnt(0, 1), Tmp3Pnt(1, 1), Tmp3Pnt(2, 1);
	voSplitNode[7].NodeRegion.col(1) << Tmp3Pnt(0, 2), Tmp3Pnt(1, 2), Tmp3Pnt(2, 2);
}
