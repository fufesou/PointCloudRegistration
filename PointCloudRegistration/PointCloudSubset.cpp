#include "PointCloudSubset.h"
#include <Eigen\LU>
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "RegUtilityFunctions.h"
#include "Bicubic.h"

#define IMPL_TRANSFORM_FUNC(ClassName) \
	boost::shared_ptr<CBasePointCloudSubset> ClassName::_transform2NewCorPointSetV(const Eigen::Matrix3d& vR, const ColVector3& vT) const \
	{ \
		boost::shared_ptr<CBasePointCloudSubset> pNewCorPointSet = makeSharedPtr(new ClassName(*this)); \
		pNewCorPointSet->transformSelf(vR, vT); \
		return pNewCorPointSet; \
	}

#define IMPL_TRANSFORM_BACK_FUNC(ClassName) \
	boost::shared_ptr<CBasePointCloudSubset> ClassName::_transformBackV(const Eigen::Matrix3d& vR, const ColVector3& vT) const \
	{ \
		boost::shared_ptr<CBasePointCloudSubset> pNewCorPointSet = makeSharedPtr(new ClassName(*this)); \
		pNewCorPointSet->transformBackSelf(vR, vT); \
		return pNewCorPointSet; \
	}

namespace hiveRegistration
{
	//*********************************************************************************
	//FUNCTION:
	const CBasePointCloudSubset& CBasePointCloudSubset::_transformSelfV(const Eigen::Matrix3d& vR, const ColVector3& vT)
	{
		for (VecColVector3::size_type i=0; i!=m_PointSet.size(); ++i)
		{
			m_PointSet[i] = vR * m_PointSet[i] + vT;
			m_NormalSet[i] = vR * m_NormalSet[i];
		}
		return *this;
	}

	//*********************************************************************************
	//FUNCTION:
	const CBasePointCloudSubset& CBasePointCloudSubset::_transformBackSelfV(const Eigen::Matrix3d& vR, const ColVector3& vT)
	{
		const Eigen::Matrix3d InverseRotationMatrix = vR.inverse();
		for (VecColVector3::size_type i=0; i!=m_PointSet.size(); ++i)
		{
			m_PointSet[i] = InverseRotationMatrix * (m_PointSet[i] - vT);
			m_NormalSet[i] = InverseRotationMatrix * m_NormalSet[i];
		}
		return *this;
	}

	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_FUNC(CBasePointCloudSubset)

	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_BACK_FUNC(CBasePointCloudSubset)
}

namespace hiveRegistration
{
	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_FUNC(CPntNormSubset)

	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_BACK_FUNC(CPntNormSubset)
}

namespace hiveRegistration
{
	CIndexSubset::CIndexSubset(const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndexSet /*= std::vector<unsigned int>()*/, bool vUpdate /*= true*/) : m_PointCloud(vPointCloud)
		, m_IndexSet(vIndexSet)
	{
		if (vUpdate) __update();
	}


	//*********************************************************************************
	//FUNCTION:
	void CIndexSubset::__update()
	{
		const int PointIndexSetSize = m_IndexSet.size();
		const double* pPos = m_PointCloud.getPosPointer();
		const double* pNorm = m_PointCloud.getNormalPointer();

		if (pPos)
		{
			m_PointSet.resize(m_IndexSet.size());

			#pragma omp parallel for
			for (int i=0; i<PointIndexSetSize; ++i)
			{
				m_PointSet[i](0, 0) = pPos[m_IndexSet[i]*3    ];
				m_PointSet[i](1, 0) = pPos[m_IndexSet[i]*3 + 1];
				m_PointSet[i](2, 0) = pPos[m_IndexSet[i]*3 + 2];
			}
		}
		if (pNorm)
		{
			m_NormalSet.resize(m_IndexSet.size());

			#pragma omp parallel for
			for (int i=0; i<PointIndexSetSize; ++i)
			{
				m_NormalSet[i](0, 0) = pNorm[m_IndexSet[i]*3    ];
				m_NormalSet[i](1, 0) = pNorm[m_IndexSet[i]*3 + 1];
				m_NormalSet[i](2, 0) = pNorm[m_IndexSet[i]*3 + 2];
			}
		}
	}

	//*********************************************************************************
	//FUNCTION:
	void CIndexSubset::setIndexSet(const std::vector<unsigned int>& vIndexSet, bool vUpdate /*= true*/)
	{
		m_IndexSet = vIndexSet;
		if (vUpdate) __update();
	}

	//*********************************************************************************
	//FUNCTION:
	void CIndexSubset::swapIndexSet(std::vector<unsigned int>& vIndexSet, bool vUpdate /*= true*/)
	{
		m_IndexSet.swap(vIndexSet);
		if (vUpdate) __update();
	}

	//*********************************************************************************
	//FUNCTION:
	void CIndexSubset::updatePointAndNormalSet(void)
	{
		__update();
	}

	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_FUNC(CIndexSubset)

	//*********************************************************************************
	//FUNCTION:
	IMPL_TRANSFORM_BACK_FUNC(CIndexSubset)
}


namespace hiveRegistration
{
	CBicubicGridSubset::CBicubicGridSubset( const MatrixControlPoints& vControlPoints )
	: m_ControlPoints(vControlPoints)
	{
		m_NormalSet.reserve(m_ControlPoints.rows() * m_ControlPoints.cols());
	}

	CBicubicGridSubset::CBicubicGridSubset( const CBicubicGridSubset& vOther )
	: m_ControlPoints(vOther.m_ControlPoints)
	, m_IndexPairSet(vOther.m_IndexPairSet)
	, m_UVSet(vOther.m_UVSet)
	{
		m_PointSet = vOther.m_PointSet;
		m_NormalSet = vOther.m_NormalSet;
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet)
	{
		_ASSERT(vCorrespondencePointSet.size() == vIndexPairSet.size());

		m_PointSet.swap(vCorrespondencePointSet);
		m_IndexPairSet.swap(vIndexPairSet);
		m_NormalSet.swap(vNormalSet);
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet)
	{
		_ASSERT(vCorrespondencePointSet.size() == vIndexPairSet.size());

		m_PointSet = vCorrespondencePointSet;
		m_IndexPairSet = vIndexPairSet;
		m_NormalSet = vNormalSet;
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::__refresh()
	{
		__clear();
		__computePntNormals();
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::__clear()
	{
		m_PointSet.clear();
		m_NormalSet.clear();
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::swapUVSet(std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(vIndexPairSet.size() == vUVSet.size());
		m_IndexPairSet.swap(vIndexPairSet);
		m_UVSet.swap(vUVSet);
		__refresh();
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::setUVSet(const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(vIndexPairSet.size() == vUVSet.size());
		m_UVSet = vUVSet;
		m_IndexPairSet = vIndexPairSet;
		__refresh();
	}

	//*********************************************************************************
	//FUNCTION:
	void CBicubicGridSubset::__computePntNormals(void)
	{
		_ASSERT(m_UVSet.size() == m_IndexPairSet.size());

		Bicubic::CBicubic BicubicObj;
		Eigen::Matrix4d	CtrlPoints[3];

		for (unsigned int i=0; i<m_IndexPairSet.size(); ++i)
		{
			int Idx[2] = { m_IndexPairSet[i].first, m_IndexPairSet[i].second };
			CtrlPoints[0] <<
				m_ControlPoints(Idx[0], Idx[1]  )[0](0, 0), m_ControlPoints(Idx[0]+1, Idx[1]  )[0](0, 0), m_ControlPoints(Idx[0]+2, Idx[1]  )[0](0, 0), m_ControlPoints(Idx[0]+3, Idx[1]  )[0](0, 0),
				m_ControlPoints(Idx[0], Idx[1]+1)[0](0, 0), m_ControlPoints(Idx[0]+1, Idx[1]+1)[0](0, 0), m_ControlPoints(Idx[0]+2, Idx[1]+1)[0](0, 0), m_ControlPoints(Idx[0]+3, Idx[1]+1)[0](0, 0),
				m_ControlPoints(Idx[0], Idx[1]+2)[0](0, 0), m_ControlPoints(Idx[0]+1, Idx[1]+2)[0](0, 0), m_ControlPoints(Idx[0]+2, Idx[1]+2)[0](0, 0), m_ControlPoints(Idx[0]+3, Idx[1]+2)[0](0, 0),
				m_ControlPoints(Idx[0], Idx[1]+3)[0](0, 0), m_ControlPoints(Idx[0]+1, Idx[1]+3)[0](0, 0), m_ControlPoints(Idx[0]+2, Idx[1]+3)[0](0, 0), m_ControlPoints(Idx[0]+3, Idx[1]+3)[0](0, 0);

			CtrlPoints[1] <<
				m_ControlPoints(Idx[0], Idx[1]  )[0](1, 0), m_ControlPoints(Idx[0]+1, Idx[1]  )[0](1, 0), m_ControlPoints(Idx[0]+2, Idx[1]  )[0](1, 0), m_ControlPoints(Idx[0]+3, Idx[1]  )[0](1, 0),
				m_ControlPoints(Idx[0], Idx[1]+1)[0](1, 0), m_ControlPoints(Idx[0]+1, Idx[1]+1)[0](1, 0), m_ControlPoints(Idx[0]+2, Idx[1]+1)[0](1, 0), m_ControlPoints(Idx[0]+3, Idx[1]+1)[0](1, 0),
				m_ControlPoints(Idx[0], Idx[1]+2)[0](1, 0), m_ControlPoints(Idx[0]+1, Idx[1]+2)[0](1, 0), m_ControlPoints(Idx[0]+2, Idx[1]+2)[0](1, 0), m_ControlPoints(Idx[0]+3, Idx[1]+2)[0](1, 0),
				m_ControlPoints(Idx[0], Idx[1]+3)[0](1, 0), m_ControlPoints(Idx[0]+1, Idx[1]+3)[0](1, 0), m_ControlPoints(Idx[0]+2, Idx[1]+3)[0](1, 0), m_ControlPoints(Idx[0]+3, Idx[1]+3)[0](1, 0);

			CtrlPoints[2] <<
				m_ControlPoints(Idx[0], Idx[1]  )[0](2, 0), m_ControlPoints(Idx[0]+1, Idx[1]  )[0](2, 0), m_ControlPoints(Idx[0]+2, Idx[1]  )[0](2, 0), m_ControlPoints(Idx[0]+3, Idx[1]  )[0](2, 0),
				m_ControlPoints(Idx[0], Idx[1]+1)[0](2, 0), m_ControlPoints(Idx[0]+1, Idx[1]+1)[0](2, 0), m_ControlPoints(Idx[0]+2, Idx[1]+1)[0](2, 0), m_ControlPoints(Idx[0]+3, Idx[1]+1)[0](2, 0),
				m_ControlPoints(Idx[0], Idx[1]+2)[0](2, 0), m_ControlPoints(Idx[0]+1, Idx[1]+2)[0](2, 0), m_ControlPoints(Idx[0]+2, Idx[1]+2)[0](2, 0), m_ControlPoints(Idx[0]+3, Idx[1]+2)[0](2, 0),
				m_ControlPoints(Idx[0], Idx[1]+3)[0](2, 0), m_ControlPoints(Idx[0]+1, Idx[1]+3)[0](2, 0), m_ControlPoints(Idx[0]+2, Idx[1]+3)[0](2, 0), m_ControlPoints(Idx[0]+3, Idx[1]+3)[0](2, 0);

			ColVector3 TmpPnt;
			ColVector3 TmpNorm;
			BicubicObj.computeNormal(m_UVSet[i].first, m_UVSet[i].second, CtrlPoints, TmpNorm);
			BicubicObj.compute3DPointPosByUV(m_UVSet[i].first, m_UVSet[i].second, CtrlPoints, TmpPnt);
			m_PointSet.push_back(TmpPnt);
			m_NormalSet.push_back(TmpNorm);
		}
	}
}