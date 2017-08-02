#pragma once
#include <vector>
#include <utility>
#include "PointCloudSubset.h"
#include "LCSNSType.h"
#include "HiveCommon.h"
#include "HiveCommonMicro.h"
#include "EventLoggerInterface.h"
#include "RegUtilityFunctions.h"
#include "Bicubic.h"


namespace hiveRegistration
{
	template <typename CtrlPntSetType>
	class CCommonLCSNSSubset : public CBasePointCloudSubset
	{
	public:
		explicit CCommonLCSNSSubset(CtrlPntSetType vControlPoints);

		void swapUVSet(std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet);
		void setUVSet(const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet);

		void swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet);
		void setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet);

		void swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet);
		void setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet);

		const CtrlPntSetType& getControlPointsSet() const                { return m_CtrlPntsSet; }
		const std::vector<std::pair<int, int> >& getIndexPairSet() const { return m_IndexPairSet; }
		const std::vector<std::pair<double, double> >& getUVSet() const  { return m_UVSet; }

	protected:
		CCommonLCSNSSubset(const CCommonLCSNSSubset& vOther)
			: m_CtrlPntsSet(vOther.m_CtrlPntsSet)
			, m_IndexPairSet(vOther.m_IndexPairSet)
			, m_UVSet(vOther.m_UVSet)
		{
			m_PointSet = vOther.m_PointSet;
			m_NormalSet = vOther.m_NormalSet;
		}

		const CCommonLCSNSSubset& operator=(const CCommonLCSNSSubset& vOther);

		virtual ~CCommonLCSNSSubset() {}

	private:
		void __refresh();
		void __clear();
		void __computePntNormals(void);

		const ControlPoints& __getCtrlPoints(const PtrControlPointsSet& vCtrlPntsSet, unsigned int vIdx) { return (*vCtrlPntsSet)[vIdx].second; }
		const ControlPoints& __getCtrlPoints(const PtrSimpleCtrlPntSet& vCtrlPntsSet, unsigned int vIdx) { return (*vCtrlPntsSet)[vIdx]; }

	private:
		CtrlPntSetType					        m_CtrlPntsSet;
		std::vector<std::pair<int, int> >		m_IndexPairSet; 
		std::vector<std::pair<double, double> >	m_UVSet;
	};
}

namespace hiveRegistration
{
	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	CCommonLCSNSSubset<CtrlPntSetType>::CCommonLCSNSSubset( CtrlPntSetType vControlPoints )
	: m_CtrlPntsSet(vControlPoints)
	{
		m_NormalSet.reserve(vControlPoints->size());
		m_PointSet.reserve(vControlPoints->size());
	}

		
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet)
	{
		_ASSERT(vCorrespondencePointSet.size() == vIndexPairSet.size() && vNormalSet.size() == vIndexPairSet.size() && m_CtrlPntsSet.get() && m_CtrlPntsSet->size() == vIndexPairSet.size());

		m_PointSet.swap(vCorrespondencePointSet);
		m_IndexPairSet.swap(vIndexPairSet);
		m_NormalSet.swap(vNormalSet);
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(
			vCorrespondencePointSet.size() == vIndexPairSet.size() && 
			vNormalSet.size() == vIndexPairSet.size() && 
			vUVSet.size() == vIndexPairSet.size() &&
			m_CtrlPntsSet.get() && 
			m_CtrlPntsSet->size() == vIndexPairSet.size());

		m_PointSet.swap(vCorrespondencePointSet);
		m_IndexPairSet.swap(vIndexPairSet);
		m_NormalSet.swap(vNormalSet);
		m_UVSet.swap(vUVSet);
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet)
	{
		_ASSERT(vCorrespondencePointSet.size() == vIndexPairSet.size() && vNormalSet.size() == vIndexPairSet.size() && m_CtrlPntsSet.get() && m_CtrlPntsSet->size() == vIndexPairSet.size());

		m_PointSet = vCorrespondencePointSet;
		m_IndexPairSet = vIndexPairSet;
		m_NormalSet = vNormalSet;
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(
			vCorrespondencePointSet.size() == vIndexPairSet.size() && 
			vNormalSet.size() == vIndexPairSet.size() && 
			vUVSet.size() == vIndexPairSet.size() &&
			m_CtrlPntsSet.get() && 
			m_CtrlPntsSet->size() == vIndexPairSet.size());

		m_PointSet = vCorrespondencePointSet;
		m_IndexPairSet = vIndexPairSet;
		m_NormalSet = vNormalSet;
		m_UVSet = vUVSet;
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::__refresh()
	{
		__clear();
		__computePntNormals();
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::__clear()
	{
		m_PointSet.clear();
		m_NormalSet.clear();
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::swapUVSet(std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(vIndexPairSet.size() == vUVSet.size() && m_CtrlPntsSet.get() && m_CtrlPntsSet->size() == vUVSet.size());
		m_IndexPairSet.swap(vIndexPairSet);
		m_UVSet.swap(vUVSet);
		__refresh();
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::setUVSet(const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet)
	{
		_ASSERT(vIndexPairSet.size() == vUVSet.size() && m_CtrlPntsSet.get() && m_CtrlPntsSet->size() == vUVSet.size());
		m_UVSet = vUVSet;
		m_IndexPairSet = vIndexPairSet;
		__refresh();
	}

	//*********************************************************************************
	//FUNCTION:
	template <typename CtrlPntSetType>
	void CCommonLCSNSSubset<CtrlPntSetType>::__computePntNormals(void)
	{
		_ASSERT(m_UVSet.size() == m_IndexPairSet.size() && m_CtrlPntsSet.get() && m_CtrlPntsSet->size() == m_UVSet.size());

		Bicubic::CBicubic BicubicObj;
		Eigen::Matrix4d	CtrlPoints[3];

		for (unsigned int i=0; i<m_IndexPairSet.size(); ++i)
		{
			int Idx[2] = { m_IndexPairSet[i].first, m_IndexPairSet[i].second };
			const Eigen::Matrix<ColVector3, Eigen::Dynamic, Eigen::Dynamic>& CurCtrlPnts = __getCtrlPoints(m_CtrlPntsSet, i);
			CtrlPoints[0] <<
				CurCtrlPnts(Idx[0], Idx[1]  )(0, 0), CurCtrlPnts(Idx[0]+1, Idx[1]  )(0, 0), CurCtrlPnts(Idx[0]+2, Idx[1]  )(0, 0), CurCtrlPnts(Idx[0]+3, Idx[1]  )(0, 0),
				CurCtrlPnts(Idx[0], Idx[1]+1)(0, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+1)(0, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+1)(0, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+1)(0, 0),
				CurCtrlPnts(Idx[0], Idx[1]+2)(0, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+2)(0, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+2)(0, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+2)(0, 0),
				CurCtrlPnts(Idx[0], Idx[1]+3)(0, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+3)(0, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+3)(0, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+3)(0, 0);

			CtrlPoints[1] <<
				CurCtrlPnts(Idx[0], Idx[1]  )(1, 0), CurCtrlPnts(Idx[0]+1, Idx[1]  )(1, 0), CurCtrlPnts(Idx[0]+2, Idx[1]  )(1, 0), CurCtrlPnts(Idx[0]+3, Idx[1]  )(1, 0),
				CurCtrlPnts(Idx[0], Idx[1]+1)(1, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+1)(1, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+1)(1, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+1)(1, 0),
				CurCtrlPnts(Idx[0], Idx[1]+2)(1, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+2)(1, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+2)(1, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+2)(1, 0),
				CurCtrlPnts(Idx[0], Idx[1]+3)(1, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+3)(1, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+3)(1, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+3)(1, 0);

			CtrlPoints[2] <<
				CurCtrlPnts(Idx[0], Idx[1]  )(2, 0), CurCtrlPnts(Idx[0]+1, Idx[1]  )(2, 0), CurCtrlPnts(Idx[0]+2, Idx[1]  )(2, 0), CurCtrlPnts(Idx[0]+3, Idx[1]  )(2, 0),
				CurCtrlPnts(Idx[0], Idx[1]+1)(2, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+1)(2, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+1)(2, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+1)(2, 0),
				CurCtrlPnts(Idx[0], Idx[1]+2)(2, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+2)(2, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+2)(2, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+2)(2, 0),
				CurCtrlPnts(Idx[0], Idx[1]+3)(2, 0), CurCtrlPnts(Idx[0]+1, Idx[1]+3)(2, 0), CurCtrlPnts(Idx[0]+2, Idx[1]+3)(2, 0), CurCtrlPnts(Idx[0]+3, Idx[1]+3)(2, 0);

			ColVector3 TmpPnt;
			ColVector3 TmpNorm;
			BicubicObj.computeNormal(m_UVSet[i].first, m_UVSet[i].second, CtrlPoints, TmpNorm);
			BicubicObj.compute3DPointPosByUV(m_UVSet[i].first, m_UVSet[i].second, CtrlPoints, TmpPnt);
			m_PointSet.push_back(TmpPnt);
			m_NormalSet.push_back(TmpNorm);
		}
	}
}