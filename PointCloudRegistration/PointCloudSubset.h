#pragma once
#include <fstream>
#include <boost\shared_ptr.hpp>
#include "ICPBaseObject.h"
#include "ControlPointsData.h"
#include "PointCloud.h"
#include "RegUtilityFunctions.h"


namespace hiveRegistration
{
	//*********************************************************************************
	//CLASS:
	class CBasePointCloudSubset : public CICPBaseObject
	{
	public:
		const VecColVector3& getPointSet() const  { return m_PointSet; }
		const VecColVector3& getNormalSet() const { return m_NormalSet; }			// FIXME: not recommended!
		const unsigned int getNumPoint() const    { return m_PointSet.size(); }

		const CBasePointCloudSubset& transformSelf(const Eigen::Matrix3d& vR, const ColVector3& vT)
		{
			return _transformSelfV(vR, vT);
		}
		const CBasePointCloudSubset& transformBackSelf(const Eigen::Matrix3d& vR, const ColVector3& vT)
		{
			return _transformBackSelfV(vR, vT);
		}

		boost::shared_ptr<CBasePointCloudSubset>
			transform2NewCorPointSet(const Eigen::Matrix3d& vR, const ColVector3& vT) const
		{
			return _transform2NewCorPointSetV(vR, vT);
		}

		boost::shared_ptr<CBasePointCloudSubset>
			transformBack(const Eigen::Matrix3d& vR, const ColVector3& vT) const
		{
			return _transformBackV(vR, vT);
		}

		void export2TXT(const std::string& vTXT) const
		{
			std::ofstream OutFile(vTXT.c_str());
			_ASSERT(OutFile.is_open());
			for (VecColVector3::const_iterator citr=m_PointSet.begin(); citr!=m_PointSet.end(); ++citr)
			{
				OutFile << (*citr)(0, 0) << " " << (*citr)(1, 0) << " " << (*citr)(2, 0) << std::endl;
			}

			OutFile.close();
		}

	protected:
		CBasePointCloudSubset() {}
		CBasePointCloudSubset(const CBasePointCloudSubset& vOther) : m_PointSet(vOther.m_PointSet), m_NormalSet(vOther.m_NormalSet) {}
		virtual ~CBasePointCloudSubset() {}

		virtual const CBasePointCloudSubset&	_transformSelfV(const Eigen::Matrix3d& vR, const ColVector3& vT);
		virtual const CBasePointCloudSubset&	_transformBackSelfV(const Eigen::Matrix3d& vR, const ColVector3& vT);

		virtual boost::shared_ptr<CBasePointCloudSubset>
			_transform2NewCorPointSetV(const Eigen::Matrix3d& vR, const ColVector3& vT) const;

		virtual boost::shared_ptr<CBasePointCloudSubset>
			_transformBackV(const Eigen::Matrix3d& vR, const ColVector3& vT) const;

		VecColVector3 m_PointSet;
		VecColVector3 m_NormalSet;

	private:
		CBasePointCloudSubset operator=(const CBasePointCloudSubset& vOther)
		{
			m_PointSet = vOther.m_PointSet;
			m_NormalSet = vOther.m_NormalSet;
		}
	};

	//*********************************************************************************
	//CLASS:
	class CPntNormSubset : public CBasePointCloudSubset
	{
	public:
		void setPointSet(const VecColVector3& vPointSet)   { m_PointSet = vPointSet; }
		void setNormalSet(const VecColVector3& vNormalSet) { m_NormalSet = vNormalSet; }

		void swapPointSet(VecColVector3& vPointSet)   { m_PointSet.swap(vPointSet); }
		void swapNormalSet(VecColVector3& vNormalSet) { m_NormalSet.swap(vNormalSet); }

	protected:
		virtual ~CPntNormSubset() {}

		virtual boost::shared_ptr<CBasePointCloudSubset>
			_transform2NewCorPointSetV(const Eigen::Matrix3d& vR, const ColVector3& vT) const override;

		virtual boost::shared_ptr<CBasePointCloudSubset>
			_transformBackV(const Eigen::Matrix3d& vR, const ColVector3& vT) const override;
	};

	//*********************************************************************************
	//CLASS:
	class CIndexSubset : public CBasePointCloudSubset
	{
	public:
		explicit CIndexSubset(const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndexSet = std::vector<unsigned int>(), bool vUpdate = true);

		const hivePointCloud::CPointCloud& getPointCloud() const { return m_PointCloud; }
		const std::vector<unsigned int>& getIndexSet()     const { return m_IndexSet; }
		void updatePointAndNormalSet(void);
		void setIndexSet(const std::vector<unsigned int>& vIndexSet, bool vUpdate = true);
		void swapIndexSet(std::vector<unsigned int>& vIndexSet, bool vUpdate = true);

	protected:
		virtual ~CIndexSubset() {}

		boost::shared_ptr<CBasePointCloudSubset>
			_transform2NewCorPointSetV(const Eigen::Matrix3d& vR, const ColVector3& vT) const override;

		boost::shared_ptr<CBasePointCloudSubset>
			_transformBackV(const Eigen::Matrix3d& vR, const ColVector3& vT) const override;

	private:
		std::vector<unsigned int> m_IndexSet;
		const hivePointCloud::CPointCloud& m_PointCloud;

		void __update();
	};

	class CBicubicGridSubset : public CBasePointCloudSubset
	{
	public:
		explicit CBicubicGridSubset(const MatrixControlPoints& vControlPoints);

		void swapUVSet(std::vector<std::pair<int, int> >& vIndexPairSet, std::vector<std::pair<double, double> >& vUVSet);
		void setUVSet(const std::vector<std::pair<int, int> >& vIndexPairSet, const std::vector<std::pair<double, double> >& vUVSet);

		void swapPointSet(VecColVector3& vCorrespondencePointSet, VecColVector3& vNormalSet, std::vector<std::pair<int, int> >& vIndexPairSet);
		void setPointSet(const VecColVector3& vCorrespondencePointSet, const VecColVector3& vNormalSet, const std::vector<std::pair<int, int> >& vIndexPairSet);

		const MatrixControlPoints& getControlPoints() const { return m_ControlPoints; }
		const std::vector<std::pair<int, int> >& getIndexPairSet() const { return m_IndexPairSet; }
		const std::vector<std::pair<double, double> >& getUVSet() const { return m_UVSet; }

	protected:
		CBicubicGridSubset(const CBicubicGridSubset& vOther);
		const CBicubicGridSubset& operator=(const CBicubicGridSubset& vOther);

		virtual ~CBicubicGridSubset() {}

	private:
		void __refresh();
		void __clear();
		void __computePntNormals(void);

	private:
		MatrixControlPoints					    m_ControlPoints;
		std::vector<std::pair<int, int> >		m_IndexPairSet; 
		std::vector<std::pair<double, double> >	m_UVSet;
	};
}