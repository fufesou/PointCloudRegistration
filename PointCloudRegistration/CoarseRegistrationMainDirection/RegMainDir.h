#pragma once
#include <utility>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CRegMainDirect
	{
	public:
		CRegMainDirect();

		static std::string getKeyCoincidentThreshold() { return boost::algorithm::to_upper_copy<std::string>("RegMainDirect.CT"); }

		bool fit(const hivePointCloud::CPointCloud& vSrc, const hivePointCloud::CPointCloud& vTgt,
			Eigen::Matrix3d& voR, ColVector3& voT, double& voCoincidentCoeff);

	private:
		typedef std::pair<Eigen::Matrix<double, 3, 1>, Eigen::Matrix<double, 3, 3> > CoordDataType;

		std::string m_FitRes;
		CoordDataType m_SrcCoordData;
		CoordDataType m_TgtCoordData;
		const hivePointCloud::CPointCloud* m_pSrcPointCloud;
		const hivePointCloud::CPointCloud* m_pTgtPointCloud;

		bool __fit(Eigen::Matrix3d& voR, ColVector3& voT, double& vioCoincidentCoeff);
		void __refreshData(const hivePointCloud::CPointCloud& vSrc, const hivePointCloud::CPointCloud& vTgt);
		void __computeCoordTransformation(const CoordDataType& vSrcCoord, const CoordDataType& vTgtCoord, Eigen::Matrix3d& voR, ColVector3& voT);
		void __transformatePointCloud(const double* vOriginPos, const unsigned int vNumPoint, const Eigen::Matrix3d& vR, const ColVector3& vT, double* voNewPos);
		bool __fitOneDir(const CoordDataType& vSrcCoord, const CoordDataType& vTgtCoord, Eigen::Matrix3d& voR, ColVector3& voT, double& vioCoincidentCoeff);
		double __computeCurCoincidentCoeff(const Eigen::Matrix<double, 3, 2>& vRegionA, const Eigen::Matrix<double, 3, 2>& vRegionB);
		double __computeVolume(const Eigen::Matrix<double, 3, 2>& vBox);
		double __computeCoincidentVolume(const Eigen::Matrix<double, 3, 2>& vBoxA, const Eigen::Matrix<double, 3, 2>& vBoxB);
	};
} 