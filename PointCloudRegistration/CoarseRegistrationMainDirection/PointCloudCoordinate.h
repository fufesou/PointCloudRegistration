#pragma once
#include <Eigen/Core>

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CPointCloudCoordinate
	{
	public:
		CPointCloudCoordinate();

		const Eigen::Matrix<double, 3, 3>& getAxisDir() const { return m_AxisDir; }
		const Eigen::Matrix<double, 3, 1>& getOriginPoint() const { return m_OriginPoint; }

		void refresh(const hivePointCloud::CPointCloud* vPointCloud);

	private:
		const hivePointCloud::CPointCloud* m_pPointCloud;

		Eigen::Matrix<double, 3, 3> m_AxisDir;
		Eigen::Matrix<double, 3, 1> m_OriginPoint;

		void __setCoordinate();
		void __computeAxisDir();
		void __buildConvarianceMatrix(Eigen::Matrix3d& voConvarianceMatrix);
		void __sortEigenValuesAndVectors(Eigen::Vector3d& vioValues, Eigen::Matrix3d& vioVectors);
	};
}