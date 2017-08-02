#pragma once
#include <Eigen/core>


namespace hiveRegistration
{
	void decreaseDimensionPCA(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vSpinMapData, unsigned int vK, Eigen::Matrix<double, Eigen::Dynamic, 1>& voOrigin, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voAxises, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voPcaDecreaseDimensionMapData);
	void project2NewCoordsSystem( const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vSpinMapData, const Eigen::Matrix<double, Eigen::Dynamic, 1>& vOrigin, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vAxises, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voPcaDecreaseDimensionMapData);
}