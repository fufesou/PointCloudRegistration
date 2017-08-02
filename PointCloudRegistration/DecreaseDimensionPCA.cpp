#include "DecreaseDimensionPCA.h"
#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/LU>


//*********************************************************************************
//FUNCTION:
void hiveRegistration::decreaseDimensionPCA(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vSpinMapData, 
											unsigned int vK, 
											Eigen::Matrix<double, Eigen::Dynamic, 1>& voOrigin,
											Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voAxises,
											Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voPcaDecreaseDimensionMapData)
{
	voOrigin.resize(vSpinMapData.rows());
	voOrigin.setZero();
	unsigned int Cols = vSpinMapData.cols();
	unsigned int Rows = vSpinMapData.rows();
	for (unsigned int i=0; i<Cols; ++i)
	{
		voOrigin += vSpinMapData.col(i);
	}
	voOrigin /= Cols;

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TempMatrix;
	TempMatrix.resize(vSpinMapData.rows(), vSpinMapData.cols());

	for (unsigned int i=0; i<Cols; ++i)
	{
		TempMatrix.col(i) = vSpinMapData.col(i) - voOrigin;
	}

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  CovarianceMatrix;
	CovarianceMatrix.resize(vSpinMapData.rows(), vSpinMapData.rows());
	CovarianceMatrix = TempMatrix * TempMatrix.transpose();

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> EigenSolver(CovarianceMatrix);
	const Eigen::MatrixXd& EigenVectors = EigenSolver.eigenvectors();

	voAxises.resize(vSpinMapData.rows(), vK);
	for (unsigned int i=0; i<vK; ++i)
	{
		voAxises.col(i) = EigenVectors.col(Rows - 1 - i);
		voAxises.col(i).normalize();
	}

	voPcaDecreaseDimensionMapData = (TempMatrix.transpose() * voAxises).transpose();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::project2NewCoordsSystem(
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vSpinMapData, 
	const Eigen::Matrix<double, Eigen::Dynamic, 1>& vOrigin, 
	const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& vAxises, 
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voPcaDecreaseDimensionMapData)
{
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> TempMatrix;
	TempMatrix.resize(vSpinMapData.rows(), vSpinMapData.cols());
	for (unsigned int i=0; i<vSpinMapData.cols(); ++i)
	{
		TempMatrix.col(i) = vSpinMapData.col(i) - vOrigin;
	}
	voPcaDecreaseDimensionMapData = (TempMatrix.transpose() * vAxises).transpose();
}
