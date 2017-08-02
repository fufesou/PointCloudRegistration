#pragma once
#include <fstream>
#include <Eigen/Core>
#include <osg/Vec3d>
#include "ICPType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	void saveCorPairSet(const CorrespondencePairSet& vCorPairSet, const std::string& vFileName, const Eigen::Matrix3d& vRot, const ColVector3& vTra);

	char *textFileRead(const char *vFileName);
	void initPointAndProperties(const char *vFilename, const int vNumProperty, std::vector<osg::Vec3d>& voPointSet, std::vector<std::vector<double> >& voProperties);

	void initRotationAndTranslation(Eigen::Matrix3d& voRot, ColVector3& voTra);

	template<typename TIter>
	void exportPointIndex(TIter vBegin, TIter vEnd, const char* vOutFile);

	void export2Ply(std::ofstream& vOutFile, const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndices);
	void export2Ply(std::ofstream& vOutFile, const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIndex);
	void export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndices);
	void export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIndex);
	void export2Ply(const char* vFileName, const VecColVector3& vPointSet);
	void export2Ply(const char* vFileName, const VecColVector3& vPointSet, const VecColVector3& vNormSet);
	void export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud);
	void fillPlyHeader(std::ofstream& vOutFile, unsigned int vNumPoints);
	void fillPlyHeaderWithoutNorm(std::ofstream& vOutFile, unsigned int vNumPoints);

	double computeCurrentFitRatio(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, 
		const Eigen::Matrix3d& vR, Eigen::Vector3d& vT,
		const hivePointCloud::CPointCloud& vIdxPointCloud,
		const hivePointCloud::CPointCloud& vGrpPointCloud,
		double vCongruentDist
		);
}