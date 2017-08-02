#define BOOST_TEST_INCLUDED
#include <math.h>
#include <time.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/test/unit_test.hpp>
#include "ProductFactoryData.h"
#include "ICPType.h"
#include "ICPMacros.h"
#include "ICPConstGlobleValue.h"
#include "RegMainDir.h"
#include "PointCloud.h"
#include "ControlParameters.h"
#include "TestUnitity.h"
#include "IterativeFit.h"
#include "SamplerClosestPoints.h"
#include "CorrespondenceEstimationClosestPoints.h"
#include "CorrespondenceRejectionClosestPoints.h"
#include "TransformationEstimationSVD2.h"

using namespace hiveRegistration;

void initTestRegMainFileName(std::string& voFileName);
void setPointCloud(const char *vFileName, const Eigen::Matrix3d& vR, const ColVector3& vT, 
						 hivePointCloud::CPointCloud& vioSourcePointCloud, hivePointCloud::CPointCloud& vioTargetPointCloud);

BOOST_AUTO_TEST_SUITE(CoarseRegistration)

BOOST_AUTO_TEST_CASE(RegMainDir)
{
	PTR_CONTROL_PARAMS->parseConfig(g_ConfigFile);

	std::string TestRegMainFileName;
	initTestRegMainFileName(TestRegMainFileName);

	Eigen::Matrix3d DesignedRotation;
	ColVector3 DesignedTranslation;
	initRotationAndTranslation(DesignedRotation, DesignedTranslation);

	hivePointCloud::CPointCloud SourcePointCloud;
	hivePointCloud::CPointCloud TargetPointCloud;
	setPointCloud(TestRegMainFileName.c_str(), DesignedRotation, DesignedTranslation, SourcePointCloud, TargetPointCloud);

	Eigen::Matrix3d ResRot = g_InitRotation;
	ColVector3 ResTra = g_InitTranslation;
	double CoincidentCoeff = 0.0;

	CRegMainDirect RegMainDirApp;
	bool IsPointCloudMatched = RegMainDirApp.fit(SourcePointCloud, TargetPointCloud, ResRot, ResTra, CoincidentCoeff);

	if (IsPointCloudMatched)
	{
		std::cout << "RegMainDir Fit OK" << std::endl;
		std::cout << "Fit Rotation: \n" << ResRot << std::endl;
		std::cout << "Fit Translation: \n" << ResTra << std::endl;
		std::cout << "Coincident Coefficient: \n" << CoincidentCoeff << std::endl;

		CIterativeFit ICPApp;

		ICPApp.setFitClasses(
			CSamplerClosestPoints::getClassSig(), 
			CCorrespondenceEstimationClosestPoints::getClassSig(), 
			CCorrespondenceRejectionClosestPoints::getClassSig(),
			CTransformationEstimationSVD2::getClassSig());
		
		RotTraFitMsg RotTranFitMsg = ICPApp.fit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);

		std::cout << "ConvergenceType: \n" << RotTranFitMsg.get<2>() << std::endl;
		std::cout << "Rotation Matrix: \n" << RotTranFitMsg.get<0>() << std::endl;
		std::cout << "Translation Vector: \n" << RotTranFitMsg.get<1>() << std::endl;
	}
	else
	{
		std::cout << "RegMainDir Not Fit" << std::endl;
		std::cout << "Fit Rotation: \n" << ResRot << std::endl;
		std::cout << "Fit Translation: \n" << ResTra << std::endl;
		std::cout << "Coincident Coefficient: \n" << CoincidentCoeff << std::endl;
	}
}

BOOST_AUTO_TEST_SUITE_END()

//*********************************************************************************
//FUNCTION:
void setPointCloud(const char *vFileName, const Eigen::Matrix3d& vR, const ColVector3& vT, 
						 hivePointCloud::CPointCloud& vioSourcePointCloud, hivePointCloud::CPointCloud& vioTargetPointCloud)
{
	std::vector<osg::Vec3d> SourcePointSet;
	std::vector<osg::Vec3d> TargetPointSet;
	std::vector<std::vector<double> > PropertySet;
	initPointAndProperties(vFileName, 3, SourcePointSet, PropertySet);

	const std::vector<double>& OriginNormalSetX = PropertySet[0];
	const std::vector<double>& OriginNormalSetY = PropertySet[1];
	const std::vector<double>& OriginNormalSetZ = PropertySet[2];

	std::vector<double> TransformedNormalSetX;
	std::vector<double> TransformedNormalSetY;
	std::vector<double> TransformedNormalSetZ;

	for (int i=0; i<SourcePointSet.size(); ++i)
	{
		TargetPointSet.push_back( osg::Vec3d(
			vR(0, 0)*SourcePointSet[i].x() + vR(0, 1)*SourcePointSet[i].y() + vR(0, 2)*SourcePointSet[i].z() + vT(0, 0) + ICP_RAND(-1.15, 1.15),
			vR(1, 0)*SourcePointSet[i].x() + vR(1, 1)*SourcePointSet[i].y() + vR(1, 2)*SourcePointSet[i].z() + vT(1, 0) + ICP_RAND(-1.15, 1.15),
			vR(2, 0)*SourcePointSet[i].x() + vR(2, 1)*SourcePointSet[i].y() + vR(2, 2)*SourcePointSet[i].z() + vT(2, 0) + ICP_RAND(-1.15, 1.15) ));

		TransformedNormalSetX.push_back(vR(0, 0)*OriginNormalSetX[i] + vR(0, 1)*OriginNormalSetY[i] + vR(0, 2)*OriginNormalSetZ[i]);
		TransformedNormalSetY.push_back(vR(1, 0)*OriginNormalSetX[i] + vR(1, 1)*OriginNormalSetY[i] + vR(1, 2)*OriginNormalSetZ[i]);
		TransformedNormalSetZ.push_back(vR(2, 0)*OriginNormalSetX[i] + vR(2, 1)*OriginNormalSetY[i] + vR(2, 2)*OriginNormalSetZ[i]);
	}

	vioSourcePointCloud.initPointCloud(SourcePointSet);
	vioTargetPointCloud.initPointCloud(TargetPointSet);
	vioSourcePointCloud.setNormal(OriginNormalSetX, OriginNormalSetY, OriginNormalSetZ);
	vioTargetPointCloud.setNormal(TransformedNormalSetX, TransformedNormalSetY, TransformedNormalSetZ);
}

//*********************************************************************************
//FUNCTION:
void initTestRegMainFileName(std::string& voFileName)
{
	std::string StrKeyTestRegMainFile = "TestRegManDirAndClosestPointsFile";
	boost::to_upper(StrKeyTestRegMainFile);

	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeyTestRegMainFile, voFileName);
	_ASSERT(DumpRes);
}