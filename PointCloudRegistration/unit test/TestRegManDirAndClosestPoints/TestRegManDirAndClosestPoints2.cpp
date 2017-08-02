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
#include "ICPType.h"

using namespace hiveRegistration;

void initTestRegMainFileName(std::string& voSrcVertFile, std::string& voSrcNormFile, std::string& voTgtVertFile, std::string& voTgtNormFile);
void setSourcePointCloud(const char *vVertFile, const char *vNormFile, hivePointCloud::CPointCloud& vioSourcePointCloud);
void setTargetPointCloud(const char *vVertFile, const char *vNormFile, const Eigen::Matrix3d& vR, const ColVector3& vT, hivePointCloud::CPointCloud& vioTargetPointCloud);

BOOST_AUTO_TEST_SUITE(CoarseRegistration)

BOOST_AUTO_TEST_CASE(RegMainDirClosestPoint2)
{
	PTR_CONTROL_PARAMS->parseConfig(g_ConfigFile);

	std::string SrcVertFile, SrcNormFile, TgtVertFile, TgtNormFile;
	initTestRegMainFileName(SrcVertFile, SrcNormFile, TgtVertFile, TgtNormFile);

	Eigen::Matrix3d DesignedRotation;
	ColVector3 DesignedTranslation;
	initRotationAndTranslation(DesignedRotation, DesignedTranslation);

	hivePointCloud::CPointCloud SourcePointCloud;
	hivePointCloud::CPointCloud TargetPointCloud;
	setSourcePointCloud(SrcVertFile.c_str(), SrcNormFile.c_str(), SourcePointCloud);
	setTargetPointCloud(TgtVertFile.c_str(), TgtNormFile.c_str(), DesignedRotation, DesignedTranslation, TargetPointCloud);

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
		
		boost::tuple<Eigen::Matrix3d, ColVector3, std::string> RotTranFitMsg = ICPApp.fit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);

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
void initTestRegMainFileName(std::string& voSrcVertFile, std::string& voSrcNormFile, std::string& voTgtVertFile, std::string& voTgtNormFile)
{
	std::string StrKeySrcVert = "TestRegManDirAndClosestPointsFile2.SrcVertFile";
	std::string StrKeySrcNorm = "TestRegManDirAndClosestPointsFile2.SrcNormFile";
	std::string StrKeyTgtVert = "TestRegManDirAndClosestPointsFile2.TgtVertFile";
	std::string StrKeyTgtNorm = "TestRegManDirAndClosestPointsFile2.TgtNormFile";
	boost::to_upper(StrKeySrcVert);
	boost::to_upper(StrKeySrcNorm);
	boost::to_upper(StrKeyTgtVert);
	boost::to_upper(StrKeyTgtNorm);

	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeySrcVert, voSrcVertFile);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeySrcNorm, voSrcNormFile);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeyTgtVert, voTgtVertFile);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeyTgtNorm, voTgtNormFile);
	_ASSERT(DumpRes);
}

//*********************************************************************************
//FUNCTION:
void setSourcePointCloud(const char *vVertFile, const char *vNormFile, hivePointCloud::CPointCloud& vioSourcePointCloud)
{
	std::vector<osg::Vec3d> VertSet;
	std::vector<osg::Vec3d> NormSet;
	std::vector<std::vector<double> > PropertySet;
	initPointAndProperties(vVertFile, 0, VertSet, PropertySet);
	initPointAndProperties(vNormFile, 0, NormSet, PropertySet);

	vioSourcePointCloud.initPointCloud(VertSet);
	vioSourcePointCloud.setNormal(NormSet);
}

//*********************************************************************************
//FUNCTION:
void setTargetPointCloud(const char *vVertFile, const char *vNormFile, const Eigen::Matrix3d& vR, const ColVector3& vT, hivePointCloud::CPointCloud& vioTargetPointCloud)
{
	std::vector<osg::Vec3d> VertSet;
	std::vector<osg::Vec3d> NormSet;
	std::vector<std::vector<double> > PropertySet;
	initPointAndProperties(vVertFile, 0, VertSet, PropertySet);
	initPointAndProperties(vNormFile, 0, NormSet, PropertySet);

	for (int i=0; i<VertSet.size(); ++i)
	{
		VertSet[i] = osg::Vec3d(
			vR(0, 0)*VertSet[i].x() + vR(0, 1)*VertSet[i].y() + vR(0, 2)*VertSet[i].z() + vT(0, 0),
			vR(1, 0)*VertSet[i].x() + vR(1, 1)*VertSet[i].y() + vR(1, 2)*VertSet[i].z() + vT(1, 0),
			vR(2, 0)*VertSet[i].x() + vR(2, 1)*VertSet[i].y() + vR(2, 2)*VertSet[i].z() + vT(2, 0) );

		NormSet[i].x() = vR(0, 0)*NormSet[i].x() + vR(0, 1)*NormSet[i].y() + vR(0, 2)*NormSet[i].z();
		NormSet[i].y() = vR(1, 0)*NormSet[i].x() + vR(1, 1)*NormSet[i].y() + vR(1, 2)*NormSet[i].z();
		NormSet[i].z() = vR(2, 0)*NormSet[i].x() + vR(2, 1)*NormSet[i].y() + vR(2, 2)*NormSet[i].z();
	}

	vioTargetPointCloud.initPointCloud(VertSet);
	vioTargetPointCloud.setNormal(NormSet);
}