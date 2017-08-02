#define BOOST_TEST_INCLUDED
#include <time.h>
#include <iostream>
#include <vector>
#include <boost/test/unit_test.hpp>
#include <osg/Vec3d>
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "PointCloud.h"
#include "ICPType.h"
#include "ICPMacros.h"
#include "D4PCS.h"
#include "ICPConstGlobleValue.h"
#include "TestUnitity.h"
#include "PLYLoad.h"

using namespace hiveRegistration;

void loadSourceAndTargetPointCloud(hivePointCloud::CPointCloud& voSrcPointCloud, hivePointCloud::CPointCloud& voTgtPointCloud);

BOOST_AUTO_TEST_SUITE(IterativeClosestPoint2)

BOOST_AUTO_TEST_CASE(FineRegistration2)
{
	PTR_CONTROL_PARAMS->parseConfig();

	hivePointCloud::CPointCloud SourcePointCloud, TargetPointCloud;
	loadSourceAndTargetPointCloud(SourcePointCloud, TargetPointCloud);

	Eigen::Matrix3d DesignedRotation;
	ColVector3 DesignedTranslation;
	initRotationAndTranslation(DesignedRotation, DesignedTranslation);

	TargetPointCloud.transform(DesignedRotation.data(), DesignedTranslation.data());

	Eigen::Matrix3d ComputedRotation = g_InitRotation;
	ColVector3 ComputedTranslation = g_InitTranslation;

	CD4PCS FourPCSObj;
	bool FitRes = FourPCSObj.coarseFit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);

	if (FitRes)
	{
		SourcePointCloud.transform(ComputedRotation.data(), ComputedTranslation.data());
		export2Ply(".\\TestData\\output\\newSrcModel.ply", SourcePointCloud);
	}
	else
	{
		std::cout << " not converged! " << std::endl;
	}
}

BOOST_AUTO_TEST_SUITE_END()


//*********************************************************************************
//FUNCTION:
void loadSourceAndTargetPointCloud(hivePointCloud::CPointCloud& voSrcPointCloud, hivePointCloud::CPointCloud& voTgtPointCloud)
{
	const std::string SrcStrKeyFileID = boost::to_upper_copy(std::string("TestFineRegSrcID"));
	const std::string TgtStrKeyFileID = boost::to_upper_copy(std::string("TestFineRegTgtID"));
	std::string SrcStrFileID;
	std::string TgtStrFileID;

	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(SrcStrKeyFileID, SrcStrFileID);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(TgtStrKeyFileID, TgtStrFileID);

	CPLYLoad SrcPlyData(SrcStrFileID.c_str());
	CPLYLoad TgtPlyData(TgtStrFileID.c_str());

	voSrcPointCloud.setNumPoint(SrcPlyData.getNumPoints());
	voTgtPointCloud.setNumPoint(TgtPlyData.getNumPoints());
	voSrcPointCloud.setPos(SrcPlyData.getPostion(), SrcPlyData.getNumPoints());
	voTgtPointCloud.setPos(TgtPlyData.getPostion(), TgtPlyData.getNumPoints());
	voSrcPointCloud.setNormal(SrcPlyData.getNormal(), SrcPlyData.getNumPoints());
	voTgtPointCloud.setNormal(TgtPlyData.getNormal(), TgtPlyData.getNumPoints());
}