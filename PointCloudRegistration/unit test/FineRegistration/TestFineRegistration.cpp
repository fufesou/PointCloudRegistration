#define BOOST_TEST_INCLUDED
#include <time.h>
#include <iostream>
#include <vector>
#include <boost/test/unit_test.hpp>
#include <osg/Vec3d>
#include <Windows.h>
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "PointCloud.h"
#include "ICPType.h"
#include "ICPMacros.h"
#include "Registration.h"
#include "IterativeFit.h"
#include "ICPConstGlobleValue.h"
#include "TransformationEstimationSVD2.h"
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


	std::string RegMethod;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue("REGSIG", RegMethod);
	_ASSERT(DumpRes);

	CRegistartion RegApp;
	Eigen::Matrix3d ComputedRotation = g_InitRotation;
	ColVector3 ComputedTranslation = g_InitTranslation;

	boost::tuple<Eigen::Matrix3d, ColVector3, std::string> RotTranFitMsg;

#ifndef _DEBUG
	std::ofstream CostFile("TestData\\output\\cost.txt");
	DWORD TimeBegin = GetTickCount();
#endif

	if (RegMethod == "COARSE")
	{
		RotTranFitMsg = RegApp.coarseFit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);
	}
	else
	{
		RotTranFitMsg = RegApp.fineFit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);
	}

#ifndef _DEBUG
	DWORD TimeCost = GetTickCount() - TimeBegin;
	std::cout << TimeCost << std::endl;
	CostFile << TimeCost << std::endl;
#endif

	const std::string ResMsgFileNameKey = boost::to_upper_copy(std::string("ResMsgFileName"));

	std::string ResMsgFileName;
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(ResMsgFileNameKey, ResMsgFileName);
	_ASSERT(DumpRes);

	std::ofstream RegMsgFile(ResMsgFileName.c_str());
	RegMsgFile << "ConvergenceType: \n" << RotTranFitMsg.get<2>() << std::endl;
	RegMsgFile << "Rotation Matrix: \n" << RotTranFitMsg.get<0>() << std::endl;
	RegMsgFile << "Translation Vector: \n" << RotTranFitMsg.get<1>() << std::endl;

	SourcePointCloud.transform(RotTranFitMsg.get<0>().data(), RotTranFitMsg.get<1>().data());

	std::cout << "Rotation Matrix: \n" << RotTranFitMsg.get<0>() << std::endl;
	std::cout << "Rotation Matrix: \n" << RotTranFitMsg.get<1>() << std::endl;

	const std::string SaveFileStrIDKey = boost::to_upper_copy(std::string("SaveFileName"));
	std::string SaveFileStrID;

	DumpRes = PTR_CONTROL_PARAMS->dumpValue(SaveFileStrIDKey, SaveFileStrID);
	_ASSERT(DumpRes);
	export2Ply(SaveFileStrID.c_str(), SourcePointCloud);
}

BOOST_AUTO_TEST_SUITE_END()


//*********************************************************************************
//FUNCTION:
void loadSourceAndTargetPointCloud(hivePointCloud::CPointCloud& voSrcPointCloud, hivePointCloud::CPointCloud& voTgtPointCloud)
{
	std::string RegMethod;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue("REGSIG", RegMethod);
	_ASSERT(DumpRes);

	if (RegMethod == "COARSE")
	{
		const std::string SrcStrKeyFileID = boost::to_upper_copy(std::string("TestCoarseRegSrcID"));
		const std::string TgtStrKeyFileID = boost::to_upper_copy(std::string("TestCoarseRegTgtID"));
		std::string SrcStrFileID;
		std::string TgtStrFileID;

		DumpRes = PTR_CONTROL_PARAMS->dumpValue(SrcStrKeyFileID, SrcStrFileID);
		_ASSERT(DumpRes);
		DumpRes = PTR_CONTROL_PARAMS->dumpValue(TgtStrKeyFileID, TgtStrFileID);

		voSrcPointCloud.plyLoad(SrcStrFileID);
		voTgtPointCloud.plyLoad(TgtStrFileID);
	}
	else
	{
		const std::string SrcStrKeyFileID = boost::to_upper_copy(std::string("TestFineRegSrcID"));
		const std::string TgtStrKeyFileID = boost::to_upper_copy(std::string("TestFineRegTgtID"));
		std::string SrcStrFileID;
		std::string TgtStrFileID;

		DumpRes = PTR_CONTROL_PARAMS->dumpValue(SrcStrKeyFileID, SrcStrFileID);
		_ASSERT(DumpRes);
		DumpRes = PTR_CONTROL_PARAMS->dumpValue(TgtStrKeyFileID, TgtStrFileID);
		_ASSERT(DumpRes);

		CPLYLoad SrcPlyData(SrcStrFileID.c_str());
		CPLYLoad TgtPlyData(TgtStrFileID.c_str());

		voSrcPointCloud.setNumPoint(SrcPlyData.getNumPoints());
		voTgtPointCloud.setNumPoint(TgtPlyData.getNumPoints());
		voSrcPointCloud.setPos(SrcPlyData.getPostion(), SrcPlyData.getNumPoints());
		voTgtPointCloud.setPos(TgtPlyData.getPostion(), TgtPlyData.getNumPoints());
		voSrcPointCloud.setNormal(SrcPlyData.getNormal(), SrcPlyData.getNumPoints());
		voTgtPointCloud.setNormal(TgtPlyData.getNormal(), TgtPlyData.getNumPoints());
	}
}