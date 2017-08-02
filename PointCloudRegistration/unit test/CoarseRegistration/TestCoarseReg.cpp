#define BOOST_TEST_INCLUDE
#include <time.h>
#include <iostream>
#include <fstream>
#include <boost/test/unit_test.hpp>
#include "ProductFactoryData.h"
#include "PointCloud.h"
#include "ICPType.h"
#include "ICPMacros.h"
#include "ControlParameters.h"
#include "IterativeFit.h"
#include "TransformationEstimationSVD2.h"
#include "TestUnitity.h"
#include "Registration.h"

using namespace hiveRegistration;

void loadSourceAndTargetPointCloud(const Eigen::Matrix3d& vRot, const ColVector3& vTra, 
								   hivePointCloud::CPointCloud& voSrcPointCloud, hivePointCloud::CPointCloud& voTgtPointCloud);

BOOST_AUTO_TEST_SUITE(TestCoarseReg)

BOOST_AUTO_TEST_CASE(CoarseReg_LWQ)
{
	PTR_CONTROL_PARAMS->parseConfig();

	Eigen::Matrix3d DesignedRotation;
	ColVector3 DesignedTranslation;
	initRotationAndTranslation(DesignedRotation, DesignedTranslation);

	hivePointCloud::CPointCloud SourcePointCloud, TargetPointCloud;
	loadSourceAndTargetPointCloud(DesignedRotation, DesignedTranslation, SourcePointCloud, TargetPointCloud);

	//send2File(TargetPointCloud, "newTeeth.ply");

	CRegistartion RegApp;
	Eigen::Matrix3d ComputedRotation = g_InitRotation;
	ColVector3 ComputedTranslation = g_InitTranslation;

	boost::tuple<Eigen::Matrix3d, ColVector3, std::string> RotTranFitMsg;
	{
#ifndef _DEBUG
		std::ofstream TimeFile("TestData\\output\\time.txt");
		//boost::progress_timer PT(TimeFile);
		DWORD T1 =  GetTickCount();
#endif
		RotTranFitMsg = RegApp.coarseFit(SourcePointCloud, TargetPointCloud, ComputedRotation, ComputedTranslation);
#ifndef _DEBUG
		TimeFile << (GetTickCount() - T1) << std::endl;
#endif
	}

	std::ofstream RegMsgFile("TestData\\output\\RegMsg.txt");
	RegMsgFile << "ConvergenceType: \n" << RotTranFitMsg.get<2>() << std::endl;
	RegMsgFile << "Rotation Matrix: \n" << RotTranFitMsg.get<0>() << std::endl;
	RegMsgFile << "Translation Vector: \n" << RotTranFitMsg.get<1>() << std::endl;

	std::cout << "Rotation Matrix: \n" << RotTranFitMsg.get<0>() << std::endl;
	std::cout << "Rotation Matrix: \n" << RotTranFitMsg.get<1>() << std::endl;

	SourcePointCloud.transform(RotTranFitMsg.get<0>().data(), RotTranFitMsg.get<1>().data());
	export2Ply(".\\TestData\\output\\newSrcModel.ply", SourcePointCloud);
}

BOOST_AUTO_TEST_SUITE_END()

//*********************************************************************************
//FUNCTION:
void loadSourceAndTargetPointCloud(const Eigen::Matrix3d& vRot, const ColVector3& vTra, 
								   hivePointCloud::CPointCloud& voSrcPointCloud, hivePointCloud::CPointCloud& voTgtPointCloud)
{
	const std::string SrcStrKeyFileID = boost::to_upper_copy(std::string("TestCoarseRegSrcID"));
	const std::string TgtStrKeyFileID = boost::to_upper_copy(std::string("TestCoarseRegTgtID"));
	std::string SrcStrFileID;
	std::string TgtStrFileID;

	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(SrcStrKeyFileID, SrcStrFileID);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(TgtStrKeyFileID, TgtStrFileID);

	//voSrcPointCloud.plyLoadNorm(SrcStrFileID);
	//voTgtPointCloud.plyLoadNorm(TgtStrFileID);
	voSrcPointCloud.plyLoad(SrcStrFileID);
	voTgtPointCloud.plyLoad(TgtStrFileID);
	voTgtPointCloud.transform(vRot.data(), vTra.data());
	
}