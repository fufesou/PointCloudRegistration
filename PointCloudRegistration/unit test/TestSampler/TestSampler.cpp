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
#include "Registration.h"
#include "ICPConstGlobleValue.h"
#include "TestUnitity.h"
#include "PLYLoad.h"
#include "BaseSampler.h"
#include "PointCloudSubset.h"

using namespace hiveRegistration;


BOOST_AUTO_TEST_SUITE(IterativeClosestPoint2)

BOOST_AUTO_TEST_CASE(FineRegistration2)
{
	PTR_CONTROL_PARAMS->parseConfig();

	hivePointCloud::CPointCloud TestPointCloud;
	const std::string StrKeyPointCloudID = boost::to_upper_copy(std::string("TestSamplerPointCloudID"));
	std::string StrPointCloudID;

	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(StrKeyPointCloudID, StrPointCloudID);
	_ASSERT(DumpRes);

	CPLYLoad PlyData(StrPointCloudID.c_str());
	TestPointCloud.setNumPoint(PlyData.getNumPoints());
	TestPointCloud.setPos(PlyData.getPostion(), PlyData.getNumPoints());
	if (PlyData.getNormal())
	{
		TestPointCloud.setNormal(PlyData.getNormal(), PlyData.getNumPoints());
	}

	const std::string SamplerKeyStrID = boost::to_upper_copy(std::string("TestSampler"));
	std::string SamplerStrID;
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(SamplerKeyStrID, SamplerStrID);
	_ASSERT(DumpRes);

	CBaseSampler* pSampler = dynamic_cast<CBaseSampler*>(hiveCommon::CProductFactoryData::getInstance()->getOrCreateProduct(SamplerStrID));
	_ASSERT(pSampler);

	SamplePointSet SrcSamplePointSet = pSampler->samplePointCloud(TestPointCloud);

	export2Ply(".\\TestData\\output\\newSrcModel.ply", SrcSamplePointSet->getPointSet(), SrcSamplePointSet->getNormalSet());
}

BOOST_AUTO_TEST_SUITE_END()