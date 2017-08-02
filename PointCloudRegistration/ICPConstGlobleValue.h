#pragma once
#include <climits>
#include <string>
#include "ICPType.h"

namespace hiveRegistration
{
	const std::string g_CoarseRegistrationPrefix = "C_";
	const std::string g_FineRegistrationPrefix = "F_";
	const std::string g_KeySampler = "SAMPLER";
	const std::string g_KeyCorrespondenceEstimation("CORRESPONDENCEESTIMATION");
	const std::string g_KeyCorrespondenceRejection("CORRESPONDENCEREJECTION");
	const std::string g_KeyTransformationEstimation("TRANSFORMATIONESTIMATION");
	const std::string g_CoarseSampler = g_CoarseRegistrationPrefix + g_KeySampler;
	const std::string g_CoarseCorrespondenceEstimation = g_CoarseRegistrationPrefix + g_KeyCorrespondenceEstimation;
	const std::string g_CoarseCorrespondenceRejection = g_CoarseRegistrationPrefix + g_KeyCorrespondenceRejection;
	const std::string g_CoarseTransformationEstimation = g_CoarseRegistrationPrefix + g_KeyTransformationEstimation;
	const std::string g_FineSampler = g_FineRegistrationPrefix + g_KeySampler;
	const std::string g_FineCorrespondenceEstimation = g_FineRegistrationPrefix + g_KeyCorrespondenceEstimation;
	const std::string g_FineCorrespondenceRejection = g_FineRegistrationPrefix + g_KeyCorrespondenceRejection;
	const std::string g_FineTransformationEstimation = g_FineRegistrationPrefix + g_KeyTransformationEstimation;

	const std::string g_ConfigFile = "RegCfg.txt";
	const Eigen::Matrix3d    g_InitRotation = Eigen::Matrix3d().Identity();
	const ColVector3 g_InitTranslation = ColVector3().setZero();
	const ColVector3 g_InvalidColVec3 =  ColVector3(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
}