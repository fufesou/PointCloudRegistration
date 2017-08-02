#include "TestUnitity.h"
#include <fstream>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <Eigen/LU>
#include "EventLoggerInterface.h"
#include "ProductFactoryData.h"
#include "PointCloud.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "PointCloudSubset.h"

void hiveRegistration::saveCorPairSet(const CorrespondencePairSet& vCorPairSet, const std::string& vFileName, const Eigen::Matrix3d& vRot, const ColVector3& vTra)
{
	std::ofstream OutFile(vFileName.c_str(), std::ios::out | std::ios::trunc);
	_ASSERT(OutFile.is_open());

	const VecColVector3& SourcePointSet = (vCorPairSet.first)->getPointSet();
	const VecColVector3& TargetPointSet = (vCorPairSet.second)->getPointSet();
	const VecColVector3& SourceNormalSet = (vCorPairSet.first)->getNormalSet();
	const VecColVector3& TargetNormalSet = (vCorPairSet.second)->getNormalSet();

	static unsigned int ClrIdx = 0;

	// _ASSERT((SourcePointSet.size() == TargetPointSet.size()) && (SourceNormalSet.size() == TargetPointSet.size()) && (SourcePointSet.size() == SourceNormalSet.size()));

	OutFile << "Size: " << SourcePointSet.size() << std::endl;

	Eigen::Matrix3d CurRot = vRot;
	ColVector3 CurTra = vTra;

	for (int i=0; i<SourcePointSet.size(); ++i)
	{
		ColVector3 OriSrcPnt = CurRot.inverse() * (SourcePointSet[i] - CurTra);

		unsigned int R = ClrIdx & 0xff;
		unsigned int G = (ClrIdx & 0xff00) >> 8;
		unsigned int B = (ClrIdx & 0xff0000) >> 16;
		unsigned int Alpha = 255;

		OutFile << boost::str(boost::format("%f %f %f %u %u %u %u ") %OriSrcPnt(0, 0) %OriSrcPnt(1, 0) %OriSrcPnt(2, 0) %R %G %B %Alpha) << std::endl;
		OutFile << boost::str(boost::format("%f %f %f %u %u %u %u ") %TargetPointSet[i](0, 0) %TargetPointSet[i](1, 0) %TargetPointSet[i](2, 0) %R %G %B %Alpha) << std::endl;

		ClrIdx += 10;
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
char * hiveRegistration::textFileRead(const char *vFileName)
{
	FILE *pFile = NULL;
	char *pContent = NULL;

	int Count=0;

	if (vFileName != NULL) 
	{
		pFile = fopen(vFileName,"rt");
		_ASSERT(pFile);

		if (pFile != NULL) 
		{
			fseek(pFile, 0, SEEK_END);
			Count = ftell(pFile);
			rewind(pFile);

			if (Count > 0) 
			{
				pContent = (char *)malloc(sizeof(char) * (Count+1));
				Count = fread(pContent,sizeof(char),Count,pFile);
				pContent[Count] = '\0';
			}
			fclose(pFile);
		}
	}
	return pContent;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::initPointAndProperties(const char *vFilename, const int vNumProperty, std::vector<osg::Vec3d>& voPointSet, std::vector<std::vector<double> >& voProperties)
{
	char *pFile = NULL;//нд╪Ч
	char *pTmp	= NULL;//tmp
	int SizeNum = 0;

	voProperties.resize(vNumProperty);

	pFile = textFileRead(vFilename);
	_ASSERT(pFile);
	char *pMoveText = pFile;

	osg::Vec3d Point;

	while (*pMoveText != '\0')
	{
		pTmp = strchr(pMoveText, ' ');
		*pTmp = '\0';
		Point[0] = atof(pMoveText);
		pMoveText = pTmp+1;

		pTmp = strchr(pMoveText, ' ');
		*pTmp = '\0';
		Point[1] = atof(pMoveText);
		pMoveText = pTmp+1;

		pTmp = strchr(pMoveText, ' ');
		*pTmp = '\0';
		Point[2] = atof(pMoveText);
		pMoveText = pTmp+1;

		for (int i=0; i!=vNumProperty; ++i)
		{
			pTmp = strchr(pMoveText, ' ');
			*pTmp = '\0';
			(voProperties[i]).push_back(atof(pMoveText));
			pMoveText = pTmp+1;
		}

		voPointSet.push_back(Point);
	}

	free(pFile);
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::initRotationAndTranslation(Eigen::Matrix3d& voRot, ColVector3& voTra)
{
	const std::string KeyRotX = "ROTX";
	const std::string KeyRotY = "ROTY";
	const std::string KeyRotZ = "ROTZ";
	const std::string KeyTraX = "TRAX";
	const std::string KeyTraY = "TRAY";
	const std::string KeyTraZ = "TRAZ";
	double AglRotX, AglRotY, AglRotZ;
	double TraX, TraY, TraZ;
	bool DumpRes = false;

	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyRotX, AglRotX);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyRotY, AglRotY);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyRotZ, AglRotZ);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyTraX, TraX);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyTraY, TraY);
	_ASSERT(DumpRes);
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(KeyTraZ, TraZ);
	_ASSERT(DumpRes);

	voRot <<
		std::cos(AglRotY)*std::cos(AglRotZ)                                                        , std::cos(AglRotY) * std::sin(AglRotZ)                                                      , -std::sin(AglRotY)                 ,
		std::sin(AglRotX)*std::sin(AglRotY)*std::cos(AglRotZ) - std::cos(AglRotX)*std::sin(AglRotZ), std::sin(AglRotX)*std::sin(AglRotY)*std::sin(AglRotZ) + std::cos(AglRotX)*std::cos(AglRotZ), std::sin(AglRotX)*std::cos(AglRotY),
		std::cos(AglRotX)*std::sin(AglRotY)*std::cos(AglRotZ) + std::sin(AglRotX)*std::sin(AglRotZ), std::cos(AglRotX)*std::sin(AglRotY)*std::sin(AglRotZ) - std::sin(AglRotX)*std::cos(AglRotZ), std::cos(AglRotX)*std::cos(AglRotY);

	voTra << TraX, TraY, TraZ;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(std::ofstream& vOutFile, const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndices)
{
	for (std::vector<unsigned int>::const_iterator citr=vIndices.begin(); citr!=vIndices.end(); ++citr)
	{
		export2Ply(vOutFile, vPointCloud, *citr);
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(std::ofstream& vOutFile, const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIndex)
{
	const double* pPos = vPointCloud.getPosPointer();
	const double* pNorm = vPointCloud.getNormalPointer();
	unsigned int NumPoints = vPointCloud.getNumPoints();
	_ASSERT(vIndex < NumPoints);
	if (pNorm)
	{
		vOutFile << boost::str(boost::format("%f %f %f %f %f %f ") %pPos[vIndex*3] %pPos[vIndex*3+1] %pPos[vIndex*3+2] %pNorm[vIndex*3] %pNorm[vIndex*3+1] %pNorm[vIndex*3+2]) << std::endl;
	}
	else
	{
		vOutFile << boost::str(boost::format("%f %f %f ") %pPos[vIndex*3] %pPos[vIndex*3+1] %pPos[vIndex*3+2]) << std::endl;
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud, const std::vector<unsigned int>& vIndices)
{
	std::ofstream OutFile(vFileName);
	_ASSERT(OutFile.is_open());
	if (vPointCloud.getNormalPointer())
	{
		fillPlyHeader(OutFile, vIndices.size());
	}
	else
	{
		fillPlyHeaderWithoutNorm(OutFile, vIndices.size());
	}
	export2Ply(OutFile, vPointCloud, vIndices);
	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud, unsigned int vIndex)
{
	std::ofstream OutFile(vFileName);
	_ASSERT(OutFile.is_open());
	if (vPointCloud.getNormalPointer())
	{
		fillPlyHeader(OutFile, 1);
	}
	else
	{
		fillPlyHeaderWithoutNorm(OutFile, 1);
	}
	export2Ply(OutFile, vPointCloud, vIndex);
	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(const char* vFileName, const VecColVector3& vPointSet)
{
	std::ofstream OutFile(vFileName);
	_ASSERT(OutFile.is_open());
	fillPlyHeaderWithoutNorm(OutFile, vPointSet.size());

	for (VecColVector3::const_iterator citr=vPointSet.begin(); citr!=vPointSet.end(); ++citr)
	{
		OutFile << boost::str(boost::format("%f %f %f ") %(*citr)(0, 0) %(*citr)(1, 0) %(*citr)(2, 0))<< std::endl;
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(const char* vFileName, const VecColVector3& vPointSet, const VecColVector3& vNormSet)
{
	unsigned PointSetSize = vPointSet.size();
	unsigned NormSetSize = vNormSet.size();

	std::ofstream OutFile(vFileName);
	_ASSERT(OutFile.is_open());
	
	if (NormSetSize == PointSetSize)
	{
		fillPlyHeader(OutFile, PointSetSize);
		for (unsigned i=0; i<PointSetSize; ++i)
		{
			OutFile << boost::str(boost::format("%f %f %f %f %f %f ") %vPointSet[i](0, 0) %vPointSet[i](1, 0) %vPointSet[i](2, 0) %vNormSet[i](0, 0) %vNormSet[i](1, 0) %vNormSet[i](2, 0))<< std::endl;
		}
	}
	else
	{
		hiveEventLogger::hiveOutputEvent("no normal data!");
		fillPlyHeaderWithoutNorm(OutFile, PointSetSize);
		for (VecColVector3::const_iterator citr=vPointSet.begin(); citr!=vPointSet.end(); ++citr)
		{
			OutFile << boost::str(boost::format("%f %f %f ") %(*citr)(0, 0) %(*citr)(1, 0) %(*citr)(2, 0))<< std::endl;
		}
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::export2Ply(const char* vFileName, const hivePointCloud::CPointCloud& vPointCloud)
{
	std::ofstream OutFile(vFileName);
	const double* pPos    = vPointCloud.getPosPointer();
	const double* pNormal = vPointCloud.getNormalPointer();

	if (pNormal)
	{
		fillPlyHeader(OutFile, vPointCloud.getNumPoints());
		for (unsigned int i=0; i<vPointCloud.getNumPoints(); ++i)
		{
			OutFile<<*(pPos+3*i)<<" "<<*(pPos+3*i+1)<<" "<<*(pPos+3*i+2)<<" " << *(pNormal+3*i)<<" "<<*(pNormal+3*i+1)<<" "<<*(pNormal+3*i+2)<<std::endl;
		}
	}
	else
	{
		fillPlyHeaderWithoutNorm(OutFile, vPointCloud.getNumPoints());
		for (unsigned int i=0; i<vPointCloud.getNumPoints(); ++i)
		{
			OutFile<<*(pPos+3*i)<<" "<<*(pPos+3*i+1)<<" "<<*(pPos+3*i+2)<<" ";
		}
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::fillPlyHeader(std::ofstream& vOutFile, unsigned int vNumPoints)
{
	vOutFile << 
		"ply\n"
		"format ascii 1.0\n"
		"comment VCGLIB generated\n" << boost::str(boost::format("element vertex %d\n") %vNumPoints) <<
		"property float x\n"
		"property float y\n"
		"property float z\n"
		"property float nx\n"
		"property float ny\n"
		"property float nz\n"
		"element face 0\n"
		"property list uchar int vertex_indices\n"
		"end_header"
		<< std::endl;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::fillPlyHeaderWithoutNorm(std::ofstream& vOutFile, unsigned int vNumPoints)
{
	vOutFile << 
		"ply\n"
		"format ascii 1.0\n"
		"comment VCGLIB generated\n" << boost::str(boost::format("element vertex %d\n") %vNumPoints) <<
		"property float x\n"
		"property float y\n"
		"property float z\n"
		"element face 0\n"
		"property list uchar int vertex_indices\n"
		"end_header"
		<< std::endl;
}

//*********************************************************************************
//FUNCTION:
double hiveRegistration::computeCurrentFitRatio(const std::vector<std::pair<unsigned, std::vector<unsigned> > >& vInitPairedPoints, const Eigen::Matrix3d& vR, Eigen::Vector3d& vT, const hivePointCloud::CPointCloud& vIdxPointCloud, const hivePointCloud::CPointCloud& vGrpPointCloud, double vCongruentDist)
{
	const double* pLoopPos = vIdxPointCloud.getPosPointer();
	const double* pMatchPos = vGrpPointCloud.getPosPointer();
	double FitRatio = 0.0;

	for (std::vector<std::pair<unsigned, std::vector<unsigned> > >::const_iterator CItr=vInitPairedPoints.begin(); CItr!=vInitPairedPoints.end(); ++CItr)
	{
		unsigned LoopIdx = CItr->first;
		Eigen::Vector3d NewPnt = vR * Eigen::Vector3d(pLoopPos[LoopIdx*3], pLoopPos[LoopIdx*3 + 1], pLoopPos[LoopIdx*3 + 2]) + vT;
		for (std::vector<unsigned>::const_iterator NestCItr=CItr->second.begin(); NestCItr!=CItr->second.end(); ++NestCItr)
		{
			unsigned MatchIdx = *NestCItr;
			double TmpDist = (NewPnt - Eigen::Vector3d(pMatchPos[MatchIdx*3], pMatchPos[MatchIdx*3 + 1], pMatchPos[MatchIdx*3 + 2])).squaredNorm();
			if (TmpDist < vCongruentDist)
			{
				FitRatio += 1.0;
				break;
			}
		}
	}

	_ASSERT(vInitPairedPoints.size() != 0);

	return FitRatio / vInitPairedPoints.size();
}
