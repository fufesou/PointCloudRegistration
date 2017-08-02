#include "PointCloud.h"
#include <cstring>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <osg/Geometry>
#include <osg/Geode>
#include <osgDB/WriteFile>
#include "EventLoggerInterface.h"
#include "HiveCommonMicro.h"
#include "PLYLoad.h"

using namespace hivePointCloud;


hivePointCloud::CPointCloud::CPointCloud(const CPointCloud& vOther)
{
	m_AvgRadius	   = 0;
	m_NumPoints	   = 0;
	m_NumNeighbors = 0;
	m_pRadius	   = NULL;
	m_pPos		   = NULL;
	m_pNormal	   = NULL;
	m_pNeighbors   = NULL;
	m_pGssCurvature  = NULL;
	m_pMeanCurvature = NULL;
	m_pK1Curvature   = NULL;
	m_pK2Curvature   = NULL;
	m_Dimension	     = 0;

	__doCopy(vOther);
}

CPointCloud::CPointCloud(unsigned int vDimension)
{
	m_AvgRadius	   = 0;
	m_NumPoints	   = 0;
	m_NumNeighbors = 0;
	m_pRadius	   = NULL;
	m_pPos		   = NULL;
	m_pNormal	   = NULL;
	m_pNeighbors   = NULL;
	m_pGssCurvature  = NULL;
	m_pMeanCurvature = NULL;
	m_pK1Curvature   = NULL;
	m_pK2Curvature   = NULL;
	m_Dimension	     = vDimension;
}

CPointCloud::~CPointCloud(void)
{
	delete[] m_pPos;
	delete[] m_pNormal;
	delete[] m_pRadius;
	delete[] m_pGssCurvature;
	delete[] m_pMeanCurvature;
	delete[] m_pK1Curvature;
	delete[] m_pK2Curvature;
	delete[] m_pNeighbors;
}

//****************************************************************************************************************
//FUNCTION:
const std::vector<unsigned int>* CPointCloud::getNeighborInfo() const
{
	return m_pNeighbors;
}

//****************************************************************************************************************
//FUNCTION:
unsigned int CPointCloud::getNumPoints() const
{
	return m_NumPoints;
}

//****************************************************************************************************************
//FUNCTION:
unsigned int CPointCloud::getNumNeighbors() const
{
	return m_NumNeighbors;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getPosPointer() const
{
	return m_pPos;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getNormalPointer() const
{
	return m_pNormal;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getGssCurvaturePointer() const
{
	return m_pGssCurvature;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getMeanCurvaturePointer() const
{
	return m_pMeanCurvature;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getK1CurvaturePointer() const
{
	return m_pK1Curvature;
}

//****************************************************************************************************************
//FUNCTION:
const double* CPointCloud::getK2CurvaturePointer() const
{
	return m_pK2Curvature;
}

//****************************************************************************************************************
//FUNCTION:
std::vector<unsigned int>* CPointCloud::getKNNPointsOfGivenPoint(unsigned int vGivenIndex) const
{
	return m_pNeighbors + vGivenIndex;
}

//****************************************************************************************************************
//FUNCTION:
double* CPointCloud::getNormalAt(unsigned int vIndex) const
{
	return m_pNormal + vIndex * m_Dimension;
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::plyLoad(const std::string& vFileID)
{
	CPLYLoad ply((vFileID+"Norm.ply").c_str(), (vFileID+"Gss.ply").c_str(), (vFileID+"Mean.ply").c_str(), (vFileID+"K1.ply").c_str(), (vFileID+"K2.ply").c_str());
	m_NumPoints      = ply.getNumPoints();
	m_pPos           = ply.getPostion();
	m_pNormal        = ply.getNormal();
	m_pGssCurvature  = ply.getGaussianCurvature();
	m_pMeanCurvature = ply.getMeanCurvature();
	m_pK1Curvature   = ply.getK1Curvature();
	m_pK2Curvature   = ply.getK2Curvature();

}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::initPointCloud(const std::vector<osg::Vec3d>& vPointSet)
{
	_ASSERT(!vPointSet.empty());

	__clear();
	m_NumPoints = vPointSet.size();
	m_pPos = new double[m_NumPoints*3];
	for (unsigned int i=0; i<m_NumPoints; i++)
	{
		m_pPos[3*i+0] = vPointSet[i][0];
		m_pPos[3*i+1] = vPointSet[i][1];
		m_pPos[3*i+2] = vPointSet[i][2];
	}
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::save2File(const std::string& vFileName)
{
	if ((!m_pPos) || (m_NumPoints == 0) || (vFileName.empty())) return;

	osg::Node *pNode = convert2OSGNode();
	_HIVE_SIMPLE_IF_ELSE(pNode, osgDB::writeNodeFile(*pNode, vFileName), hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, _BOOST_STR1("Fail to the point cloud to file [%1%].", vFileName)));
}

//****************************************************************************************************************
//FUNCTION: 
osg::Node* CPointCloud::convert2OSGNode()
{
	if ((!m_pPos) || (m_NumPoints == 0)) return NULL;

	osg::Geometry *pGeometry = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> pVertex = new osg::Vec3Array;
	for (unsigned int i=0; i<m_NumPoints; i++)
	{
		pVertex->push_back(osg::Vec3(m_pPos[3*i], m_pPos[3*i+1], m_pPos[3*i+2]));
	}
	pGeometry->setVertexArray(pVertex);

	if (m_pNormal)
	{
		osg::ref_ptr<osg::Vec3Array> pNormal = new osg::Vec3Array;
		for (unsigned int i=0; i<m_NumPoints; i++)
		{
			pNormal->push_back(osg::Vec3(m_pNormal[3*i], m_pNormal[3*i+1], m_pNormal[3*i+2]));
		}
		pGeometry->setNormalArray(pNormal);
		pGeometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	}

	pGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, m_NumPoints));

	osg::ref_ptr<osg::Geode> pGeode = new osg::Geode;
	pGeode->addDrawable(pGeometry);

	return pGeode;
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::__initNeighborsInfo()
{
	_ASSERT(m_NumPoints > 0);
	m_pNeighbors = new std::vector<unsigned int>[m_NumPoints];
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::__clear()
{
	if (m_pPos) SAFE_DELETE_ARRAY(m_pPos);
	if (m_pNormal) SAFE_DELETE_ARRAY(m_pNormal);
	if (m_pRadius) SAFE_DELETE_ARRAY(m_pRadius);
	if (m_pNeighbors) SAFE_DELETE_ARRAY(m_pNeighbors);
	if (m_pGssCurvature) SAFE_DELETE_ARRAY(m_pGssCurvature);
	if (m_pMeanCurvature) SAFE_DELETE_ARRAY(m_pMeanCurvature);
	if (m_pK1Curvature) SAFE_DELETE_ARRAY(m_pK1Curvature);
	if (m_pK2Curvature) SAFE_DELETE_ARRAY(m_pK2Curvature);
	_clearExtraAttributesV();
	m_NumPoints = 0;
}

//****************************************************************************************************************
//FUNCTION:
double* CPointCloud::__reverseNormal(unsigned int vIndex)
{
	double* pPointNormal = m_pNormal + vIndex * m_Dimension;
	(*pPointNormal)		 = -(*pPointNormal);
	(*(pPointNormal+1))	 = -(*(pPointNormal+1));
	(*(pPointNormal+2))  = -(*(pPointNormal+2));
	return pPointNormal;
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::__setNormalPointer(const double* vNormalPointer)
{
	m_pNormal = const_cast<double*>(vNormalPointer);
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::__setCurvaturePointer(const double* vCurvature)
{
	m_pGssCurvature = const_cast<double*>(vCurvature);
}

//****************************************************************************************************************
//FUNCTION:
void CPointCloud::__setNeighborInfo(const std::vector<unsigned int>* vNeighborInfoPointer)
{
	m_pNeighbors = const_cast<std::vector<unsigned int>*>(vNeighborInfoPointer);
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::setNormal(const std::vector<double>& vXSet, const std::vector<double>& vYSet, const std::vector<double>& vZSet)
{
	_ASSERT(vXSet.size() == vYSet.size() && vXSet.size() == vZSet.size());

	if (m_pNormal) SAFE_DELETE_ARRAY(m_pNormal);
	m_pNormal = new double[vXSet.size() * 3];

	const int SizeNormal = vXSet.size();

#pragma omp parallel for
	for (int i=0; i<SizeNormal; ++i)
	{
		*(m_pNormal + i*3 + 0) = vXSet[i];
		*(m_pNormal + i*3 + 1) = vYSet[i];
		*(m_pNormal + i*3 + 2) = vZSet[i];
	}
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::setNormal(const std::vector<osg::Vec3d>& vNormSet)
{
	if (m_pNormal) SAFE_DELETE_ARRAY(m_pNormal);
	m_pNormal = new double[vNormSet.size() * 3];
	const int SizeNormal = vNormSet.size();

#pragma omp parallel for
	for (int i=0; i<SizeNormal; ++i)
	{
		*(m_pNormal + i*3 + 0) = vNormSet[i].x();
		*(m_pNormal + i*3 + 1) = vNormSet[i].y();
		*(m_pNormal + i*3 + 2) = vNormSet[i].z();
	}
}

void hivePointCloud::CPointCloud::setNormal(const double* vNormal, unsigned int vNumPoint)
{
	__setNewData(&m_pNormal, vNormal, 3 * vNumPoint);
}

void hivePointCloud::CPointCloud::setPos(const double* vPos, unsigned int vNumPoint)
{
	__setNewData(&m_pPos, vPos, 3 * vNumPoint);
}

void hivePointCloud::CPointCloud::setK1(const double* vK1, unsigned int vNumPoint)
{
	__setNewData(&m_pK1Curvature, vK1, vNumPoint);
}

void hivePointCloud::CPointCloud::setK2(const double* vK2, unsigned int vNumPoint)
{
	__setNewData(&m_pK2Curvature, vK2, vNumPoint);
}

void hivePointCloud::CPointCloud::setGss(const double* vGss, unsigned int vNumPoint)
{
	__setNewData(&m_pGssCurvature, vGss, vNumPoint);
}

void hivePointCloud::CPointCloud::setMean(const double* vMean, unsigned int vNumPoint)
{
	__setNewData(&m_pMeanCurvature, vMean, vNumPoint);
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::__setNewData(double** vDest, const double* vSrc, unsigned int vNum)
{
	if (*vDest) SAFE_DELETE_ARRAY(*vDest);
	*vDest = new double[vNum];
	::memcpy(*vDest, vSrc, vNum * sizeof(double));
}

//*********************************************************************************
//FUNCTION:
const CPointCloud& hivePointCloud::CPointCloud::operator=(const CPointCloud& vOther)
{
	__doCopy(vOther);
	return *this;
}

//*********************************************************************************
//FUNCTION:
const CPointCloud& hivePointCloud::CPointCloud::transform(const double vRot[9], const double vTra[3])
{
	_ASSERT(vRot && vTra);

	for (unsigned int i=0; i<m_NumPoints; ++i)
	{
		const double PosX = *(m_pPos + i*3 + 0);
		const double PosY = *(m_pPos + i*3 + 1);
		const double PosZ = *(m_pPos + i*3 + 2);

		*(m_pPos + i*3 + 0) = vRot[0]*PosX + vRot[3]*PosY + vRot[6]*PosZ + vTra[0];
		*(m_pPos + i*3 + 1) = vRot[1]*PosX + vRot[4]*PosY + vRot[7]*PosZ + vTra[1];
		*(m_pPos + i*3 + 2) = vRot[2]*PosX + vRot[5]*PosY + vRot[8]*PosZ + vTra[2];
	}

	if (m_pNormal)
	{
		for (unsigned int i=0; i<m_NumPoints; ++i)
		{
			const double NormX = *(m_pNormal + i*3 + 0);
			const double NormY = *(m_pNormal + i*3 + 1);
			const double NormZ = *(m_pNormal + i*3 + 2);
			*(m_pNormal + i*3 + 0) = vRot[0]*NormX + vRot[3]*NormY + vRot[6]*NormZ;
			*(m_pNormal + i*3 + 1) = vRot[1]*NormX + vRot[4]*NormY + vRot[7]*NormZ;
			*(m_pNormal + i*3 + 2) = vRot[2]*NormX + vRot[5]*NormY + vRot[8]*NormZ;
		}
	}

	return *this;
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::export2TXT(const std::string& vFileName) const
{
	std::ofstream OutFile(vFileName.c_str());
	_ASSERT(OutFile.is_open());
	for (int i=0; i!=m_NumPoints; ++i)
	{
		OutFile << *(m_pPos + i*3 + 0) << " " << *(m_pPos + i*3 + 1) << " " << *(m_pPos + i*3 + 2) << std::endl;
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::__doCopy(const CPointCloud& vOther)
{
	this->m_Dimension = vOther.m_Dimension;
	this->m_NumPoints = vOther.m_NumPoints;
	this->m_NumNeighbors = vOther.m_NumNeighbors;
	this->m_AvgRadius = vOther.m_AvgRadius;

	if (m_pPos) SAFE_DELETE_ARRAY(m_pPos); m_pPos = new double[m_NumPoints*3];
	if (m_pNormal) SAFE_DELETE_ARRAY(m_pNormal); m_pNormal = new double[m_NumPoints*3];
	if (m_pGssCurvature) SAFE_DELETE_ARRAY(m_pGssCurvature); m_pGssCurvature = new double[m_NumPoints];
	if (m_pMeanCurvature) SAFE_DELETE_ARRAY(m_pMeanCurvature); m_pMeanCurvature = new double[m_NumPoints];
	if (m_pK1Curvature) SAFE_DELETE_ARRAY(m_pK1Curvature); m_pK1Curvature = new double[m_NumPoints];
	if (m_pK2Curvature) SAFE_DELETE_ARRAY(m_pK2Curvature); m_pK2Curvature = new double[m_NumPoints];

	if (m_pRadius) SAFE_DELETE_ARRAY(m_pRadius);
	if (m_pNeighbors) SAFE_DELETE(m_pNeighbors);

	if (vOther.m_pPos)
	{
		memcpy(this->m_pPos, vOther.m_pPos, sizeof(double)*m_NumPoints*3);
	}
	if (vOther.m_pNormal)
	{
		memcpy(this->m_pNormal, vOther.m_pNormal, sizeof(double)*m_NumPoints*3);
	}
	if (vOther.m_pGssCurvature)
	{
		memcpy(this->m_pGssCurvature, vOther.m_pGssCurvature, sizeof(double)*m_NumPoints);
	}
	if (vOther.m_pMeanCurvature)
	{
		memcpy(this->m_pMeanCurvature, vOther.m_pMeanCurvature, sizeof(double)*m_NumPoints);
	}
	if (vOther.m_pK1Curvature)
	{
		memcpy(this->m_pK1Curvature, vOther.m_pK1Curvature, sizeof(double)*m_NumPoints);
	}
	if (vOther.m_pK2Curvature)
	{
		memcpy(this->m_pK2Curvature, vOther.m_pK2Curvature, sizeof(double)*m_NumPoints);
	}
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::plyLoadNorm(const std::string& vFileID)
{
	CPLYLoad ply(vFileID.c_str());
	m_NumPoints      = ply.getNumPoints();
	m_pPos           = ply.getPostion();
	m_pNormal        = ply.getNormal();
}
