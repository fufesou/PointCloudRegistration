#include "PointCloud.h"
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <osg/Geometry>
#include <osg/Geode>
#include <osgDB/WriteFile>
#include "EventLoggerInterface.h"
#include "HiveCommonMicro.h"

using namespace hivePointCloud;

CPointCloud::CPointCloud(void) : m_pRadius(NULL), m_pPos(NULL), m_pNormal(NULL), m_pNeighbors(NULL), m_pCurvature(NULL), m_AvgRadius(0), m_NumPoints(0), m_NumNeighbors(0)
{
}

CPointCloud::~CPointCloud(void)
{
	delete[] m_pPos;
	delete[] m_pNormal;
	delete[] m_pRadius;
	delete[] m_pCurvature;
	delete[] m_pNeighbors;
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::__initNeighborsInfo()
{
	_ASSERT(m_NumPoints > 0);
	m_pNeighbors = new std::vector<unsigned int>[m_NumPoints];
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::__initNormals()
{
	_ASSERT(m_NumPoints > 0);
	m_pNormal = new double[m_NumPoints*3];
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::__initCurvature()
{
	_ASSERT(m_NumPoints > 0);
	m_pCurvature = new double[m_NumPoints];
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::initPointCloud(const std::vector<osg::Vec3d>& vPointSet)
{
	_ASSERT(!vPointSet.empty());

	__clear();
	m_NumPoints = vPointSet.size();
	m_pPos = new double[m_NumPoints*3];
	for (unsigned int i=0; i<m_NumPoints; i++)
	{
		m_pPos[3*i]   = vPointSet[i][0];
		m_pPos[3*i+1] = vPointSet[i][1];
		m_pPos[3*i+2] = vPointSet[i][2];
	}
}

//********************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::initPointCloud( const std::string& vFileName )
{
	std::vector<osg::Vec3d> PointSet;
	std::fstream InputStream;
	InputStream.open(vFileName.c_str(), std::ios::in);
	if (!InputStream.is_open())
	{
		exit(0);
	}
	unsigned int NumPoints = 0;
	InputStream>>NumPoints;
	osg::Vec3d Tmp;

	while (!InputStream.eof())
	{
		InputStream>>Tmp[0];
		InputStream>>Tmp[1];
		InputStream>>Tmp[2];
		PointSet.push_back(Tmp);
	}
	initPointCloud(PointSet);
}

//*********************************************************************************
//FUNCTION:
void hivePointCloud::CPointCloud::exportFile(const char* vFile, EExportMode vMode /*= EPoints*/)
{
	if (!m_pPos)
	{
		return;
	}

	std::ofstream OutFile(vFile);
	if (!OutFile.is_open())
	{
		return;
	}
	
	OutFile << "Num Points: " << m_NumPoints << std::endl;

	if (vMode & EPoints)
	{
		OutFile << "x y z " << std::endl;
	}
	if ((vMode & ENormal) && m_pNormal)
	{
		OutFile << "nx ny nz " << std::endl;
	}
	if ((vMode & ECurvature) && m_pCurvature)
	{
		OutFile << "curv " << std::endl;
	}
	
	for (unsigned int i=0; i<m_NumPoints; ++i)
	{
		if (vMode & EPoints)
		{
			OutFile << m_pPos[i*3] << " " << m_pPos[i*3 + 1] << " " << m_pPos[i*3 + 2] << " ";
		}
		if ((vMode & ENormal) && m_pNormal)
		{
			OutFile << m_pNormal[i*3] << " " << m_pNormal[i*3 + 1] << " " << m_pNormal[i*3 + 2] << " ";
		}
		if ((vMode & ECurvature) && m_pCurvature)
		{
			OutFile << m_pCurvature[i] << " ";
		}
		OutFile << std::endl;
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::__clear()
{
	delete[] m_pPos;
	delete[] m_pNormal;
	delete[] m_pRadius;
	delete[] m_pNeighbors;
	
	m_NumPoints = 0;

	_clearExtraAttributesV();
}

//*********************************************************************************
//FUNCTION:
void CPointCloud::save2File(const std::string& vFileName)
{
	if (!m_pPos || (m_NumPoints == 0) || vFileName.empty()) return;

	osg::Node *pNode = convert2OSGNode();
	_HIVE_SIMPLE_IF_ELSE(pNode, osgDB::writeNodeFile(*pNode, vFileName),
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, _BOOST_STR1("Fail to the point cloud to file [%1%].", vFileName)));
}

//*********************************************************************************
//FUNCTION: 
osg::Node* CPointCloud::convert2OSGNode()
{
	if (!m_pPos || (m_NumPoints == 0)) return NULL;

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