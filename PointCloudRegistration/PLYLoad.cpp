#include "PLYLoad.h"
#include <string>
#include <fstream>

#define  SAFE_DEL_ARR(p) { if (p) { delete [] p; p = NULL; } }


CPLYLoad::CPLYLoad( const char *vNormalFileName, const char *vGaussianFileName, const char *vMeanFileName, const char *vK1FileName, const char *vK2FileName )
	: m_pPos(NULL)
	, m_pNormal(NULL)
	, m_pGaussianCurvature(NULL)
	, m_pMeanCurvature(NULL)
	, m_pK1Curvature(NULL)
	, m_pK2Curvature(NULL)
{
	__readFilePos(vNormalFileName, 3, &m_pNormal);
	__readFile(vGaussianFileName, 1, &m_pGaussianCurvature);
	__readFile(vMeanFileName, 1, &m_pMeanCurvature);
	__readFile(vK1FileName, 1, &m_pK1Curvature);
	__readFile(vK2FileName, 1, &m_pK2Curvature);
}

//*********************************************************************************
//FUNCTION:
CPLYLoad::CPLYLoad(const char *vNormalFileName)
: m_pPos(NULL)
, m_pNormal(NULL)
, m_pGaussianCurvature(NULL)
, m_pMeanCurvature(NULL)
, m_pK1Curvature(NULL)
, m_pK2Curvature(NULL)
{
	__readFilePos(vNormalFileName, 3, &m_pNormal);
}

CPLYLoad::~CPLYLoad()
{
	//SAFE_DEL_ARR(m_pPos);
	//SAFE_DEL_ARR(m_pNormal);
	//SAFE_DEL_ARR(m_pGaussianCurvature);
	//SAFE_DEL_ARR(m_pMeanCurvature);
	//SAFE_DEL_ARR(m_pK1Curvature);
	//SAFE_DEL_ARR(m_pK2Curvature);
}

void CPLYLoad::__readFile( const char* vFileName, int vNumProp, double** voData )
{
	std::ifstream InFile(vFileName);
	std::string StrLine;

	if (!InFile.is_open()) return;

	*voData = new double[vNumProp * m_NumPoints];

	while (!InFile.eof() && StrLine!="end_header")
	{
		InFile >> StrLine;
	}

	double Tmp;
	for (unsigned int i=0; i<m_NumPoints; ++i)
	{
		InFile >> Tmp;
		InFile >> Tmp;
		InFile >> Tmp;

		for (int k = 0; k < vNumProp; k++)
		{
			InFile >> (*voData)[i * vNumProp + k];
		}
	}

	InFile.close();
}

void CPLYLoad::__readFilePos( const char* vFileName, int vNumProp, double** voData )
{
	std::ifstream InFile(vFileName);
	std::string StrLine;

	if (!InFile.is_open()) return;

	while (!InFile.eof() && StrLine!="vertex")
	{
		InFile >> StrLine;
	}

	InFile >> m_NumPoints;

	m_pPos = new double[3 * m_NumPoints];
	*voData = new double[vNumProp * m_NumPoints];

	while (!InFile.eof() && StrLine!="end_header")
	{
		InFile >> StrLine;
	}

	for (unsigned int i=0; i<m_NumPoints; ++i)
	{
		InFile >> m_pPos[i*3 + 0];
		InFile >> m_pPos[i*3 + 1];
		InFile >> m_pPos[i*3 + 2];

		for (int k = 0; k < vNumProp; k++)
		{
			InFile >> (*voData)[i * vNumProp + k];
		}
	}

	InFile.close();
}
