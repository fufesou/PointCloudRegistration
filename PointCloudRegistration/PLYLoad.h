#pragma once 

class CPLYLoad
{
public:
	CPLYLoad(const char *vNormalFileName);
	CPLYLoad(const char *vNormalFileName, const char *vGaussianFileName, const char *vMeanFileName, const char *vK1FileName, const char *vK2FileName);
	~CPLYLoad();

	double* getPostion() {return m_pPos;};
	double* getNormal() {return m_pNormal;};
	double* getGaussianCurvature() {return m_pGaussianCurvature;};
	double* getMeanCurvature() {return m_pMeanCurvature;};
	double* getK1Curvature() {return m_pK1Curvature;};
	double* getK2Curvature() {return m_pK2Curvature;};
	unsigned int getNumPoints() {return m_NumPoints;};

private:
	void __readFile(const char* vFileName, int vNumProp, double** voData);
	void __readFilePos(const char* vFileName, int vNumProp, double** voData);

	double* m_pPos;
	double* m_pNormal;
	double* m_pGaussianCurvature;
	double* m_pMeanCurvature;
	double* m_pK1Curvature;
	double* m_pK2Curvature;
	unsigned int m_NumPoints;
};
