#pragma once
#include <vector>
#include <osg/Array>
#include "PointCloudExport.h"

namespace hivePointCloud
{
	const unsigned int g_Dimension = 3;
	class POINT_CLOUD_DLL_EXPORT CPointCloud
	{
	public:
		enum EExportMode{ ENormal = 0x1, ECurvature = 0x2, EPoints = 0x4 };

		CPointCloud(void);
		~CPointCloud(void);

		unsigned int  getNumPoints()        const { return m_NumPoints;}
		unsigned int  getNumNeighbors()     const { return m_NumNeighbors;}
		const double* getPosPointer()       const { return m_pPos;}
		const double* getNormalPointer()    const { return m_pNormal;}
		const double* getCurvaturePointer() const { return m_pCurvature;}
		const std::vector<unsigned int>* getNeighborInfo() const{ return m_pNeighbors;}

		void initPointCloud(const std::vector<osg::Vec3d>& vPointSet);
		void initPointCloud(const std::string& vFileName);
		void exportFile(const char* vFile, EExportMode vMode = EPoints);
		void save2File(const std::string& vFileName);

		osg::Node* convert2OSGNode();

	protected:
		virtual void _clearExtraAttributesV() {}

		void _dumpKNearestNeighbor(unsigned int vNumNeighbor, std::vector<unsigned int>& voResult) const; 
	
	private:
		double *m_pPos;
		double *m_pNormal;
		double *m_pRadius;
		double *m_pCurvature;
		double  m_AvgRadius;
		unsigned int m_NumPoints;
		unsigned int m_NumNeighbors;   
		std::vector<unsigned int> *m_pNeighbors;

		void __initNeighborsInfo();
		void __initNormals();
		void __initCurvature();
		void __clear();

	friend class CNeighborDetector;
	friend class CComputeNormalByMLS;
	};
}
