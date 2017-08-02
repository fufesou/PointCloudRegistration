#pragma once
#include <vector>
#include <osg/Array>

namespace hiveTriangleSoup
{
	class CBaseTriangleSoup;
}

namespace hivePointCloud
{
	class CPointCloud
	{
	public:
		CPointCloud(const CPointCloud& vOther);
		CPointCloud(unsigned int vDimension = 3);
		~CPointCloud(void);

		unsigned int  getNumPoints()        const;
		unsigned int  getNumNeighbors()     const;
		const double* getPosPointer()       const;
		const double* getNormalPointer()    const;
		const double* getGssCurvaturePointer()  const;
		const double* getMeanCurvaturePointer() const;
		const double* getK1CurvaturePointer() const;
		const double* getK2CurvaturePointer() const;

		const std::vector<unsigned int>* getNeighborInfo() const;
		double* getNormalAt(unsigned int vIndex) const;
		std::vector<unsigned int>* getKNNPointsOfGivenPoint(unsigned int vGivenIndex) const;

		void        plyLoad(const std::string& vFileID);
		void        plyLoadNorm(const std::string& vFileID);
		void        setNormal(const std::vector<osg::Vec3d>& vNormSet);
		void        setNormal(const std::vector<double>& vXSet, const std::vector<double>& vYSet, const std::vector<double>& vZSet);
		void		initPointCloud(const std::vector<osg::Vec3d>& vPointSet);
		void		save2File(const std::string& vFileName);
		void        export2TXT(const std::string& vFileName) const;
		osg::Node*	convert2OSGNode();

		const CPointCloud& operator=(const CPointCloud& vOther);
		const CPointCloud& transform(const double vRot[9], const double vTra[3]);

		void setNumPoint(unsigned int vNumPoint)
		{
			m_NumPoints = vNumPoint;
		}
		void setPos(const double* vPos, unsigned int vNumPoint);
		void setK1(const double* vK1, unsigned int vNumPoint);
		void setK2(const double* vK2, unsigned int vNumPoint);
		void setGss(const double* vGss, unsigned int vNumPoint);
		void setMean(const double* vMean, unsigned int vNumPoint);
		void setNormal(const double* vNormal, unsigned int vNumPoint);

	protected:
		virtual void	_clearExtraAttributesV() {}
		void			_dumpKNearestNeighbor(unsigned int vNumNeighbor, std::vector<unsigned int>& voResult) const; 

	private:
		unsigned int m_Dimension;
		unsigned int m_NumPoints;
		unsigned int m_NumNeighbors;   
		double		 m_AvgRadius;
		double		*m_pPos;
		double		*m_pNormal;
		double		*m_pRadius;
		double      *m_pGssCurvature;
		double      *m_pMeanCurvature;
		double      *m_pK1Curvature;
		double      *m_pK2Curvature;
		std::vector<unsigned int> *m_pNeighbors;
		
		void	__clear();
		void	__initNeighborsInfo();
		void	__setNormalPointer(const double* vpNormalPointer);
		void	__setCurvaturePointer(const double* vpCurvature);
		void	__setNeighborInfo(const std::vector<unsigned int>* vNeighborInfoPointer);
		double* __reverseNormal(unsigned int vIndex);

		void __setNewData(double** vDest, const double* vSrc, unsigned int vNum);
		void __doCopy(const CPointCloud& vOther);

		friend void hiveTriangleSouplizePointCloud(CPointCloud& vioPointCloud, hiveTriangleSoup::CBaseTriangleSoup& voTriangleSoup);
	};
}
