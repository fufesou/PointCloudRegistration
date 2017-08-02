#pragma once
#include <algorithm>
#include <boost/tuple/tuple.hpp>
#include "BaseSearch.h"
#include "DistanceQueue.h"

namespace hiveSearch
{
	class CKNNSearch : public CBaseSearch
	{
	public:
		CKNNSearch(void);
		virtual ~CKNNSearch(void);

		int  getMaxNumDataInLeafNode() const                          {return m_MaxNumDataInLeafNode;}
		void setMaxNumDataInLeafNode(const int vMaxNumDataInLeafNode) {m_MaxNumDataInLeafNode = vMaxNumDataInLeafNode;}

	protected:
		virtual bool _buildAccelerationStructureV() override;  
		virtual void _executeSearchV(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, bool vMultiThreaded, std::vector<unsigned int>& voResult) const override;

	private:
		struct STreeNode 
		{
			int SplitAxis;
			int LeftIndex, RightIndex;
			double SplitLowerBound, SplitUpperBound;
			STreeNode *pLeftChild;
			STreeNode *pRightChild;

			STreeNode() : LeftIndex(0), RightIndex(0), SplitAxis(0), SplitLowerBound(0.0), SplitUpperBound(0.0), pLeftChild(NULL), pRightChild(NULL) {}
		};

		typedef std::vector<std::pair<double, double>> CoordinateRange;

	private:
		STreeNode       *m_pRootNode;
		double          *m_pReorderedData;
		int              m_MaxNumDataInLeafNode;
		unsigned int    *m_pInputDataIndex;
		CoordinateRange  m_RootBoundingBox;

		void   __initInputDataIndex();
		void   __deleteTreeNodes();
		void   __destroyKDTree();
		void   __buildKDTree();
		void   __calculateBoundingBox(unsigned int vStart, unsigned int vEnd, CoordinateRange& vBoundingBox);
		void   __executeKNNForSingle(const double *vTargetPoint, unsigned int vNumNearestNeighbor, std::vector<unsigned int>& voResultSet) const;
		void   __searchNeighborInTreeNode(const double *vPoint, unsigned int vNumNearestNeighbor, const STreeNode *vNode, double vMinDistSquare, std::vector<double>& vioDistanceSet, CDistanceQueue& vioDistanceQueue) const;
		void   __collectSearchResult(const std::vector<std::vector<unsigned int>>& vSearchResultSet, std::vector<unsigned int>& voOutput) const;
		
		int    __splitNodeBySwappingIndex(unsigned int vNumInputPoint, int vSplitAxis, double vSplitValue, unsigned int *vioIndex) const;
		double __calculateInitialDistances(const double *vPoint, std::vector<double>& vDists) const;
		STreeNode* __splitData2TreeNode(int vLeftDataOffset, int vRightDataOffset, CoordinateRange& voSubBoundingBox);

		std::pair<double, double>      __computeCoordRangeOnSplitAxis(unsigned int *vIndex, int vNumInputPoint, int vSplitAxis) const;
		boost::tuple<int, int, double> __calculateSplitHyperPlaneInfo(int vNumInputPoint, unsigned int *vioIndex) const;

	    double __computeDiffSquare(double vA, double vB) const {return std::pow((vA-vB), 2);}
	};
}