#include <stack>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include "CommonInterface.h"
#include "HiveCommonMicro.h"
#include "NewKNNSearch.h"

using namespace hiveSearch;

CKNNSearch::CKNNSearch(void) : m_pReorderedData(NULL), m_pRootNode(NULL), m_MaxNumDataInLeafNode(10), m_pInputDataIndex(NULL)
{
}

CKNNSearch::~CKNNSearch(void)
{
	__destroyKDTree();
	delete[] m_pInputDataIndex;
}

//*********************************************************************************
//FUNCTION: 
bool CKNNSearch::_buildAccelerationStructureV()
{
	_ASSERT(!m_pRootNode);
	__initInputDataIndex();
	__buildKDTree();	
	return true;
}

//*********************************************************************************
//FUNCTION: 
void CKNNSearch::_executeSearchV(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, bool vMultiThreaded, std::vector<unsigned int>& voResult) const
{
	_ASSERT(m_pRootNode && vTargetPoint && vNumNearestNeighbor > 0 && vNumNearestNeighbor <= getNumInputData() && vNumTargetPoint > 0);

	std::vector<std::vector<unsigned int>> SearchResultSet(vNumTargetPoint);
	if (vMultiThreaded)
	{
#pragma omp parallel for
		for (int i=0; i<vNumTargetPoint; ++i)
		{
			__executeKNNForSingle(&vTargetPoint[i*getDimensionOfInputData()], vNumNearestNeighbor, SearchResultSet[i]);
		}
	}
	else
	{
		for (unsigned int i=0; i<vNumTargetPoint; ++i)
		{
			__executeKNNForSingle(&vTargetPoint[i*getDimensionOfInputData()], vNumNearestNeighbor, SearchResultSet[i]);
		}
	}

	__collectSearchResult(SearchResultSet, voResult);
}

//*********************************************************************************
//FUNCTION:
void CKNNSearch::__collectSearchResult(const std::vector<std::vector<unsigned int>>& vSearchResultSet, std::vector<unsigned int>& voOutput) const
{
	voOutput.clear();
	for (unsigned int i=0; i<vSearchResultSet.size(); i++)
	{
		for (unsigned int k=0; k<vSearchResultSet[i].size(); k++)
		{
			voOutput.push_back(vSearchResultSet[i][k]);
		}
	}
}

//*********************************************************************************
//FUNCTION: 
void CKNNSearch::__initInputDataIndex()
{
	if (m_pInputDataIndex) delete[] m_pInputDataIndex;

	m_pInputDataIndex = new unsigned int[getNumInputData()];
	for (unsigned int i=0; i<getNumInputData(); ++i) m_pInputDataIndex[i] = i;
}

//*********************************************************************************
//FUNCTION
void hiveSearch::CKNNSearch::__executeKNNForSingle(const double *vTargetPoint, unsigned int vNumNearestNeighbor, std::vector<unsigned int>& voResultSet) const
{
 	CDistanceQueue DistanceQueue;
	DistanceQueue.setQueueMaxSize(vNumNearestNeighbor);

	std::vector<double> DistSet(getDimensionOfInputData(), 0.0);
	double DistanceSquare = __calculateInitialDistances(vTargetPoint, DistSet);
	__searchNeighborInTreeNode(vTargetPoint, vNumNearestNeighbor, m_pRootNode, DistanceSquare, DistSet, DistanceQueue);

	DistanceQueue.dumpIndex(voResultSet);
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__destroyKDTree()
{
	if (m_pReorderedData) 
	{
		delete []m_pReorderedData; 
		m_pReorderedData = NULL;
	}
	__deleteTreeNodes();
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__buildKDTree()
{
	__calculateBoundingBox(0, getNumInputData(), m_RootBoundingBox);
	m_pRootNode = __splitData2TreeNode(0, getNumInputData(), m_RootBoundingBox);
	m_pReorderedData = new double[getNumInputData()*getDimensionOfInputData()];

	for (unsigned int i=0; i<getNumInputData(); ++i) 
	{
		for (unsigned int k=0; k<getDimensionOfInputData(); ++k) 
		{
			m_pReorderedData[i*getDimensionOfInputData()+k] = getInputDataAt(m_pInputDataIndex[i])[k];
		}
	}
	_ASSERT(m_pRootNode && m_pReorderedData);
}
                           
//*********************************************************************************
//FUNCTION
void CKNNSearch::__calculateBoundingBox(unsigned int vStart, unsigned int vEnd, CoordinateRange& vBoundingBox)
{
	vBoundingBox.resize(getDimensionOfInputData());
	for (unsigned int i=0; i<getDimensionOfInputData(); ++i) 
	{
		vBoundingBox[i].first = getInputDataAt(m_pInputDataIndex[vStart])[i];
		vBoundingBox[i].second = getInputDataAt(m_pInputDataIndex[vStart])[i];
	}
	for (unsigned int i=vStart+1; i<vEnd; ++i) 
	{ 
		for (unsigned int k=0; k<getDimensionOfInputData(); ++k) 
		{
			if (getInputDataAt(m_pInputDataIndex[i])[k] < vBoundingBox[k].first) 
			{
				vBoundingBox[k].first  = (getInputDataAt(m_pInputDataIndex[i]))[k];
			}
			if (getInputDataAt(m_pInputDataIndex[i])[k] > vBoundingBox[k].second)
			{
				vBoundingBox[k].second = (getInputDataAt(m_pInputDataIndex[i]))[k];
			}
		}
	}
}

//*********************************************************************************
//FUNCTION
int CKNNSearch::__splitNodeBySwappingIndex(unsigned int vNumInputPoint, int vSplitAxis, double vSplitValue, unsigned int *vioIndex) const
{
    int Left  = 0;
    int Right = vNumInputPoint - 1;
    while (true)
	{
		while (Left <= Right && getInputDataAt(vioIndex[Left])[vSplitAxis] < vSplitValue)   
		{
			++Left;
		}
		while (Left <= Right && getInputDataAt(vioIndex[Right])[vSplitAxis] >= vSplitValue)
		{
			--Right;
		}
        if (Left > Right) 
		{
			break;
		}
        std::swap(vioIndex[Left], vioIndex[Right]); 
		++Left;
		--Right;
    }
	return Left;
}

//*********************************************************************************
//FUNCTION
boost::tuple<int, int, double> CKNNSearch::__calculateSplitHyperPlaneInfo(int vNumInputPoint, unsigned int *vioIndex) const
{
	int SplitAxis = 0;
	std::pair<double, double> TempExtremeElem(std::make_pair(0.0, 0.0)); 
	TempExtremeElem = __computeCoordRangeOnSplitAxis(vioIndex, vNumInputPoint, SplitAxis);
	double MedianValue = (TempExtremeElem.first + TempExtremeElem.second) / 2;
	double MaxSpaceRange = TempExtremeElem.second - TempExtremeElem.first;
	double TempSpaceRange;

	for (unsigned int i=1; i<getDimensionOfInputData(); ++i) 
	{
	   TempExtremeElem = __computeCoordRangeOnSplitAxis(vioIndex, vNumInputPoint, i);
	   TempSpaceRange = TempExtremeElem.second - TempExtremeElem.first;
	   if (TempSpaceRange > MaxSpaceRange) 
	   {
		   MaxSpaceRange = TempSpaceRange;
		   SplitAxis = i;
		   MedianValue = (TempExtremeElem.first + TempExtremeElem.second) / 2;
	   }
	}
	int LastIndexOfLeftNode = __splitNodeBySwappingIndex(vNumInputPoint, SplitAxis, MedianValue, vioIndex);
	return boost::make_tuple(LastIndexOfLeftNode, SplitAxis, MedianValue);
}

//*********************************************************************************
//FUNCTION
CKNNSearch::STreeNode* CKNNSearch::__splitData2TreeNode(int vLeftDataOffset, int vRightDataOffset, CoordinateRange& voSubBoundingBox)
{
	STreeNode* pNode = new STreeNode;
	_ASSERT(vRightDataOffset != vLeftDataOffset);
	if (vRightDataOffset - vLeftDataOffset <= m_MaxNumDataInLeafNode) 
	{
		pNode->pLeftChild = pNode->pRightChild = NULL;
		pNode->LeftIndex  = vLeftDataOffset;
		pNode->RightIndex = vRightDataOffset;
		__calculateBoundingBox(vLeftDataOffset, vRightDataOffset, voSubBoundingBox);
	}
	else 
	{
		int IndexOffset;
		int SplitAxis;
		double MedianValue;
		boost::tie(IndexOffset, SplitAxis, MedianValue) = __calculateSplitHyperPlaneInfo(vRightDataOffset - vLeftDataOffset, &m_pInputDataIndex[vLeftDataOffset]);
		pNode->SplitAxis = SplitAxis;
		CoordinateRange LeftSubBox(voSubBoundingBox);
		LeftSubBox[SplitAxis].second = MedianValue;
		pNode->pLeftChild = __splitData2TreeNode(vLeftDataOffset, vLeftDataOffset + IndexOffset, LeftSubBox);

		CoordinateRange RightSubBox(voSubBoundingBox);
		RightSubBox[SplitAxis].first = MedianValue;
		pNode->pRightChild = __splitData2TreeNode(vLeftDataOffset + IndexOffset, vRightDataOffset, RightSubBox);

		pNode->SplitLowerBound = LeftSubBox[SplitAxis].second;
		pNode->SplitUpperBound = RightSubBox[SplitAxis].first;

		for (unsigned int i=0; i<getDimensionOfInputData(); ++i) 
		{
			voSubBoundingBox[i].first  = std::min(LeftSubBox[i].first,  RightSubBox[i].first);
			voSubBoundingBox[i].second = std::max(LeftSubBox[i].second, RightSubBox[i].second);
		}
	}
	return pNode;
}

//*********************************************************************************
//FUNCTION
double CKNNSearch::__calculateInitialDistances(const double *vPoint, std::vector<double>& voDistance) const
{
	double DistSquare = 0.0;
	for (unsigned int i=0; i<getDimensionOfInputData(); ++i) 
	{
		if (vPoint[i] < m_RootBoundingBox[i].first) 
		{
			voDistance[i] = __computeDiffSquare(vPoint[i], m_RootBoundingBox[i].first);
			DistSquare += voDistance[i];
		}
		if (vPoint[i] > m_RootBoundingBox[i].second) 
		{
			voDistance[i] = __computeDiffSquare(vPoint[i], m_RootBoundingBox[i].second);
			DistSquare += voDistance[i];
		}
	}
	return DistSquare;
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__searchNeighborInTreeNode(const double *vTargetPoint, unsigned int vNumNeighbour, const STreeNode *vNode, double vMinDistSquare, std::vector<double>& vioDistanceSet, CDistanceQueue& vioDistanceQueue) const
{	
	if ((vNode->pLeftChild == NULL) && (vNode->pRightChild == NULL)) 
	{
		for (int i=vNode->LeftIndex; i<vNode->RightIndex; ++i) 
		{
			double TempDist = _computeDisV(vTargetPoint, &m_pReorderedData[i*getDimensionOfInputData()]);
			vioDistanceQueue.addElement2Queue(TempDist, m_pInputDataIndex[i]);
		}
		return;
	}
	int SplitAxis = vNode->SplitAxis;
	double CurrentDist = 0.0;
	STreeNode* pBestChild  = NULL;
	STreeNode* pOtherChild = NULL;
	if ((vTargetPoint[SplitAxis]-vNode->SplitLowerBound) + (vTargetPoint[SplitAxis]-vNode->SplitUpperBound) < 0) 
	{
		pBestChild  = vNode->pLeftChild;
		pOtherChild = vNode->pRightChild;
		CurrentDist = __computeDiffSquare(vTargetPoint[SplitAxis], vNode->SplitUpperBound);
	}
	else 
	{
		pBestChild  = vNode->pRightChild;
		pOtherChild = vNode->pLeftChild;
		CurrentDist = __computeDiffSquare(vTargetPoint[SplitAxis], vNode->SplitLowerBound);
	}

	__searchNeighborInTreeNode(vTargetPoint, vNumNeighbour, pBestChild, vMinDistSquare, vioDistanceSet, vioDistanceQueue);

	double TempDistSquare = vioDistanceSet[SplitAxis];
	vMinDistSquare = vMinDistSquare + CurrentDist - TempDistSquare;
	vioDistanceSet[SplitAxis] = CurrentDist;
	if ((vioDistanceQueue.getSize() < vNumNeighbour) || (vioDistanceQueue.getSize() >= vNumNeighbour && vMinDistSquare <= vioDistanceQueue.getMaxDistance()))
	{
		__searchNeighborInTreeNode(vTargetPoint, vNumNeighbour, pOtherChild, vMinDistSquare, vioDistanceSet, vioDistanceQueue);
	}
	vioDistanceSet[SplitAxis] = TempDistSquare;
}

//*********************************************************************************
//FUNCTION
std::pair<double, double> CKNNSearch::__computeCoordRangeOnSplitAxis(unsigned int *vIndex, int vNumInputPoint, int vSplitAxis) const
{
	std::pair<double, double> ExtremeElemPair = std::make_pair(getInputDataAt(vIndex[0])[vSplitAxis], getInputDataAt(vIndex[0])[vSplitAxis]);
	for (int i=1; i<vNumInputPoint; ++i) 
	{
		double TempValue = getInputDataAt(vIndex[i])[vSplitAxis];
		if (TempValue < ExtremeElemPair.first)  {ExtremeElemPair.first  = TempValue;}
		if (TempValue > ExtremeElemPair.second) {ExtremeElemPair.second = TempValue;}
	}
	return ExtremeElemPair;
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__deleteTreeNodes()
{
	std::stack<bool> IsRightUsed;
	std::stack<STreeNode*> TreeNodeStack;
	STreeNode *pNode = m_pRootNode;
	
	while (pNode != NULL)
	{
		while (pNode->pLeftChild != NULL || pNode->pRightChild != NULL)
		{
			while (pNode->pLeftChild != NULL) 
			{
				if (pNode->pRightChild != NULL) 
				{
					IsRightUsed.push(false);
				}
				else 
				{
					IsRightUsed.push(true);
				}
				TreeNodeStack.push(pNode);
				pNode = pNode->pLeftChild;
			}
			if (pNode->pRightChild != NULL)
			{
				IsRightUsed.push(true);
				TreeNodeStack.push(pNode);
				pNode = pNode->pRightChild;
			}
		}
		delete pNode;

		while (!TreeNodeStack.empty())
		{
			if (!IsRightUsed.top())
			{
				pNode = TreeNodeStack.top()->pRightChild;
				IsRightUsed.pop();
				IsRightUsed.push(true);
				break;
			}
			else 
			{
				delete TreeNodeStack.top();
				TreeNodeStack.pop();
				IsRightUsed.pop();
			}
		}
		if (TreeNodeStack.empty()) {pNode = NULL;}
	}
	m_pRootNode = NULL;
}

