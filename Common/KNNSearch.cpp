#include "KNNSearch.h"

using namespace hiveCommon;

CKNNSearch::CKNNSearch(void) : m_IsKDTreeBuilt(false), m_NumPoint(0), m_NumNeighbor(0), m_Dimension(0), m_LeafMaxSize(10), m_pReorderData(NULL), m_pRootNode(NULL)
{
}

CKNNSearch::~CKNNSearch(void)
{
	if (m_pReorderData) delete []m_pReorderData;
	if (m_pRootNode) delete m_pRootNode;
}

//*********************************************************************************
//FUNCTION: 
void CKNNSearch::initKNNSearch(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vDimension, unsigned int vNumNeighbor)
{
	_ASSERT(vInputPoint && (vNumNeighbor <= vNumInputPoint) && (vNumNeighbor > 0) && (vDimension>0));
	m_NumPoint    = vNumInputPoint; 	
	m_NumNeighbor = vNumNeighbor;
	m_Dimension   = vDimension;
	m_IndexOfData.resize(m_NumPoint, 0);
	for (unsigned int i=0; i<m_NumPoint; ++i)
		m_IndexOfData[i] = i;
	m_InputData = vInputPoint;
	__buildSearchIndex();
	m_IsKDTreeBuilt = true;
}

//*********************************************************************************
//FUNCTION: 
std::vector<unsigned int>* CKNNSearch::executeKNN(const double *vTargetPoint, unsigned int vNumTargetPoint)
{
	_ASSERT(m_IsKDTreeBuilt);
	std::vector<unsigned int> *pResult = new std::vector<unsigned int>[vNumTargetPoint];
	for (unsigned int i=0; i<vNumTargetPoint; ++i)
	{
		pResult[i].resize(m_NumNeighbor);
		executeKNN(&vTargetPoint[i*m_Dimension], pResult[i]);
	}
	return pResult;
}

//*********************************************************************************
//FUNCTION:
std::vector<unsigned int>* CKNNSearch::executeKNN(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vNumNeighbor, unsigned int vDimension, double *vTargetPoints, unsigned int vNumTargetPoint)
{
	_ASSERT((vNumInputPoint > 0) && vInputPoint && vTargetPoints && (vNumTargetPoint > 0) && (vNumTargetPoint < vNumInputPoint) && (vDimension > 0));
	initKNNSearch(vInputPoint, vNumInputPoint, vDimension, vNumNeighbor);
	return executeKNN(vTargetPoints, vNumTargetPoint);
}

//*********************************************************************************
//FUNCTION: 
void CKNNSearch::executeKNN(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vNumNeighbor, unsigned int vDimension, const double *vTargetPoint, std::vector<unsigned int>& voKNNResultSet)
{
	_ASSERT(vInputPoint && (vNumInputPoint > 0) && (vNumNeighbor > 0) && (vDimension > 0) && vTargetPoint);
	initKNNSearch(vInputPoint, vNumInputPoint, vDimension, vNumNeighbor);
	executeKNN(vTargetPoint, voKNNResultSet);
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::executeKNN(const double *vTargetPoint, std::vector<unsigned int>& voKNNResultSet)
{
	_ASSERT(m_IsKDTreeBuilt);
	if (voKNNResultSet.size() < m_NumNeighbor) voKNNResultSet.resize(m_NumNeighbor);
	std::vector<double> Distances;
	Distances.resize(m_NumNeighbor, 0.0);
	std::vector<double> Query (m_Dimension);
	for (unsigned int i=0; i<m_Dimension; ++i)
		Query[i] = vTargetPoint[i];
	__searchKNearestNeighbor(&Query[0], &voKNNResultSet[0], &Distances[0]);
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__buildSearchIndex()
{
	__computeBoundingBox(m_RootBbox);
	m_pRootNode = __divideTree(0, m_NumPoint, m_RootBbox);

	m_pReorderData = new double[m_NumPoint*m_Dimension];
	for (unsigned int i=0; i<m_NumPoint; ++i) 
	{
		for (unsigned int k=0; k<m_Dimension; ++k) 
		{
			m_pReorderData[i*m_Dimension+k] = m_InputData[m_IndexOfData[i]*m_Dimension+k];
		}
	}
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__computeBoundingBox(AABBBoundingBox& voBBox)
{
	voBBox.resize(m_Dimension);
	for (unsigned int i=0; i<m_Dimension; ++i) 
	{
		voBBox[i].Low  = m_InputData[i];
		voBBox[i].High = m_InputData[i];
	}

	for (unsigned int i=1; i<m_NumPoint; ++i) 
	{ 
		for (unsigned int k=0; k<m_Dimension; ++k) 
		{
			int Index = i * m_Dimension + k;
			if (m_InputData[Index] < voBBox[k].Low)
				voBBox[k].Low = m_InputData[Index];
			if (m_InputData[Index] > voBBox[k].High)
				voBBox[k].High = m_InputData[Index];
		}
	}
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__computeMinMax(const int *vIndices, int vNumInputPoint, int vSplitAxis, double& voMinElem, double& voMaxElem)
{
	int Index = vIndices[0] * m_Dimension + vSplitAxis;
	voMinElem = m_InputData[Index];
	voMaxElem = m_InputData[Index];
	for (int i=1; i<vNumInputPoint; ++i) 
	{
		double TempValue = m_InputData[vIndices[i]*m_Dimension+vSplitAxis];
		if (TempValue < voMinElem) voMinElem = TempValue;
		if (TempValue > voMaxElem) voMaxElem = TempValue;
	}
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__splitPlane(int vNumInputPoint, int vSplitAxis, double vSplitValue, int *vioIndices, int& voIndexLTSplitValue, int& voIndexEorLTSplitValue)
{
    int Left  = 0;
    int Right = vNumInputPoint - 1;
    while (true)
	{
		while (Left<=Right && m_InputData[vioIndices[Left] *m_Dimension+vSplitAxis]<vSplitValue)  ++Left;
		while (Left<=Right && m_InputData[vioIndices[Right]*m_Dimension+vSplitAxis]>=vSplitValue) --Right;
        if (Left > Right) break;
        std::swap(vioIndices[Left], vioIndices[Right]); ++Left; --Right;
    }

    voIndexLTSplitValue = Left;
    Right = vNumInputPoint - 1;
    while (true)
	{
		while (Left<=Right && m_InputData[vioIndices[Left] *m_Dimension+vSplitAxis]<=vSplitValue) ++Left;
		while (Left<=Right && m_InputData[vioIndices[Right]*m_Dimension+vSplitAxis]>vSplitValue)  --Right;
        if (Left > Right) break;
        std::swap(vioIndices[Left], vioIndices[Right]); ++Left; --Right;
    }
    voIndexEorLTSplitValue = Left;
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__split(const AABBBoundingBox& vBBox, int *vioIndices, int vNumInputPoint, int& voIndexOffset, int& voSplitAxis, double& voSplitValue)
{
	double MaxSpan = vBBox[0].High - vBBox[0].Low;
	voSplitAxis    = 0;
	voSplitValue   = (vBBox[0].High + vBBox[0].Low) / 2;
	for (unsigned int i=1; i<m_Dimension; ++i)
	{
		double Span = vBBox[i].High - vBBox[i].Low;
		if (Span > MaxSpan) 
		{
			MaxSpan      = Span;
			voSplitAxis  = i;
			voSplitValue = (vBBox[i].High+vBBox[i].Low)/2;
		}
	}

	double MinElem = 0.0, MaxElem = 0.0;
	__computeMinMax(vioIndices, vNumInputPoint, voSplitAxis, MinElem, MaxElem);
	voSplitValue = (MinElem + MaxElem) / 2;
	MaxSpan = MaxElem - MinElem;

	unsigned int k = voSplitAxis;
	for (unsigned int i=0; i<m_Dimension; ++i) 
	{
		if (i == k) continue;
		double Span = vBBox[i].High - vBBox[i].Low;
		if (Span > MaxSpan) 
		{
			__computeMinMax(vioIndices, vNumInputPoint, i, MinElem, MaxElem);
			Span = MaxElem - MinElem;
			if (Span > MaxSpan) 
			{
				MaxSpan      = Span;
				voSplitAxis  = i;
				voSplitValue = (MinElem + MaxElem) / 2;
			}
		}
	}

	int Index1, Index2;
	__splitPlane(vNumInputPoint, voSplitAxis, voSplitValue, vioIndices, Index1, Index2);

	if (Index1 > vNumInputPoint/2) voIndexOffset = Index1;
	else if (Index2 < vNumInputPoint/2) voIndexOffset = Index2;
	else voIndexOffset = vNumInputPoint / 2;
}

//*********************************************************************************
//FUNCTION
CKNNSearch::SNode* CKNNSearch::__divideTree(int vLeft, int vRight, AABBBoundingBox& vBBox)
{
	SNode*  pNode = new SNode;

	if ((vRight-vLeft) <= m_LeafMaxSize) 
	{
		pNode->pLeftChild = pNode->pRightChild = NULL;
		pNode->LeftIndex  = vLeft;
		pNode->RightIndex = vRight;

		for (unsigned int i=0; i<m_Dimension; ++i) 
		{
			int Index = m_IndexOfData[vLeft] * m_Dimension + i;
			vBBox[i].Low  = m_InputData[Index];
			vBBox[i].High = m_InputData[Index];
		}
		for (int k=vLeft+1; k<vRight; ++k) 
		{
			for (unsigned int i=0; i<m_Dimension; ++i) 
			{
				int Index = m_IndexOfData[k] * m_Dimension + i;
				if (vBBox[i].Low  > m_InputData[Index])
					vBBox[i].Low  = m_InputData[Index];
				if (vBBox[i].High < m_InputData[Index])
					vBBox[i].High = m_InputData[Index];
			}
		}
	}
	else 
	{
		int SplitAxis;
		int IndexOffset;
		double SplitValue;
		__split(vBBox, &m_IndexOfData[0]+vLeft, vRight-vLeft, IndexOffset, SplitAxis, SplitValue);

		pNode->DivAxis = SplitAxis;

		AABBBoundingBox LeftBBox(vBBox);
		LeftBBox[SplitAxis].High = SplitValue;
		pNode->pLeftChild = __divideTree(vLeft, vLeft+IndexOffset, LeftBBox);

		AABBBoundingBox RightBBox(vBBox);
		RightBBox[SplitAxis].Low = SplitValue;
		pNode->pRightChild = __divideTree(vLeft+IndexOffset, vRight, RightBBox);

		pNode->Divlow  = LeftBBox[SplitAxis].High;
		pNode->Divhigh = RightBBox[SplitAxis].Low;

		for (unsigned int i=0; i<m_Dimension; ++i) 
		{
			vBBox[i].Low  = std::min(LeftBBox[i].Low,  RightBBox[i].Low);
			vBBox[i].High = std::max(LeftBBox[i].High, RightBBox[i].High);
		}
	}
	return pNode;
}


//*********************************************************************************
//FUNCTION
void CKNNSearch::__searchKNearestNeighbor(const double *vQueries, unsigned int *voKIdices, double *voKDistance)
{
	CKNNSimpleResultSet<double> resultSet(m_NumNeighbor);
	resultSet.clear();
	__findNeighbors(vQueries, resultSet);
	resultSet.copy(voKIdices, voKDistance, m_NumNeighbor, true);
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__findNeighbors(const double *vPoint, CKNNSimpleResultSet<double>& voResult)
{
	std::vector<double> Distance(m_Dimension, 0);
	double DistanceSquare = __computeInitialDistances(vPoint, Distance);
	__searchLevel(vPoint, m_pRootNode, DistanceSquare, Distance, voResult);
}

//*********************************************************************************
//FUNCTION
double CKNNSearch::__computeInitialDistances(const double *vPoint, std::vector<double>& voDistance)
{
	double DistSq = 0.0;
	AABBBoundingBox AABBBox = m_RootBbox;
	for (unsigned int i=0; i<m_Dimension; ++i) 
	{
		if (vPoint[i] < AABBBox[i].Low) 
		{
			voDistance[i] = m_Distance.computeDistance(vPoint[i], AABBBox[i].Low);
			DistSq += voDistance[i];
		}
		if (vPoint[i] > AABBBox[i].High) 
		{
			voDistance[i] = m_Distance.computeDistance(vPoint[i], AABBBox[i].High);
			DistSq += voDistance[i];
		}
	}
	return DistSq;
}

//*********************************************************************************
//FUNCTION
void CKNNSearch::__searchLevel(const double *vPoint, const SNode *vNode, double vMindistsq, std::vector<double>& vDists, CKNNSimpleResultSet<double>& voResult)
{
	if ((vNode->pLeftChild == NULL) && (vNode->pRightChild == NULL)) 
	{
		double WorstDist = voResult.getWorstDistance();
		for (int i=vNode->LeftIndex; i<vNode->RightIndex; ++i) 
		{
			int Index = i;
			double *tempMatrix = m_pReorderData;
			double temp[3] = {tempMatrix[Index*m_Dimension], tempMatrix[Index*m_Dimension + 1], tempMatrix[Index*m_Dimension + 2]};
			double TempDistance = m_Distance(vPoint, temp, m_Dimension);
			if (TempDistance < WorstDist) 
			{
				voResult.addPoint(TempDistance,m_IndexOfData[i]);
			}
		}
		return;
	}

	int Idx = vNode->DivAxis;
	double TempVal = vPoint[Idx];
	double Diff1 = TempVal - vNode->Divlow;
	double Diff2 = TempVal - vNode->Divhigh;
	SNode* pBestChild  = NULL;
	SNode* pOtherChild = NULL;
	double CutDist = 0.0;
	if ((Diff1 + Diff2) < 0) 
	{
		pBestChild  = vNode->pLeftChild;
		pOtherChild = vNode->pRightChild;
		CutDist = m_Distance.computeDistance(TempVal, vNode->Divhigh);
	}
	else 
	{
		pBestChild  = vNode->pRightChild;
		pOtherChild = vNode->pLeftChild;
		CutDist = m_Distance.computeDistance( TempVal, vNode->Divlow);
	}

	__searchLevel(vPoint, pBestChild, vMindistsq, vDists, voResult);

	double TempDis = vDists[Idx];
	vMindistsq  = vMindistsq + CutDist - TempDis;
	vDists[Idx] = CutDist;
	if (vMindistsq <= voResult.getWorstDistance()) 
	{
		__searchLevel(vPoint, pOtherChild, vMindistsq, vDists, voResult);
	}
	vDists[Idx] = TempDis;
}