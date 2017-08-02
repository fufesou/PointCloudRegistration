#pragma once
#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>
#include "CommonExport.h"

namespace hiveCommon
{
	class COMMON_DLL_EXPORT CKNNSearch
	{
	public:
		CKNNSearch(void);
		virtual ~CKNNSearch(void);

		void initKNNSearch(const double *vInputPoints, unsigned int vNumInputPoint, unsigned int vDimension, unsigned int vNumNeighbor);

		void executeKNN(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vNumNeighbor, unsigned int vDimension, const double *vTargetPoint, std::vector<unsigned int>& voKNNResultSet);
		void executeKNN(const double *vTargetPoint, std::vector<unsigned int>& voKNNResultSet);

		std::vector<unsigned int>* executeKNN(const double *vTargetPoint, unsigned int vNumTargetPoint);
		std::vector<unsigned int>* executeKNN(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vNumNeighbor, unsigned int vDimension, double *vTargetPoint, unsigned int vNumTargetPoint);

	private:
		struct SNode 
		{
			int LeftIndex, RightIndex;
			int DivAxis;
			double Divlow, Divhigh;
			SNode *pLeftChild;
			SNode *pRightChild;

			SNode() : LeftIndex(0), RightIndex(0), DivAxis(0), Divlow(0.0), Divhigh(0.0), 
				pLeftChild(NULL), pRightChild(NULL) {}
		};

		struct SInterval
		{
			double Low, High;
			SInterval() : Low(0.0), High(0.0) {}
		};
		typedef std::vector<SInterval> AABBBoundingBox;

		template<class ElementType>
		class CKNNSimpleResultSet 
		{
		public:
			CKNNSimpleResultSet(unsigned int vCapacity) : m_Capacity(vCapacity)
			{
				m_DistIndex.resize(vCapacity, DistanceIndex<ElementType>(std::numeric_limits<ElementType>::max(),-1));
				clear();
			}

			~CKNNSimpleResultSet(){ }

			void clear()
			{
				m_WorstDis = std::numeric_limits<ElementType>::max();
				m_DistIndex[m_Capacity-1].Distance = m_WorstDis;
				m_Count = 0;
			}

			unsigned int size() const
			{
				return m_Count;
			}

			bool full() const
			{
				return m_Count == m_Capacity;
			}

			void addPoint(ElementType vDist, unsigned int vIndex)
			{
				if (vDist >= m_WorstDis) return;

				if (m_Count < m_Capacity) ++m_Count;
				unsigned int i = 0;
				for (i=m_Count-1; i>0; --i){
					if (m_DistIndex[i-1].Distance>vDist)
					{
						m_DistIndex[i] = m_DistIndex[i-1];
					}
					else break;
				}
				m_DistIndex[i].Distance = vDist;
				m_DistIndex[i].Index = vIndex;
				m_WorstDis = m_DistIndex[m_Capacity-1].Distance;
			}

			void copy(unsigned int *voIndices, ElementType *voDistance, unsigned int vNumOfElement, bool sorted = true)
			{
				unsigned int n = std::min(m_Count, vNumOfElement);
				for (unsigned int i=0; i<n; ++i) {
					*voIndices++ = m_DistIndex[i].Index;
					*voDistance++ = m_DistIndex[i].Distance;
				}
			}

			ElementType getWorstDistance() const
			{
				return m_WorstDis;
			}

		private:
			template<class ElementType>
			struct DistanceIndex
			{
				DistanceIndex(ElementType vDist, unsigned int vIndex) : Distance(vDist), Index(vIndex){}
				ElementType Distance;
				unsigned int Index;
			};
			unsigned int m_Capacity;
			unsigned int m_Count;
			ElementType m_WorstDis;
			std::vector<DistanceIndex<ElementType>> m_DistIndex;
		};

	private:
		void   __buildSearchIndex();
		void   __computeBoundingBox(AABBBoundingBox& voBbox);
		void   __computeMinMax(const int *vIndices, int vNumInputPoint, int vDivAxis, double& voMinElem, double& voMaxElem);
		void   __findNeighbors(const double *vPoint, CKNNSimpleResultSet<double>& voResult);
		void   __searchKNearestNeighbor(const double *vQueries, unsigned int *voKIdices, double *voKDistance);
		void   __split(const AABBBoundingBox& vBBox, int *vIndices, int vNumInputPoint, int& voDivOffset, int& voDivAxis, double& voDivValue);
		void   __splitPlane(int vNumInputPoint, int vDivAxis, double vDivValue, int *vioIndices, int& voIndexLTDivValue, int& voIndexEorLTDivValue);
		void   __searchLevel(const double *vPoint, const SNode *vNode, double vMindistsq, std::vector<double>& vDists, CKNNSimpleResultSet<double>& voResult);
		double __computeInitialDistances(const double *vPoint, std::vector<double>& vDists);
		SNode* __divideTree(int vLeft, int vRight, AABBBoundingBox& vBbox);

	private:
		struct SComputeDistance
		{
			template <typename Iterator1, typename Iterator2>
			double operator()(Iterator1 a, Iterator2 b, unsigned int vSize) const
			{
				double Result = 0;
				double Diff;
				for(unsigned int i=0; i<vSize; ++i)
				{
					Diff = *a++ - *b++;
					Result += Diff*Diff;
				}
				return Result;
			}

			template <typename U, typename V>
			double computeDistance(const U& vA, const V& vB) const
			{
				return (vA-vB)*(vA-vB);
			}
		};

	private:
		bool    m_IsKDTreeBuilt;
		SNode  *m_pRootNode;
		double *m_pReorderData;
		unsigned int     m_NumPoint;
		unsigned int     m_Dimension;
		unsigned int     m_NumNeighbor;		
		AABBBoundingBox  m_RootBbox;
		SComputeDistance m_Distance;
		std::vector<int> m_IndexOfData;
		const int m_LeafMaxSize;
		const double *m_InputData;
	};
}