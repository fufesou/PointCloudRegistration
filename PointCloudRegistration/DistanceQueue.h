#pragma once
#include <vector>
#include <queue>

namespace hiveSearch
{
	class CDistanceQueue
	{
	public:
		CDistanceQueue(void);
		~CDistanceQueue(void);

		void setQueueMaxSize(unsigned int vSize) {_ASSERT(vSize > 0); m_QueueMaxSize = vSize;}
		void addElement2Queue(double vDistance, unsigned int vIndex);
		void dumpIndex(std::vector<unsigned int>& voIndex, bool vClearOutputBeforeDump=false) const;
		void clearQueue();
		unsigned int getSize() const {return m_Queue.size();}
		double getMaxDistance() const {return m_Queue.top().Distance;}

	private:
		struct SDistance2Point 
		{
			double Distance;
			unsigned int Index;

			SDistance2Point() : Distance(0), Index(0) {}
			SDistance2Point(double vDistance, unsigned int vIndex) : Distance(vDistance), Index(vIndex) {}

			friend bool operator<(const SDistance2Point& vA, const SDistance2Point& vB) 
			{
				return(vA.Distance < vB.Distance);
			}
		};

		unsigned int m_QueueMaxSize;
		std::priority_queue<SDistance2Point, std::vector<SDistance2Point>, std::less<SDistance2Point>> m_Queue;
	};
}