#include "DistanceQueue.h"

using namespace hiveSearch;

CDistanceQueue::CDistanceQueue(void) : m_QueueMaxSize(0)
{
}

CDistanceQueue::~CDistanceQueue(void)
{
}

//*********************************************************************************
//FUNCTION:
void CDistanceQueue::addElement2Queue(double vDistance, unsigned int vIndex)
{
	_ASSERT(vDistance >= 0);
	if (m_Queue.size() < m_QueueMaxSize)
		m_Queue.push(SDistance2Point(vDistance, vIndex));
	else if (vDistance < m_Queue.top().Distance)
	{
		m_Queue.pop();
		m_Queue.push(SDistance2Point(vDistance, vIndex));
	}
}

//*********************************************************************************
//FUNCTION:
void CDistanceQueue::clearQueue()
{
	while(!m_Queue.empty()) 
		m_Queue.pop();
}

//*********************************************************************************
//FUNCTION:
void CDistanceQueue::dumpIndex(std::vector<unsigned int>& voIndex, bool vClearOutputBeforeDump) const
{
	if (vClearOutputBeforeDump) voIndex.clear();

	std::priority_queue<SDistance2Point, std::vector<SDistance2Point>, std::less<SDistance2Point>> CopyQueue = m_Queue;
	while (!CopyQueue.empty())
	{
		voIndex.push_back(CopyQueue.top().Index);
		CopyQueue.pop();
	}
}