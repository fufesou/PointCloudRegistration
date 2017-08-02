#pragma once
#include <xutility>
#include <algorithm>
#include <vector>
#include "HiveCommonMicro.h"
#include "EventLogger.h"
#include "ICPConstGlobleValue.h"

namespace hiveRegistration
{
	class CSimpleMedianFilter
	{
	public:
		template<typename TIter>
		static bool filter1D(TIter vBegin, TIter vEnd, unsigned int vDegree = 3);

		template<typename TIter, typename TPred>
		static bool filter1D(TIter vBegin, TIter vEnd, unsigned int vDegree, TPred vPred);

		template<typename TIter>
		static bool filter2D(TIter vBegin, TIter vEnd, unsigned int vWidth, unsigned int vHeight, unsigned int vDegree = 3);

		template<typename TIter, typename TPred>
		static bool filter2D(TIter vBegin, TIter vEnd, unsigned int vWidth, unsigned int vHeight, unsigned int vDegree, TPred vPred);

	private:
		static bool __checkDegree(unsigned int vDegree)
		{
			return (vDegree > 1) && (vDegree & 0x1) && (vDegree < 11);		// 好像 degree 通常取3和5，但此处为调试，就把值设大点，确定了后，用enum来限定范围
		}

		struct DefaultPred 
		{
			template<typename T>
			bool operator()(T vLeft, T vRight) const
			{
				return vLeft < vRight;
			}
		};
	};

	template<typename TIter>
	bool CSimpleMedianFilter::filter1D(TIter vBegin, TIter vEnd, unsigned int vDegree /*= 3*/)
	{
		return filter1D(vBegin, vEnd, vDegree, DefaultPred());
	}

	template<typename TIter, typename TPred>
	bool CSimpleMedianFilter::filter1D(TIter vBegin, TIter vEnd, unsigned int vDegree, TPred vPred)
	{
		_HIVE_ASSERT_AND_EARLY_RETURN(__checkDegree(vDegree), "invalid median filter degree!", false);	// FIXME: 这里用enum就不用check了，先用unsigned int是方便测试
		const unsigned int NumElem = std::distance(vBegin, vEnd);
		_HIVE_ASSERT_AND_EARLY_RETURN(vDegree <= NumElem, "degree is greater than the number of elements", false);
		TIter MoveIter = vBegin;
		std::advance(MoveIter, (vDegree >> 2));

		std::vector<TIter::value_type> TempValues;
		const unsigned int CmpTimes = NumElem - vDegree;

		for (unsigned int i=0; i<=CmpTimes; ++i)
		{
			TIter TempIter = vBegin;
			std::advance(TempIter, vDegree);
			TempValues.insert(TempValues.begin(), vBegin, TempIter);
			std::partial_sort(TempValues.begin(), TempValues.begin() + (TempValues.size()>>1), TempValues.end(), vPred);
			*MoveIter = *(TempValues.begin() + (TempValues.size()>>1));
			TempValues.clear();
			++MoveIter;
			++vBegin;
		}

		return true;
	}

	template<typename TIter>
	bool CSimpleMedianFilter::filter2D(TIter vBegin, TIter vEnd, unsigned int vWidth, unsigned int vHeight, unsigned int vDegree /*= 3*/)
	{
		return filter2D(vBegin, vEnd, vWidth, vHeight, vDegree, DefaultPred());
	}

	template<typename TIter, typename TPred>
	bool CSimpleMedianFilter::filter2D(TIter vBegin, TIter vEnd, unsigned int vWidth, unsigned int vHeight, unsigned int vDegree, TPred vPred)
	{
		_HIVE_ASSERT_AND_EARLY_RETURN(__checkDegree(vDegree), "invalid median filter degree!", false);
		const unsigned int NumElem = std::distance(vBegin, vEnd);

		return false;
	}

}