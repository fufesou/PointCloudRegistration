#pragma once
#include <cmath>
#include <xutility>
#include <vector>

namespace hiveRegistration
{
	enum EDeviationFlag { POPULATION, SAMPLE };

	template<typename TIter>
	double computeMeanValue(TIter vBeginIter, TIter vEndIter, const unsigned int vStep = 1)
	{
		const unsigned int NumElem = std::distance(vBeginIter, vEndIter);
		unsigned int LoopTimes = NumElem / vStep;
		double Sum = 0.0;
		TIter Iter=vBeginIter;
		for (unsigned int LoopCounter = 0; LoopCounter != LoopTimes; std::advance(Iter, vStep), ++LoopCounter)
		{
			Sum += *Iter;
		}
		return Sum / NumElem;
	}

	template<typename TIter>
	double computeVariance(TIter vBeginIter, TIter vEndIter, EDeviationFlag vDeviationFlag = SAMPLE, const unsigned int vStep = 1)
	{
		const unsigned int NumElem = std::distance(vBeginIter, vEndIter);
		unsigned int LoopTimes = NumElem / vStep;
		double MeanValue = computeMeanValue(vBeginIter, vEndIter);
		double Variance = 0.0;
		TIter Iter=vBeginIter;
		for (unsigned int LoopCounter = 0; LoopCounter != LoopTimes; std::advance(Iter, vStep), ++LoopCounter)
		{
			Variance += (*Iter - MeanValue)*(*Iter - MeanValue);
		}

		Variance /= ( (vDeviationFlag == SAMPLE) ? (NumElem - 1) : (NumElem) );
		return Variance;
	}

	template<typename TIter>
	double computeVariance(TIter vBeginIter, TIter vEndIter, const double vMeanValue, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		const unsigned int NumElem = std::distance(vBeginIter, vEndIter);
		unsigned int LoopTimes = NumElem / vStep;
		double Variance = 0.0;
		TIter Iter=vBeginIter;
		for (unsigned int LoopCounter = 0; LoopCounter != LoopTimes; std::advance(Iter, vStep), ++LoopCounter)
		{
			Variance += (*Iter - vMeanValue)*(*Iter - vMeanValue);
		}

		Variance /= ( (vDeviationFlag == SAMPLE) ? (NumElem - 1) : (NumElem) );
		return Variance;
	}

	template<typename TIter>
	double computeStandardVariance(TIter vBeginIter, TIter vEndIter, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		return std::sqrt( computeVariance(vBeginIter, vEndIter, vDeviationFlag, vStep) );
	}

	template<typename TIter>
	double computeStandardVariance(TIter vBeginIter, TIter vEndIter, const double vMeanValue, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		return std::sqrt( computeVariance(vBeginIter, vEndIter, vMeanValue, vDeviationFlag, vStep) );
	}
}

namespace hiveRegistration
{
	template<typename T>
	double computeMeanValue(const T* vDataSet, unsigned int vNumData, unsigned int vStep = 1)
	{
		T Sum = 0.0;
		for (unsigned int i=0; i<vNumData; i+=vStep)
		{
			Sum += *(vDataSet + i);
		}
		return Sum / vNumData;
	}

	template<typename T>
	double computeVariance(const T* vDataSet, unsigned int vNumData, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		double MeanValue = computeMeanValue(vDataSet, vNumData);
		T Variance = 0.0;
		for (unsigned int i=0; i<vNumData; i+=vStep)
		{
			Variance += (*(vDataSet+i) - MeanValue)*(*(vDataSet+i) - MeanValue);
		}

		Variance /= ( (vDeviationFlag == SAMPLE) ? (vNumData - 1) : (vNumData) );
		return Variance;
	}

	template<typename T>
	double computeVariance(const T* vDataSet, unsigned int vNumData, const double vMeanValue, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		double Variance = 0.0;
		for (unsigned int i=0; i<vNumData; i+=vStep)
		{
			Variance += (*(vDataSet+i) - vMeanValue)*(*(vDataSet+i) - vMeanValue);
		}

		Variance /= ( (vDeviationFlag == SAMPLE) ? (vNumData - 1) : (vNumData) );
		return Variance;
	}

	template<typename T>
	double computeStandardVariance(const T* vDataSet, unsigned int vNumData, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		return std::sqrt( computeVariance(vDataSet, vNumData, vDeviationFlag) );
	}

	template<typename T>
	double computeStandardVariance(const T* vDataSet, unsigned int vNumData, const double vMeanValue, EDeviationFlag vDeviationFlag = SAMPLE, unsigned int vStep = 1)
	{
		return std::sqrt( computeVariance(vDataSet, vNumData, vMeanValue, vDeviationFlag) );
	}
}