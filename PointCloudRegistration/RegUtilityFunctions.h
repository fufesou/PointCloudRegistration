#pragma once
#include <limits>
#include <xutility>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include "ICPType.h"


namespace hiveRegistration
{
	void setVec3ForwardDir(ColVector3& vioVec3);
	void compute3DCentroid(const VecColVector3& vPointSet, ColVector3& voCenteroid);
	void compute3DCentroid(const double* vPoint, const unsigned int vNumPoints, ColVector3& voCenteroid);

	template<class T1, class T2>
	bool hiveLexicalCast(const T1& vFrom, T2& voTo);

	template<unsigned int TDim>
	void computeRegion(const double* vPosPtr, const unsigned int vNumPoint, Eigen::Matrix<double, TDim, 2>& voRegion);

	template<unsigned int TDim>
	void computeSubsetRegion(const double* vPosPtr, const std::vector<unsigned>& vSubset, Eigen::Matrix<double, TDim, 2>& voRegion);

	template<typename TValue, typename TItr>
	bool isValueInArr(TValue vValue, TItr vBegin, TItr vEnd);

	template<typename TItr>
	double comSquaredDist(TItr vPABegin, unsigned int vPAIdx, TItr vPBBegin, unsigned int vPBIdx, unsigned int vDim = 3);

	template<typename TItr>
	double dot(TItr vPABegin, unsigned int vPAIdx, TItr vPBBegin, unsigned int vPBIdx, unsigned int vDim = 3);
}

//*********************************************************************************
//FUNCTION:
template<typename TItr>
double hiveRegistration::dot(TItr vPABegin, unsigned int vPAIdx, TItr vPBBegin, unsigned int vPBIdx, unsigned int vDim /*= 3*/)
{
	double Ret = 0.0;
	unsigned int i = 0;
	TItr PAMove = vPABegin;
	TItr PBMove = vPBBegin;
	std::advance(PAMove, vPAIdx*vDim);
	std::advance(PBMove, vPBIdx*vDim);
	while (i < vDim)
	{
		Ret += (*PAMove) * (*PBMove);
		++PAMove;
		++PBMove;
		++i;
	}
	return Ret;
}

//*********************************************************************************
//FUNCTION:
template<typename TItr>
double hiveRegistration::comSquaredDist(TItr vPABegin, unsigned int vPAIdx, TItr vPBBegin, unsigned int vPBIdx, unsigned int vDim /*= 3*/)
{
	double SquaredDist = 0.0;
	unsigned int i = 0;
	TItr PAMove = vPABegin;
	TItr PBMove = vPBBegin;
	std::advance(PAMove, vPAIdx*vDim);
	std::advance(PBMove, vPBIdx*vDim);
	while (i < vDim)
	{
		SquaredDist += ((*PAMove - *PBMove) * (*PAMove - *PBMove));
		++PAMove;
		++PBMove;
		++i;
	}
	return SquaredDist;
}

//*********************************************************************************
//FUNCTION:
template<unsigned int TDim>
void hiveRegistration::computeRegion(const double* vPosPtr, const unsigned int vNumPoint, Eigen::Matrix<double, TDim, 2>& voRegion)
{
	_ASSERT(vPosPtr && vNumPoint > 0 && TDim > 0);

	for (unsigned int i=0; i< TDim; ++i)
	{
		voRegion(i, 0) = std::numeric_limits<double>::max();
		voRegion(i, 1) = -std::numeric_limits<double>::max();
	}

	for (unsigned int i=0; i<vNumPoint; ++i)
	{
		for (unsigned int k=0; k<TDim; ++k)
		{
			voRegion(k, 0) = (voRegion(k, 0) > *vPosPtr) ? (*vPosPtr) : (voRegion(k, 0));
			voRegion(k, 1) = (voRegion(k, 1) < *vPosPtr) ? (*vPosPtr) : (voRegion(k, 1));
			++vPosPtr;
		}
	}
}

//*********************************************************************************
//FUNCTION:
template<unsigned int TDim>
void hiveRegistration::computeSubsetRegion(const double* vPosPtr, const std::vector<unsigned>& vSubset, Eigen::Matrix<double, TDim, 2>& voRegion)
{
	_ASSERT(vPosPtr && vSubset.size() > 0 && TDim > 0);

	for (unsigned int i=0; i< TDim; ++i)
	{
		voRegion(i, 0) = std::numeric_limits<double>::max();
		voRegion(i, 1) = -std::numeric_limits<double>::max();
	}

	for (std::vector<unsigned>::const_iterator CItr=vSubset.begin(); CItr!=vSubset.end(); ++CItr)
	{
		for (unsigned int k=0; k<TDim; ++k)
		{
			voRegion(k, 0) = (voRegion(k, 0) > vPosPtr[*CItr * TDim + k]) ? (vPosPtr[*CItr * TDim + k]) : (voRegion(k, 0));
			voRegion(k, 1) = (voRegion(k, 1) < vPosPtr[*CItr * TDim + k]) ? (vPosPtr[*CItr * TDim + k]) : (voRegion(k, 1));
		}
	}
}

//*********************************************************************************
//FUNCTION:
template<class T1, class T2>
bool hiveRegistration::hiveLexicalCast(const T1& vFrom, T2& voTo)
{
	try
	{
		voTo = boost::lexical_cast<T2>(vFrom);
		return true;
	}
	catch (boost::bad_lexical_cast e)
	{
#ifdef _DEBUG
		std::cout << "Bad boost::lexical_cast." << std::endl;
#endif
		return false;
	}
}

//*********************************************************************************
//FUNCTION:
template<typename TValue, typename TItr>
bool hiveRegistration::isValueInArr(TValue vValue, TItr vBegin, TItr vEnd)
{
	return std::find(vBegin, vEnd, vValue) != vEnd;
}