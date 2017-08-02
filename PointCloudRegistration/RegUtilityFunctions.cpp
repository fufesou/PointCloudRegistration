#include "RegUtilityFunctions.h"
#include <omp.h>
#include <climits>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "ICPMacros.h"

//*********************************************************************************
//FUNCTION:
void hiveRegistration::setVec3ForwardDir(ColVector3& vioVec3)
{
	if (std::abs(vioVec3(0, 0)) > EPSILON)
	{
		if (vioVec3(0, 0) < 0)
		{
			vioVec3 = -vioVec3;
		}
		return;
	}
	else if (std::abs(vioVec3(1, 0)) > EPSILON)
	{
		if (vioVec3(1, 0) < 0)
		{
			vioVec3 = -vioVec3;
		}
		return;
	}
	else if (std::abs(vioVec3(2, 0)) > EPSILON)
	{
		if (vioVec3(2, 0) < 0)
		{
			vioVec3 = -vioVec3;
		}
		return;
	}
	else
	{
		vioVec3 << 0.0, 0.0, 0.0;
	}
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::compute3DCentroid(const VecColVector3& vPointSet, ColVector3& voCenteroid)
{
	const int NumPointSet = vPointSet.size();
	_ASSERT(NumPointSet != 0);

	const int NumCore = omp_get_num_procs();
	std::vector<boost::shared_ptr<ColVector3> > VecSPColVec3;

	for (int i=0; i<NumCore; ++i)
	{
		VecSPColVec3.push_back(boost::shared_ptr<ColVector3>(new ColVector3));
		*(VecSPColVec3.back()) << 0.0, 0.0, 0.0;
	}

#pragma omp parallel for
	for (int i=0; i<NumPointSet; ++i)
	{
		const int ThreadID = omp_get_thread_num();
		*(VecSPColVec3[ThreadID]) += vPointSet[i];
	}

	voCenteroid<< 0.0, 0.0, 0.0;
	for (int i=0; i<NumCore; ++i)
	{
		voCenteroid += *(VecSPColVec3[i]);
	}

	voCenteroid /= NumPointSet;
}

//*********************************************************************************
//FUNCTION:
void hiveRegistration::compute3DCentroid(const double* vPoint, const unsigned int vNumPoints, ColVector3& voCenteroid)
{
	_ASSERT(vPoint && vNumPoints != 0);

	const int NumCore = omp_get_num_procs();
	std::vector<boost::shared_ptr<ColVector3> > VecSPColVec3;

	for (int i=0; i<NumCore; ++i)
	{
		VecSPColVec3.push_back(boost::shared_ptr<ColVector3>(new ColVector3));
		*(VecSPColVec3.back()) << 0.0, 0.0, 0.0;
	}

#pragma omp parallel for
	for (int i=0; i<vNumPoints; ++i)
	{
		const int ThreadID = omp_get_thread_num();
		(*(VecSPColVec3[ThreadID]))(0, 0) += vPoint[i*3 + 0];
		(*(VecSPColVec3[ThreadID]))(1, 0) += vPoint[i*3 + 1];
		(*(VecSPColVec3[ThreadID]))(2, 0) += vPoint[i*3 + 2];
	}

	voCenteroid<< 0.0, 0.0, 0.0;
	for (int i=0; i<NumCore; ++i)
	{
		voCenteroid += *(VecSPColVec3[i]);
	}

	voCenteroid /= vNumPoints;
}