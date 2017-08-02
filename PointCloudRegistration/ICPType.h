#pragma once
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "ICPBaseObject.h"

namespace hiveRegistration
{
	typedef Eigen::Matrix<double, 3, 1> ColVector3;
	typedef Eigen::Matrix<double, 3, 2> RegionDim3;
	
	typedef std::vector<ColVector3, Eigen::aligned_allocator<ColVector3> > VecColVector3;


	class CBasePointCloudSubset;
	typedef boost::shared_ptr<CBasePointCloudSubset> SamplePointSet;
	typedef boost::shared_ptr<CBasePointCloudSubset> CorrespondencePointSet;
	typedef std::pair<CorrespondencePointSet, CorrespondencePointSet> CorrespondencePairSet;
}