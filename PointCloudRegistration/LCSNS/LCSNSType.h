#pragma once 
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include "ICPType.h"

namespace hiveRegistration
{
	typedef Eigen::Matrix<double, 2, 1> ColVector2;

	typedef Eigen::Matrix<ColVector3, Eigen::Dynamic, Eigen::Dynamic>                                             ControlPoints;
	typedef std::vector<ControlPoints, Eigen::aligned_allocator<ControlPoints> >                                  SimpleCtrlPntSet;
	typedef std::vector<std::pair<int, ControlPoints>, Eigen::aligned_allocator<std::pair<int, ControlPoints> > > ControlPointsSet;
	typedef boost::shared_ptr<SimpleCtrlPntSet>                                                                   PtrSimpleCtrlPntSet;
	typedef boost::shared_ptr<ControlPointsSet>                                                                   PtrControlPointsSet;

	typedef Eigen::Matrix<VecColVector3, Eigen::Dynamic, Eigen::Dynamic>       CandCtrlPnts;
	typedef std::vector<CandCtrlPnts, Eigen::aligned_allocator<CandCtrlPnts> > VecCandCtrlPnts;
}