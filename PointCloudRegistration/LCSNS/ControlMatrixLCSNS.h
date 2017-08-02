#pragma once
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include <Eigen/Core>
#include "ControlPointsData.h"
#include "ICPType.h"
#include "LCSNSType.h"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CControlMatrixLCSNS
	{
	public:
		CControlMatrixLCSNS(void);
		~CControlMatrixLCSNS(void);

		static std::string getKeySizeCtrlMatrixRow()    { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.SizeCMRow")); }
		static std::string getKeySizeCtrlMatrixCol()    { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.SizeCMCol")); }
		static std::string getKeySampleStrID()          { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.SampleStrID")); }
		static std::string getKeyNumNeibs()             { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.NumNeibs")); }
		static std::string getKeyZDistThreshodlFactor() { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.ZDTF")); }
		static std::string getKeySizeExtendCtrlMat()    { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.ExtendCtrlMat")); }
		static std::string getKeyExtendLengthFactor()   { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.ExtendLengthFactor")); }
		static std::string getKeyMatrixGenerationType() { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.MatrixGenerationType")); }

#ifdef _DEBUG
		static std::string getKeyControlPointSetFile()  { return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.CPSFile")); }
		static std::string getKeySurfaceSetFile()		{ return boost::algorithm::to_upper_copy(std::string("ControlMatrixLCSNS.SSFile")); }
#endif

		void genControlPointsMatrix(const hivePointCloud::CPointCloud* vPointCloud, PtrControlPointsSet& voCtrlPntsSet);

	private:
		int m_CtrlMatRow;
		int m_CtrlMatCol;
		int m_NumNeibs;
		int m_SizeExtendCtrlMat;
		int m_MatrixGenerationType;
		double m_ExtendLengthFactor;
		double m_ZDistThresholdFactor;
		std::string m_SampleStrID;
		const hivePointCloud::CPointCloud* m_pPointCloud;

		void __genControlPointsMatrix( const SamplePointSet& vSamplePointSet, PtrControlPointsSet& voCtrlPntsSet ) const;
		void __removeRedundantPoints(VecCandCtrlPnts& vCandCtrlPntsSet, PtrControlPointsSet& voCtrlPntsSet) const;
		void __initControlPointsMatrix( const SamplePointSet& vSamplePointSet, VecCandCtrlPnts& voCandCtrlPntsSet ) const;
		void __initNewCoordinates(const ColVector3& vNormal, ColVector3& voNewCoordinateAxisX, ColVector3& voNewCoordinateAxisY, ColVector3& voNewCoordinateAxisZ) const;
		void __computeRegion(const Eigen::Matrix<double, 3, Eigen::Dynamic>& vPointSet, RegionDim3& voRegion) const;
		void __setCoordOrigin(const std::vector<unsigned int>& vPointSet, ColVector3& voOrigin, ColVector3& voNormal) const;
		void __parseConfig(void);
	};
}

