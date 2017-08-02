#include "SquareCellsInitialization.h"
#include <time.h>
#include <cfloat>
#include "HiveCommon.h"
#include "PointCloud.h"
#include "ProductFactoryData.h"
#include "ControlParameters.h"
#include "ICPMacros.h"
#include "RegUtilityFunctions.h"
#include "RegMath.h"

#ifdef _DEBUG
#include "TestUnitity.h"
#endif

using namespace hiveRegistration;

CSquareCellsInitialization::CSquareCellsInitialization(void)
	: m_pPointCloud(NULL)
{
	PTR_CONTROL_PARAMS->setIfNotExist(getSizeCtrlMatrixRow(), int(48));
	PTR_CONTROL_PARAMS->setIfNotExist(getSizeCtrlMatrixCol(), int(48));
}

CSquareCellsInitialization::~CSquareCellsInitialization(void)
{
}

//*********************************************************************************
//FUNCTION:
RegionDim3 CSquareCellsInitialization::__computeRegion() const
{
	Eigen::Matrix<double, 3, 2> PointsRegion;
	computeRegion<3>(m_pPointCloud->getPosPointer(), m_pPointCloud->getNumPoints(), PointsRegion);

	const double ExtendFactor = 0.0001;
	double ExtendLen[3] = { (PointsRegion(0, 1) - PointsRegion(0, 0)) * ExtendFactor, (PointsRegion(1, 1) - PointsRegion(1, 0)) * ExtendFactor, (PointsRegion(2, 1) - PointsRegion(2, 0)) * ExtendFactor };
	RegionDim3 ExtendRegion;
	ExtendRegion <<
		PointsRegion(0, 0) - ExtendLen[0], PointsRegion(0, 1) + ExtendLen[0],
		PointsRegion(1, 0) - ExtendLen[1], PointsRegion(1, 1) + ExtendLen[1], 
		PointsRegion(2, 0) - ExtendLen[2], PointsRegion(2, 1) + ExtendLen[2];

	return ExtendRegion;
}

//*********************************************************************************
//FUNCTION:
void CSquareCellsInitialization::__removeRedundantPoints( const SControlPointsMatrixInfo& vControlPointsMatrix, MatrixControlPoints& vioControlPoints ) const
{
	const int SizeCtrlMatrixRow = vControlPointsMatrix.NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = vControlPointsMatrix.NumTotalRowAndCols.second;

// #ifdef _DEBUG
// 	static int s_CtlIdx = 0;
// 	++s_CtlIdx;
// 	VecColVector3 CtlPnts;
// #endif

	for (int i=0; i<SizeCtrlMatrixRow; ++i)
	{
		for (int k=0; k<SizeCtrlMatrixCol; ++k)
		{
			if (vioControlPoints(i, k).size() > 1)
			{
				int RandomIndex = rand()%vioControlPoints(i, k).size();
				ColVector3 TmpPoint = vioControlPoints(i, k)[RandomIndex];
				vioControlPoints(i, k).swap(VecColVector3(1, TmpPoint));
			}
// #ifdef _DEBUG
// 			if (vioControlPoints(i, k).size())
// 			CtlPnts.push_back(vioControlPoints(i, k)[0]);
// #endif
		}
	}

// #ifdef _DEBUG
// 	static char FileName[255];
// 	sprintf(FileName, "TestData\\output\\CorPointPairs\\FineReg_Sample\\%d_Ctl_Pnts.ply", s_CtlIdx);
// 	export2Ply(FileName, CtlPnts);
// #endif
}

//*********************************************************************************
//FUNCTION:
void CSquareCellsInitialization::__genControlPointsMatrix( SControlPointsMatrixInfo& voProjectionCellMatrix, MatrixControlPoints& voControlPoints ) const
{
	RegionDim3 Region = __computeRegion();
	__initControlPointsMatrix(Region, voProjectionCellMatrix, voControlPoints);

	const int SizeCtrlMatrixRow = voProjectionCellMatrix.NumTotalRowAndCols.first;
	const int SizeCtrlMatrixCol = voProjectionCellMatrix.NumTotalRowAndCols.second;
#ifdef _DEBUG
	__storeControlPointsMatrix2File(voControlPoints, SizeCtrlMatrixRow, SizeCtrlMatrixCol);
#endif
	
	__removeRedundantPoints(voProjectionCellMatrix, voControlPoints);
}

//*********************************************************************************
//FUNCTION:
void CSquareCellsInitialization::genControlPointsMatrix( const hivePointCloud::CPointCloud* vPointCloud, SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints )
{
	_ASSERT(vPointCloud);

	if (m_pPointCloud != vPointCloud)
	{
		m_pPointCloud = vPointCloud;

		__genControlPointsMatrix(voControlPointsMatrix, voControlPoints);
	}
}

//*********************************************************************************
//FUNCTION:
void CSquareCellsInitialization::__initControlPointsMatrix( const RegionDim3& vRegion, SControlPointsMatrixInfo& voControlPointsMatrix, MatrixControlPoints& voControlPoints ) const
{
	int SizeCtrlMatrixRow;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getSizeCtrlMatrixRow(), SizeCtrlMatrixRow);
	_ASSERT(DumpRes);

	int SizeCtrlMatrixCol;
	DumpRes = PTR_CONTROL_PARAMS->dumpValue(getSizeCtrlMatrixCol(), SizeCtrlMatrixCol);
	_ASSERT(DumpRes);

	voControlPointsMatrix.NumTotalRowAndCols.first = SizeCtrlMatrixRow;
	voControlPointsMatrix.NumTotalRowAndCols.second = SizeCtrlMatrixCol;

	const double UnitLength0 = ( vRegion(0, 1) - vRegion(0, 0)) / SizeCtrlMatrixRow;
	const double UnitLength1 = ( vRegion(1, 1) - vRegion(1, 0)) / SizeCtrlMatrixCol;
	voControlPointsMatrix.SizeEachBlock.first  = UnitLength0;
	voControlPointsMatrix.SizeEachBlock.second = UnitLength1;

	voControlPointsMatrix.Range = vRegion;

	voControlPoints.resize(SizeCtrlMatrixRow, SizeCtrlMatrixCol);

	const int NumPoint = static_cast<int>(m_pPointCloud->getNumPoints());
	const double* pPos = m_pPointCloud->getPosPointer();
	
//#pragma omp parallel for
	for (int i=0; i<NumPoint; ++i)
	{
		ColVector3 Point(pPos[i*3 + 0], pPos[i*3 + 1], pPos[i*3 + 2]);
		const int CellIndex0 = (Point(0, 0) - vRegion(0, 0)) / UnitLength0;
		const int CellIndex1 = (Point(1, 0) - vRegion(1, 0)) / UnitLength1;
		voControlPoints(CellIndex0, CellIndex1).push_back(Point);
	}
}

#ifdef _DEBUG
//*********************************************************************************
//FUNCTION:
void hiveRegistration::CSquareCellsInitialization::__storeControlPointsMatrix2File( const MatrixControlPoints& vControlPoints, int vIndexMaxRows, int vIndexMaxCols ) const
{
	int IndexSampling = 0;
	char FileName[255];

	std::string OutputFilePathRoot;
	bool DumpRes = PTR_CONTROL_PARAMS->dumpValue(getOutputFilePathRoot(), OutputFilePathRoot);
	_ASSERT(DumpRes);

	std::string CurrentFilePath;

	for (int i=0; i<vIndexMaxRows; i+=4)
	{
		for (int k=0; k<vIndexMaxCols; k+=4)
		{
			if ( !vControlPoints(i,	k  ).empty() && !vControlPoints(i+1, k  ).empty() && !vControlPoints(i+2, k  ).empty() && !vControlPoints(i+3, k  ).empty() &&
				!vControlPoints(i,	k+1).empty() && !vControlPoints(i+1, k+1).empty() && !vControlPoints(i+2, k+1).empty() && !vControlPoints(i+3, k+1).empty() &&
				!vControlPoints(i,	k+2).empty() && !vControlPoints(i+1, k+2).empty() && !vControlPoints(i+2, k+2).empty() && !vControlPoints(i+3, k+2).empty() &&
				!vControlPoints(i,	k+3).empty() && !vControlPoints(i+1, k+3).empty() && !vControlPoints(i+2, k+3).empty() && !vControlPoints(i+3, k+3).empty() )
			{
				for (int m=0; m<4; ++m)
				{
					for (int n=0; n<4; ++n)
					{
						const VecColVector3 &CurPointsInCell = vControlPoints(i+m, k+n);
						sprintf(FileName, "SurfaceRepresentedByPoints_%d_%d_%d.ply", IndexSampling++, i+m, k+n);
						CurrentFilePath = OutputFilePathRoot + FileName;
						export2Ply(CurrentFilePath.c_str(), CurPointsInCell);
					}
				}
			}
		}
	}
}
#endif