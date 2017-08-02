#include "Utility.h"
#include <fstream>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include "ICPType.h"
#include "Intersection.h"
#include "CreateMatrix.h"

//*********************************************************************************
//FUNCTION:
void export2Ply(const char *vFileName, const hiveRegistration::VecColVector3 &vPointSet)
{
	std::ofstream OutFile(vFileName);
	_ASSERT(OutFile.is_open());
	fillPlyHeader(OutFile, vPointSet.size());

	for (hiveRegistration::VecColVector3::const_iterator citr=vPointSet.begin(); citr!=vPointSet.end(); ++citr)
	{
		OutFile << boost::str(boost::format("%f %f %f ") %(*citr)(0, 0) %(*citr)(1, 0) %(*citr)(2, 0))<< std::endl;
	}

	OutFile.close();
}

//*********************************************************************************
//FUNCTION:
void fillPlyHeader(std::ofstream &vOutFile, unsigned int vNumPoints)
{
	vOutFile << 
		"ply\n"
		"format ascii 1.0\n"
		"comment VCGLIB generated\n" << boost::str(boost::format("element vertex %d\n") %vNumPoints) <<
		"property float x\n"
		"property float y\n"
		"property float z\n"
		"element face 0\n"
		"property list uchar int vertex_indices\n"
		"end_header"
		<< std::endl;
}

//*********************************************************************************
//FUNCTION:
void genBSplineVerticesFile( const char *vFileName, const Eigen::Matrix4d *vCtlPnts )
{
	const Eigen::Matrix<double, 4, 4> DX = vCtlPnts[0];
	const Eigen::Matrix<double, 4, 4> DY = vCtlPnts[1];
	const Eigen::Matrix<double, 4, 4> DZ = vCtlPnts[2];

	static Eigen::Matrix<double, 4, 4> N;
	N <<  1,  4,  1, 0,
		-3,  0 , 3, 0,
		3, -6,  3, 0,
		-1,  3, -3, 1;
	N = N * (1.0/6.0);

	Eigen::Matrix4d NNx = N * DX * (N.transpose());
	Eigen::Matrix4d NNy = N * DY * (N.transpose());
	Eigen::Matrix4d NNz = N * DZ * (N.transpose());

	hiveRegistration::VecColVector3 BSplineVertices;

	double StepU = 0.01;
	double StepV = 0.01;

	for (double u=0; u<1; u+=StepU)
	{
		for (double v=0; v<1; v+=StepV)
		{
			double Vertex[3];
			
			Vertex[0] = createMatrixU(u)*NNx*createMatrixV(v);
			Vertex[1] = createMatrixU(u)*NNy*createMatrixV(v);
			Vertex[2] = createMatrixU(u)*NNz*createMatrixV(v);

  			BSplineVertices.push_back(hiveRegistration::ColVector3(Vertex[0], Vertex[1], Vertex[2]));
		}
	}

	export2Ply(vFileName, BSplineVertices);
}
