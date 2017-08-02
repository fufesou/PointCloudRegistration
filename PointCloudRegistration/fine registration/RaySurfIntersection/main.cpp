#include <iostream>
#include "Intersection.h"
#include "Utility.h"

int main(int argc, char* argv[])
{
	Eigen::Matrix<double, 4, 4> CtrlPoints[3];
	Eigen::Matrix<double, 3, 1> RayPnt;
	Eigen::Matrix<double, 3, 1> RayOri;

	RayPnt << 8,  5,  0;
	RayOri << 13, 9, -30;

	CtrlPoints[0] << -6, -6, -6, -6,
		-2, -2, -2, -2,
		2,  2,  2,  2,
		6,  6,  6,  6;

	CtrlPoints[1] << -6, -2, 2, 6,
		-6, -2, 2, 6,
		-6, -2, 2, 6,
		-6, -2, 2, 6;

	CtrlPoints[2] << 0, 0,  0,  0,
		0, 20, 20, 0,
		0, 20, 20, 0,
		0, 0,  0,  0;
	
	//VecColVector3 VertexSet;
	//
	//for (unsigned int i=0; i<16; ++i)
	//{	
	//	VertexSet.push_back(ColVector3(CtrlPoints[0](i/4, i%4), CtrlPoints[1](i/4, i%4), CtrlPoints[2](i/4, i%4)));
	//}

	//export2Ply("CtrPnts.ply", VertexSet);
	//genBSplineVerticesFile("BSplineSurf.ply", CtrlPoints);

	Eigen::Matrix<double, 2, 1> EndMatrix;
	EndMatrix << 0, 0;
	
	bool IsHaveIntersect = intersect(CtrlPoints, RayPnt, RayOri, EndMatrix);
	
	(IsHaveIntersect ? (std::cout << EndMatrix) : (std::cout << "no intersection")) << std::endl;
	
	return 0;
}