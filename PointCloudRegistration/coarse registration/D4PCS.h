#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "ICPType.h"


namespace hivePointCloud
{
	class CPointCloud;
};

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CD4PCS
	{
	public:
		static std::string getKeyApproximationLever() { return boost::to_upper_copy(std::string("D4PCS.AL")); }
		static std::string getKeyOverlapRatio() { return boost::to_upper_copy(std::string("D4PCS.OR")); }
		static std::string getKeyScoreFeet() { return boost::to_upper_copy(std::string("D4PCS.SF")); }
		static std::string getKeyScoreAln() { return boost::to_upper_copy(std::string("D4PCS.SA")); }
		static std::string getKeyFeetSize() { return boost::to_upper_copy(std::string("D4PCS.FS")); }
		static std::string getKeyNumLoop() { return boost::to_upper_copy(std::string("D4PCS.NL")); }
		static std::string getKeySampleStrID() { return boost::to_upper_copy(std::string("D4PCS.SSI")); }

		CD4PCS();
		~CD4PCS();

		bool coarseFit(const hivePointCloud::CPointCloud& vSrcPntCld, const hivePointCloud::CPointCloud& vTgtPntCld, Eigen::Matrix3d& voRot, Eigen::Vector3d& voTra);

	private:
		struct Param 
		{
			double ApproximationLevel;
			double OverlapRatio;
			int ScoreFeet;
			int ScoreAln;
			int FeetSize;
			int NumLoop;
			std::string SampleStrID;

			Param(): ApproximationLevel(0.5), FeetSize(25), OverlapRatio(0.5), ScoreFeet(50), ScoreAln(200), NumLoop(20), SampleStrID("SamplerRandom")
			{
			}
		};

		struct Candidate 
		{
			ColVector3 FourPnts[4];
			Eigen::Matrix3d R;
			Eigen::Vector3d T;
			double Err;
			int Score;
			int Base;

		};

		struct CmpLenDist
		{
			bool operator()(const boost::tuple<unsigned int, unsigned int, double>& vA, const boost::tuple<unsigned int, unsigned int, double>& vB)
			{
				return vA.get<2>() < vB.get<2>();
			}
			bool operator()(const boost::tuple<unsigned int, unsigned int, double>& vA, double vD)
			{
				return vA.get<2>() < vD;
			}
			bool operator()(double vD, const boost::tuple<unsigned int, unsigned int, double>& vB)
			{
				return vD < vB.get<2>();
			}
		};

		Param m_Param;
		double m_PntCldUnitDist;
		double m_SampleUnitDist;
		double m_FourPntsSide;
		double m_CurR1R2[2];
		ColVector3 m_FourPnts[4];
		VecColVector3 m_FeetPnts[4];
		VecColVector3 m_FeetPntsNormal[4];
		std::vector<Candidate> m_CandSet;
		SamplePointSet m_SrcPntSubset;
		SamplePointSet m_TgtPntSubset;
		std::vector<boost::tuple<unsigned int, unsigned int, double> > m_VecD1Len;
		std::vector<boost::tuple<unsigned int, unsigned int, double> > m_VecD2Len;

		const hivePointCloud::CPointCloud* m_pSrcPntCld;
		const hivePointCloud::CPointCloud* m_pTgtPntCld;
		VecColVector3 m_SrcPntSet;
		VecColVector3 m_SrcNormalSet;
		VecColVector3 m_TgtPntSet;
		VecColVector3 m_TgtNormalSet;

		boost::shared_ptr<hiveCommon::CKNNSearch> m_SrcSubsetKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_TgtSubsetKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_SrcPntCldKNN;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_TgtPntCldKNN;

		inline void __intersectLineLine(const Eigen::Vector3d vLineAVert[2], const Eigen::Vector3d vLineBVert[2], Eigen::Vector3d& voIntPntA, Eigen::Vector3d& voIntPntB) const
		{
			Eigen::Vector3d VecA = vLineAVert[1] - vLineAVert[0];
			Eigen::Vector3d VecB = vLineBVert[1] - vLineBVert[0];
			Eigen::Vector3d VecC = vLineBVert[0] - vLineAVert[0];
			voIntPntA = vLineAVert[0] + VecA * ((VecC.cross(VecB)).dot(VecA.cross(VecB))) / (VecA.cross(VecB)).squaredNorm();
			voIntPntB = vLineBVert[0] + VecA * ((VecC.cross(VecA)).dot(VecA.cross(VecB))) / (VecA.cross(VecB)).squaredNorm();
		}

		int __evaluateSample(const Candidate& vCand, const ColVector3& vPnt, const ColVector3& vNormal, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN, const VecColVector3& vPntSet, const VecColVector3& vNormSet, double vDistThr, double vCosAgl);
		bool __selectCoplanarBase(void);
		bool __computeR1R2(void);
		bool __findCongruent(void);
		bool __isTransformCongruent(const ColVector3 vFourPnts[4], Eigen::Matrix3d& voR, Eigen::Vector3d& voT, double& voErr);
		void __parseConfig(void);
		void __resetParam(void);
		void __comUnitDist(void);
		void __constructKNN(void);
		void __doSample(void);
		void __selectFeetPnts(void);
		void __testAlignment(Candidate& vioCand);
		void __evaluateAlignment(Candidate& vioCand);
		void __computeEdgeLen(double vD1, double vD2);
		void __init(const hivePointCloud::CPointCloud& vSrcPntCld, const hivePointCloud::CPointCloud& vTgtPntCld);
		double __comUnitSquareDist(const VecColVector3& vPntSet, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN) const;
		double __comUnitSquareDist(const double* vPos, unsigned int vNumPnts, boost::shared_ptr<hiveCommon::CKNNSearch> vKNN);
	};
}