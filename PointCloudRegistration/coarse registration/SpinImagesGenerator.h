#pragma once
#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include "ICPType.h"
#include "bitmap_image.hpp"

namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveCommon
{
	class CKNNSearch;
}

namespace hiveRegistration
{
	class CSpinImagesGeneratorData;
	class CIndexSubset;

	class CSpinImagesGenerator
	{
	public:
		static std::string getKeyBinSize()       { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.BS")); }
		static std::string getKeyResFactor()     { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.RF")); }
		static std::string getKeyTopBeta()       { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.TB")); }
		static std::string getKeySupportAngle()  { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.SA")); }
		static std::string getKeyImageWidth()    { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.IW")); }
		static std::string getKeyImageHeight()   { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.IH")); }
		static std::string getKeySquareDistFactor() { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.SDF")); }
		static std::string getKeyNumKNNNeibs()   { return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.NKNN")); }
		static std::string getKeyBlockThreshold(){ return boost::algorithm::to_upper_copy(std::string("SpinImagesGenerator.BT")); }

		CSpinImagesGenerator(const hivePointCloud::CPointCloud& vPointCloud);
		~CSpinImagesGenerator();

		void genSpinImages(const std::vector<unsigned>& vSeeds, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& voSpinImages) const;
		void genImamge(unsigned int vCentIdx, Eigen::Matrix<double, Eigen::Dynamic, 1>& voImg) const;

	private:
		double m_BinSize;
		double m_ResFact;
		double m_TopBeta;
		double m_SupportAngle;
		double m_BlockThreshold;
		double m_ResDist;
		double m_SquareDistFactor;
		double m_SquareDistThreshold;
		unsigned m_NumKNNNeibs;
		unsigned m_ImageWidth;
		unsigned m_ImageHeight;

		mutable bitmap_image m_SpinBitMap;
		const hivePointCloud::CPointCloud* m_pPointCloud;
		boost::shared_ptr<hiveCommon::CKNNSearch> m_pKNN;

		void __parseCtrlParams(void);
		std::pair<double, double> __comImageDist(unsigned vCentPoint, unsigned vPixelPoint) const;
		std::pair<double, double> __comBilinearWeights(const std::pair<double, double>& vDistPair, const std::pair<unsigned, unsigned>& vIdxPair) const;
		std::pair<unsigned, unsigned> __comImageIdxPair(const std::pair<double, double>& vDistPair) const;
	};
}