#pragma once

#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/serialization/access.hpp>
#include "BaseProduct.h"


namespace hivePointCloud
{
	class CPointCloud;
}

namespace hiveRegistration
{
	class CUniqueData : public hiveCommon::CBaseProduct
	{
		friend boost::serialization::access;

	public:
		static std::string getSigSrcUniqueData() { return boost::to_upper_copy(std::string("SrcUniqueData")); }
		static std::string getSigTgtUniqueData() { return boost::to_upper_copy(std::string("TgtUniqueData")); }

		CUniqueData();

		void setPointCloud(const hivePointCloud::CPointCloud& vPointCloud);
		double getUniqSquareDist() const { return m_UnitSquareDist; }

	private:
		~CUniqueData();

		double m_UnitSquareDist;
	};
}