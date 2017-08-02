#pragma once
#include <map>
#include <boost/algorithm/string.hpp>
#include "HiveCommon.h"
#include "BaseProduct.h"
#include "ICPConstGlobleValue.h"
#include "RegUtilityFunctions.h"

namespace hiveRegistration
{
	#define DEF_BAD_FILE_STRING "BAD FILE"
	// 这里有很多代码与configuration里面一样

	class CControlParameters : public hiveCommon::CBaseProduct
	{
	public:
		CControlParameters(void);
		virtual ~CControlParameters() {}

		static std::string getClassSig() { std::string Tmp("RegistrationConfig"); boost::algorithm::to_upper(Tmp); return Tmp; }

		bool parseConfig(const std::string& vConfigFile = g_ConfigFile);
		template<typename T>
		void set(const std::string& vKey, const T& vValue);
		template<typename T>
		void setIfNotExist(const std::string& vKey, const T& vValue);

		template<typename T>
		bool dumpValue(const std::string& vKey, T& voResult) const;
		
	private:
		std::map<std::string, std::string> m_ControlParams;

		void __loadConfig(const std::string& vConfigFile);
		bool __isEmptyLine(char *vBuffer, int vBufferSize);
		std::string __getLine(std::ifstream& vInput);
	};

	//*********************************************************************************
	//FUNCTION:
	template<typename T>
	void CControlParameters::setIfNotExist(const std::string& vKey, const T& vValue)
	{
		if (m_ControlParams.end() == m_ControlParams.find(vKey))
		{
			std::string Tmp;
			bool CastRes = hiveLexicalCast(vValue, Tmp);
			_ASSERT(CastRes);

			m_ControlParams[vKey] = Tmp;
		}
	}

	//*********************************************************************************
	//FUNCTION:	caution--set值时最好注明类型，如set("aa", 100)，100默认为signed int类型，如果不注意用unsigned int来取值，会出问题
	template<typename T>
	void CControlParameters::set(const std::string& vKey, const T& vValue)
	{
		std::string Tmp;
		bool CastRes = hiveLexicalCast(vValue, Tmp);
		_ASSERT(CastRes);
		m_ControlParams[vKey] = Tmp;
	}

	//*********************************************************************************
	//FUNCTION:
	template<typename T>
	bool CControlParameters::dumpValue(const std::string& vKey, T& voResult) const
	{
		std::map<std::string, std::string>::const_iterator ConstItr = m_ControlParams.find(vKey);
		if (ConstItr == m_ControlParams.end())
		{
			return false;
		}
		else
		{
			std::string Tmp;
			return (hiveLexicalCast(ConstItr->second, voResult));
		}
	}
}