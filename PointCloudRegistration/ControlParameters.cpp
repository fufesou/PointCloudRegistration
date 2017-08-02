#include "ControlParameters.h"
#include <fstream>
#include <boost/format.hpp>
#include "HiveCommonMicro.h"
#include "ProductFactory.h"
#include "CommonInterface.h"
#include "ConfigInterface.h"
#include "StringInterface.h"
#include "EventLoggerInterface.h"

using namespace hiveRegistration;

hiveCommon::CProductFactory<CControlParameters> TheCreator(CControlParameters::getClassSig());

CControlParameters::CControlParameters(void)
{
	_letFactoryReleaseProduct();
}

//*********************************************************************************
//FUNCTION:
bool CControlParameters::parseConfig(const std::string& vConfigFile /*= g_ConfigFile*/)
{
	if (vConfigFile.empty()) return false;

	std::string ConfigFileName = hiveCommon::hiveLocateFile(vConfigFile);
	_HIVE_EARLY_RETURN(ConfigFileName.empty(), _BOOST_STR1("Fail to locate the configuration file [%1%].", vConfigFile), false);

	__loadConfig(ConfigFileName);

	return true;
}

//*********************************************************************************
//FUNCTION:
void CControlParameters::__loadConfig(const std::string& vConfigFile)
{
	std::ifstream fin(vConfigFile.c_str());

	_HIVE_ASSERT_AND_EARLY_EXIT(!fin.bad(), _BOOST_STR1("Fail to open the pecified configuration file: %1%.", vConfigFile));

	std::string StrLine;
	std::string StrKey;
	std::string StrValue;

	try
	{
		while (!fin.eof())
		{
			StrLine = __getLine(fin);
			if ((StrLine == DEF_BAD_FILE_STRING) || (StrLine.empty())) break;

			hiveString::hiveSplitLine(StrLine, '=', StrKey, StrValue, true);
			m_ControlParams[StrKey] = StrValue;
		}
	}
	catch (...)
	{
		hiveEventLogger::hiveOutputWarning(__EXCEPTION_SITE__, _BOOST_STR1("Fail to parse the configuration file [%1%] due to unexpected error.", vConfigFile));
	}	
	fin.close();
}

//*********************************************************************************
//FUNCTION:
std::string CControlParameters::__getLine(std::ifstream& vInput)
{
	using hiveConfig::HIVE_CONFIG_LINE_MAX_SIZE;
	using hiveConfig::HIVE_COMMENT_CHAR;

	char LocalBuffer[HIVE_CONFIG_LINE_MAX_SIZE];
	std::string strOutput;
	int  numEmptyLine=0;

	//FIX_ME: crash if the number of characters in the current line is larger than HIVE_CONFIG_LINE_MAX_SIZE
	vInput.getline(LocalBuffer, HIVE_CONFIG_LINE_MAX_SIZE);
	while (__isEmptyLine(LocalBuffer, HIVE_CONFIG_LINE_MAX_SIZE) && (!vInput.eof()))
	{//ignore the empty line 
		vInput.getline(LocalBuffer, HIVE_CONFIG_LINE_MAX_SIZE);
		numEmptyLine++;
		if (numEmptyLine > 1000)
		{
			std::cout << "Too many empty lines. The configuration file may be damaged." << std::endl;
			return DEF_BAD_FILE_STRING;
		}
	}

	strOutput = "";
	if (!__isEmptyLine(LocalBuffer, HIVE_CONFIG_LINE_MAX_SIZE))
	{
		for (int i=0; i<HIVE_CONFIG_LINE_MAX_SIZE; i++)
		{
			if (LocalBuffer[i] == '\t') LocalBuffer[i] = ' ';   //replace TAB with empty pace
			if (LocalBuffer[i] == HIVE_COMMENT_CHAR)  
			{
				LocalBuffer[i] = '\0';     //remove the comment from the line
				break;
			}
			if (LocalBuffer[i] == 0) break;
		}
		strOutput = boost::algorithm::trim_copy(std::string(LocalBuffer));
	}

	return strOutput;
}

//*********************************************************************************
//FUNCTION:
bool CControlParameters::__isEmptyLine(char *vBuffer, int vBufferSize)
{
	using hiveConfig::HIVE_COMMENT_CHAR;

	for (int i=0; i<vBufferSize; i++)
	{
		if (('\n' == vBuffer[i]) || (0 == vBuffer[i]) || (HIVE_COMMENT_CHAR == vBuffer[i])) break;
		if ((' ' == vBuffer[i]) || ('\t' == vBuffer[i])) continue;
		return false;
	}
	return true;
}