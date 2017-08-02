#include "PointCloudConfig.h"

using namespace hivePointCloud;

CConfig::CConfig(std::string vFileName, std::string vDelimiter, std::string vComment) : m_Delimiter(vDelimiter), m_Comment(vComment)
{
	std::ifstream in(vFileName.c_str());
	if (!in) throw SFileNotFound(vFileName); 
	in >> (*this);
}

CConfig::CConfig() : m_Delimiter(std::string(1,'=')), m_Comment(std::string(1,'#')){}

//*********************************************************************************
//FUNCTION:
void CConfig::Trim(std::string& vInOutS)
{
	const char whitespace[] = " \n\t\v\r\f";
	vInOutS.erase(0, vInOutS.find_first_not_of(whitespace));
	vInOutS.erase(vInOutS.find_last_not_of(whitespace) + 1U);
}