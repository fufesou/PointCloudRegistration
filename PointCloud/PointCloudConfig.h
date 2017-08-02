#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

namespace hivePointCloud
{
	class CConfig 
	{
	private:
		std::string m_Comment; 
		std::string m_Delimiter;  
		std::map<std::string,std::string> m_Contents;  
		typedef std::map<std::string,std::string>::iterator       MapIterator;
		typedef std::map<std::string,std::string>::const_iterator MapConstIterator;

	public:
		CConfig(std::string vFileName,std::string vDelimiter = "=",std::string vComment = "#");
		CConfig();

		template<class Type> 
		Type getValueAt(const std::string& vInKey) const; 
		template<class Type> 
		Type getValueAt(const std::string& vInKey);

	private:
		template<class Type> 
		std::string translateOtherTypeToString(const Type& vInputValue);
		template<class Type> 
		Type translateStringToOtherType(const std::string& vInputString);
		static void Trim(std::string& vInOutS);
		friend std::ostream& operator<<(std::ostream& vOstream, const CConfig& vConfig)
		{
			for (CConfig::MapConstIterator p=vConfig.m_Contents.begin(); p!=vConfig.m_Contents.end(); ++p)
			{
				vOstream << p->first << " " << vConfig.m_Delimiter << " ";
				vOstream << p->second << std::endl;
			}
			return vOstream;
		}
		friend std::istream& operator>>(std::istream& vIstream, CConfig& vConfig)
		{
			typedef std::string::size_type pos;
			const std::string& delim  = vConfig.m_Delimiter; 
			const std::string& comm   = vConfig.m_Comment;   
			const pos skip = delim.length();       
			std::string nextline = "";  

			while (vIstream || nextline.length() > 0)
			{
				std::string line;
				if (nextline.length() > 0)
				{
					line = nextline;
					nextline = "";
				}
				else
				{
					std::getline(vIstream, line);
				}

				line = line.substr(0, line.find(comm));
				pos delimPos = line.find(delim);
				if (delimPos < std::string::npos)
				{
					std::string key = line.substr(0, delimPos);
					line.replace(0, delimPos+skip, "");

					bool terminate = false;
					while (!terminate && vIstream)
					{
						std::getline(vIstream, nextline);
						terminate = true;

						std::string nlcopy = nextline;
						CConfig::Trim(nlcopy);
						if (nlcopy == "") continue;

						nextline = nextline.substr(0, nextline.find(comm));
						if (nextline.find(delim) != std::string::npos)
							continue;

						nlcopy = nextline;
						CConfig::Trim(nlcopy);
						if (nlcopy != "") line += "\n";
						line += nextline;
						terminate = false;
					}

					CConfig::Trim(key);
					CConfig::Trim(line);
					vConfig.m_Contents[key] = line; 
				}
			}
			return vIstream;

		}

	public:
		struct SFileNotFound 
		{
		std::string FileName;
		SFileNotFound(const std::string& vFileName = std::string()) : FileName(vFileName) {} 
		};

		struct SKeyNotFound
		{ 
		std::string Key;
		SKeyNotFound(const std::string& vKey = std::string()) : Key(vKey) {} 
		};
	};

	//****************************************************************************************************************
	//FUNCTION:
	template<class Type>
	std::string CConfig::translateOtherTypeToString(const Type& vInputValue)
	{
		std::ostringstream Ost;
		Ost << vInputValue;
		return Ost.str();
	}

	//****************************************************************************************************************
	//FUNCTION:
	template<class Type>
	Type CConfig::translateStringToOtherType(const std::string& vInputString)
	{
		Type TempT;
		std::istringstream Ist(vInputString);
		Ist >> TempT;
		return TempT;
	}

	//****************************************************************************************************************
	//FUNCTION:
	template<>
	inline bool CConfig::translateStringToOtherType<bool>(const std::string& vInputString)
	{
		bool TempB = true;
		std::string Sup = vInputString;
		for (std::string::iterator p = Sup.begin(); p!=Sup.end(); ++p)
			*p = toupper(*p);  
		if (Sup == std::string("FALSE") || Sup == std::string("F") || Sup == std::string("NO") || Sup == std::string("N") || Sup == std::string("0") || Sup == std::string("NONE"))
			TempB = false;
		return TempB;
	}

	//****************************************************************************************************************
	//FUNCTION:
	template<class Type>
	Type CConfig::getValueAt(const std::string& vKey) const
	{
		MapConstIterator TempP = m_Contents.find(vKey);
		if (TempP == m_Contents.end()) throw SKeyNotFound(vKey);
		return translateStringToOtherType<Type>(TempP->second);
	}

	//****************************************************************************************************************
	//FUNCTION:
	template<class Type>
	Type hivePointCloud::CConfig::getValueAt(const std::string& vKey)
	{
		MapConstIterator TempP = m_Contents.find(vKey);
		if (TempP == m_Contents.end()) throw SKeyNotFound(vKey);
		return translateStringToOtherType<Type>(TempP->second);
	}
}
