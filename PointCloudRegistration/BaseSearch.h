#pragma once
#include <vector>

namespace hiveSearch
{
	class CBaseSearch
	{
	public:
		CBaseSearch(void);
		virtual ~CBaseSearch(void);

		void setInputData(const double *vInputPoint, unsigned int vNumInputPoint, unsigned int vDimension);
		void enableOutputSearchResult()   {m_IsOutputResultRequested = true;}
		void disableOutputSearchResult()  {m_IsOutputResultRequested = false;}

		std::vector<unsigned int>* executeSearch(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, bool vMultiThreaded=false);

		const double* getInputDataAt(unsigned int vIndex) const {return m_InputData.getInputDataAt(vIndex);}
		unsigned int  getDimensionOfInputData() const           {return m_InputData.Dimension;}
		unsigned int  getNumInputData() const                   {return m_InputData.NumInput;}

	protected:
		virtual bool _buildAccelerationStructureV() {return true;};
		virtual void _executeSearchV(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, bool vMultiThreaded, std::vector<unsigned int>& voResult) const {};
		virtual double _computeDisV(const double *vData1, const double *vData2) const;

	private:
		struct SInputData
		{
			const double *pData;
			unsigned int  NumInput;
			unsigned int  Dimension;

			SInputData() : pData(NULL), NumInput(0), Dimension(0) {}

			const double* getInputDataAt(unsigned int vIndex) const;
		};

		bool __buildAccelerationStructure();
		void __outputSearchResult(unsigned int vNumNearestNeighbor, const double *vTargetPoint, unsigned int vNumTargetPoint, const std::vector<unsigned int>* vResult) const;

		std::string __convertData2String(const double *vData) const;

		SInputData   m_InputData;
		bool         m_IsOutputResultRequested;
		bool         m_IsAccelerationStructureBuilt;
	};
}