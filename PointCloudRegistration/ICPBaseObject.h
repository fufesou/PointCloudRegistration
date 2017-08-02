#pragma once
#include <boost\shared_ptr.hpp>

namespace hiveRegistration
{
	class CICPBaseObject
	{
	public:
		void deleteFunc() { _deleteFunc(); }

	protected:
		virtual ~CICPBaseObject() {}
		virtual void _deleteFunc() { delete this; }
	};

	struct BaseObjDeleter 
	{
		void operator()(CICPBaseObject* vPtr) { vPtr->deleteFunc(); }
	};

	template<typename T>
	boost::shared_ptr<T> makeSharedPtr()
	{
		return boost::shared_ptr<T>();
	}

	template<typename T>
	boost::shared_ptr<T> makeSharedPtr(T* vPtr)
	{
		return boost::shared_ptr<T>(vPtr, BaseObjDeleter());
	}
}