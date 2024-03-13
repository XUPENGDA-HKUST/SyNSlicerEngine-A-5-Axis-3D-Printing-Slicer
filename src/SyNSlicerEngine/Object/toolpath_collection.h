#ifndef SYNSLICERENGINE_OBJECT_TOOLPATHCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_TOOLPATHCOLLECTION_H_

#include "Object/toolpath.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! This class is used to store multiple tool paths.
	/*!

	*/
	class ToolpathCollection
	{
	public:
		ToolpathCollection();
		ToolpathCollection(const ToolpathCollection &other);
		~ToolpathCollection();
		
		void setNumberOfToolPath(int number);
		void addToolPath(const Toolpath &tool_path);

		const unsigned int size() const;
		const Toolpath &operator[](unsigned int index) const;

		void reset();

		ToolpathCollection &operator=(const ToolpathCollection &other);

	protected:
		std::vector<Toolpath> m_tool_paths;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPATHCOLLECTION_H_