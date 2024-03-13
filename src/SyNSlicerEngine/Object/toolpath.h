#ifndef SYNSLICERENGINE_OBJECT_TOOLPATH_H_
#define SYNSLICERENGINE_OBJECT_TOOLPATH_H_

#include "tool_point.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! This class is used to store multiple tool points.
	/*!
		
	*/
	class Toolpath
	{
	public:
		Toolpath();
		Toolpath(const Toolpath &other);
		~Toolpath();

		void addToolPoint(const ToolPoint &tool_point);
		void setToolPoint(int index, const ToolPoint &tool_point);
		void setExtrusion(int index, double extrusion);

		void reset();

		const int size() const;
		const ToolPoint &back() const;

		ToolPoint &operator[](int index);
		const ToolPoint &operator[](int index) const;
		Toolpath &operator = (const Toolpath &other);

	protected:
		std::vector<ToolPoint> m_tool_path;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPATH_H_