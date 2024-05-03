#ifndef SYNSLICERENGINE_OBJECT_TOOLPATH_H_
#define SYNSLICERENGINE_OBJECT_TOOLPATH_H_

#include "tool_point.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! This class is used to store multiple tool points.
	class Toolpath
	{
	public:
		//! Default constructor.
		Toolpath();

		//! Copy constructor.
		Toolpath(const Toolpath &other);
		
		//! Destructor.
		~Toolpath();

		//! Add tool point.
		/*!
			\param[in]	tool_point	Tool point.
		*/
		void addToolPoint(const ToolPoint &tool_point);

		//! Replace the n-th tool point with a new tool point.
		/*!
			\param[in]	index	n.
			\param[in]	tool_point	Tool point.
		*/
		void setToolPoint(int index, const ToolPoint &tool_point);

		//! Set extrusion of the n-th tool point.
		/*!
			\param[in]	index	n.
			\param[in]	extrusion	Extrusion.
		*/
		void setExtrusion(int index, double extrusion);

		//! Restore to default value.
		void reset();

		//! Get number of tool points.
		/*!
			\return \b int Number of tool points.
		*/
		const int size() const;

		// Get the last tool point.
		/*!
			\return <b> const ToolPoint & </b> Last tool point.
		*/
		const ToolPoint &back() const;

		//! Get the n-th tool point.
		/*!
			\param	index	n;
			\return <b> ToolPoint & </b> The n-th tool point.
		*/
		ToolPoint &operator[](int index);

		//! Get the n-th tool point.
		/*!
			\param	index	n;
			\return <b> ToolPoint & </b> The n-th tool point.
		*/
		const ToolPoint &operator[](int index) const;

		//! Copy assignment operator.
		Toolpath &operator = (const Toolpath &other);

	protected:
		// Store all the tool points.
		std::vector<ToolPoint> m_tool_path;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPATH_H_