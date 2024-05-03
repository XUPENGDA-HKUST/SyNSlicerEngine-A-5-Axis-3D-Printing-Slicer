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
		//! Default constructor.
		ToolpathCollection();

		//! Copy constructor.
		ToolpathCollection(const ToolpathCollection &other);

		//! Destructor.
		~ToolpathCollection();
		
		//! Set number of tool path.
		/*!
			\param	number	Number of tool path.
		*/
		void setNumberOfToolPath(int number);

		//! Add tool path.
		/*!
			\param	tool_path	Tool path.
		*/
		void addToolPath(const Toolpath &tool_path);

		//! Get number of tool paths.
		/*!
			\return <b> const unsigned int </b> Number of tool paths.
		*/
		const unsigned int size() const;

		//! Get the n-th tool path.
		/*!
			\param	index	n.
			\return <b> const Toolpath &</b> Tool path.
		*/
		const Toolpath &operator[](unsigned int index) const;

		//! Restore to default value.
		void reset();

		//! Copy assignment operator.
		ToolpathCollection &operator=(const ToolpathCollection &other);

	protected:
		//! Store all the tool paths.
		std::vector<Toolpath> m_tool_paths;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPATHCOLLECTION_H_