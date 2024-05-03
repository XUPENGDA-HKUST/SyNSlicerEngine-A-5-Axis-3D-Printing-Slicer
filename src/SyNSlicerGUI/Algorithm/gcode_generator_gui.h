#ifndef SYNSLICERENGINE_GUI_GCODEGENERATOR_H_
#define SYNSLICERENGINE_GUI_GCODEGENERATOR_H_

#define _USE_MATH_DEFINES // for C++

#include "SyNSlicerEngine/Algorithm/gcode_generator.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//!  This class is used to generate gcode
	/*!
		Make sure all the contours are closed.
	*/
	class GcodeGeneratorGUI : public SA::GcodeGenerator
	{
	public:
		//! Default constructure is not allowed.
		GcodeGeneratorGUI() = delete;

		//! Constructor
		/*!
			\param[in] partitions All partitions to be printed.
			\param[in] side_step Distance between consecutive paths.
		*/
		GcodeGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> partitions, vtkRenderer *p_renderer, double side_step = 0.4);

		//! Constructor
		/*!
			\param[in] partitions All partitions to be printed.
			\param[in] side_step Distance between consecutive paths.
		*/
		GcodeGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> partitions, SyNSlicerGUI::ObjectDrawer *p_drawer, double side_step = 0.4);

		//! Destructor
		~GcodeGeneratorGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};

}

#endif  // SYNSLICERENGINE_GUI_GCODEGENERATOR_H_