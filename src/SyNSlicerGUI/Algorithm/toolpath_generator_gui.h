#ifndef SYNSLICERENGINE_GUI_TOOLPATHGENERATOR_H_
#define SYNSLICERENGINE_GUI_TOOLPATHGENERATOR_H_

#include "SyNSlicerEngine/Algorithm/toolpath_generator.h"
#include "SyNSlicerGUI/Algorithm/infill_path_generator_gui.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to generate toolpath to print a model.
	class ToolpathGeneratorGUI : public SA::ToolpathGenerator
	{
	public:
		//!  Default constructor is not allowed.
		ToolpathGeneratorGUI() = delete;

		//!  Constructor.
		/*!
			\param[in,out]	partition		The diameter of the filament used by the printer.
			\param[in]		with_support	Whether generate printing paths for support structure.
		*/
		ToolpathGeneratorGUI(SO::Partition<CgalMesh_EPICK> &partition, vtkRenderer *p_renderer, bool with_support = false);

		//!  Constructor.
		/*!
			\param[in,out]	partition		The diameter of the filament used by the printer.
			\param[in]		with_support	Whether generate printing paths for support structure.
		*/
		ToolpathGeneratorGUI(SO::Partition<CgalMesh_EPICK> &partition, SyNSlicerGUI::ObjectDrawer *p_drawer, bool with_support = false);

		//!  Destructor.
		~ToolpathGeneratorGUI();

	protected:
		//! Generate printing path for the bottom and top of the model.
		/*!
			\param[in]	wall_count		Number of wall.
		*/
		virtual void generateTopBottomUnionAndInfillContoursForModel(int wall_count) override;

		//! Generate printing path for the infill of the model.
		/*!
			\param[in]	wall_count	Number of wall.
			\param[in]	infill_type	Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
		*/
		virtual void generateInfillForModel(int wall_count, int infill_type) override;

		//! Generate printing path for the bottom and top of the model.
		/*!
			\param[in]	wall_count	Number of wall.
		*/
		virtual void generateTopBottomUnionAndInfillContoursForSupport(int wall_count) override;

		//! Generate printing path for the infill of the model.
		/*!
			\param[in]	wall_count	Number of wall.
			\param[in]	infill_type	Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
		*/
		virtual void generateInfillForSupport(int wall_count, int infill_type) override;

		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_TOOLPATHGENERATOR_H_