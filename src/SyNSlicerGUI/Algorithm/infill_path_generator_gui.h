#ifndef SYNSLICERENGINE_GUI_INFILLPATHGENERATOR_H_
#define SYNSLICERENGINE_GUI_INFILLPATHGENERATOR_H_

#include "SyNSlicerEngine/Algorithm/infill_path_generator.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to slice a 3D model with already calculated planes.
	class InfillPathGeneratorGUI : public SA::InfillPathGenerator
	{
	public:
		//! Default constructore is not allowed.
		InfillPathGeneratorGUI() = delete;


		InfillPathGeneratorGUI(
			const SO::PolygonCollection &contours, 
			const std::vector<SO::Plane> &cutting_planes,
			double side_step,
			int infill_type,
			vtkRenderer *p_renderer);

		InfillPathGeneratorGUI(
			const SO::PolygonCollection &contours,
			const std::vector<SO::Plane> &cutting_planes,
			double side_step,
			int infill_type,
			SyNSlicerGUI::ObjectDrawer *p_drawer);

		//! Destructor.
		~InfillPathGeneratorGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_INFILLPATHGENERATOR_H_