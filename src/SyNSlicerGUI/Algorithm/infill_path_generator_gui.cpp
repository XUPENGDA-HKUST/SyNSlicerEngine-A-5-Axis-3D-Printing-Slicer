#include "infill_path_generator_gui.h"

using SyNSlicerGUI::InfillPathGeneratorGUI;

InfillPathGeneratorGUI::InfillPathGeneratorGUI(const SO::PolygonCollection &contours, const std::vector<SO::Plane> &cutting_planes, double side_step, int infill_type, vtkRenderer *p_renderer)
	: InfillPathGenerator(contours, cutting_planes, side_step, infill_type)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

InfillPathGeneratorGUI::InfillPathGeneratorGUI(const SO::PolygonCollection &contours, const std::vector<SO::Plane> &cutting_planes, double side_step, int infill_type, SyNSlicerGUI::ObjectDrawer *p_drawer)
	: InfillPathGenerator(contours, cutting_planes, side_step, infill_type)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

InfillPathGeneratorGUI::~InfillPathGeneratorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}