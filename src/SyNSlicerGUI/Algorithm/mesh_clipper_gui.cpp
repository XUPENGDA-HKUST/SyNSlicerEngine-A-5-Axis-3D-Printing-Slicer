#include "mesh_clipper_gui.h"

using SyNSlicerGUI::MeshClipperGUI;

MeshClipperGUI::MeshClipperGUI(vtkRenderer *p_renderer)
	: m_should_drawer_delete_in_destructer(true)
	, mp_drawer(new ObjectDrawer(p_renderer))
{

}

MeshClipperGUI::~MeshClipperGUI()
{
	if (m_should_drawer_delete_in_destructer)
	{
		delete mp_drawer;
	}
}