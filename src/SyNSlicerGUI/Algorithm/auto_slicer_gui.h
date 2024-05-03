#ifndef SYNSLICERENGINE_GUI_AUTOSLICER_H_
#define SYNSLICERENGINE_GUI_AUTOSLICER_H_

#include "SyNSlicerEngine/Algorithm/auto_slicer.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to slice a 3D model automatically with non-parallel planes.
	class AutoSlicerGUI : public SA::AutoSlicer
	{
	public:
		//! Default constructor not allowed.
		AutoSlicerGUI() = delete;

		//! Constructor.
		/*!
			\param p_partition				The partition to be sliced.
			\param p_renderer				The VTK Renderer.
			\param target_layer_thickness	The average layer thickness want to achieve.
			\param side_step				The distance between consecutive paths.
			\param min_layer_thickness		The minimum layer thickness that the printer can print.
			\param max_layer_thickness		The maximum layer thickness that the printer can print.
		*/
		AutoSlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, vtkRenderer *p_renderer,
			double target_layer_thickness = 0.3, double side_step = 0.4, double min_layer_thickness = 0.25,
			double max_layer_thickness = 0.35);

		//! Constructor.
		/*!
			\param p_partition				The partition to be sliced.
			\param p_drawer					The drawer.
			\param target_layer_thickness	The average layer thickness want to achieve.
			\param side_step				The distance between consecutive paths.
			\param min_layer_thickness		The minimum layer thickness that the printer can print.
			\param max_layer_thickness		The maximum layer thickness that the printer can print.
		*/
		AutoSlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, SyNSlicerGUI::ObjectDrawer *p_drawer,
			double target_layer_thickness = 0.3, double side_step = 0.4, double min_layer_thickness = 0.25,
			double max_layer_thickness = 0.35);

		//! Destructor.
		~AutoSlicerGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_AUTOSLICER_H_