#ifndef SYNSLICERENGINE_GUI_SLICER_H_
#define SYNSLICERENGINE_GUI_SLICER_H_

#include "SyNSlicerEngine/Algorithm/slicer.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to slice a 3D model with already calculated planes.
	class SlicerGUI : public SA::Slicer
	{
	public:
		//! Default constructor not allowed.
		SlicerGUI() = delete;

		//! Constructor.
		/*!
			\param	p_partition	The partition to be sliced.
		*/
		SlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, vtkRenderer *p_renderer);

		//! Constructor.
		/*!
			\param	p_partition	The partition to be sliced.
		*/
		SlicerGUI(SO::Partition<CgalMesh_EPICK> &p_partition, SyNSlicerGUI::ObjectDrawer *p_drawer);

		//! Destructor.
		~SlicerGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_SLICER_H_