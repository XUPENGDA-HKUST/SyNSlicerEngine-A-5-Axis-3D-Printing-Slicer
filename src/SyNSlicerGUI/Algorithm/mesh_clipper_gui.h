#ifndef SYNSLICERGUI_MESHCLIPPER_H_
#define SYNSLICERGUI_MESHCLIPPER_H_

#include "SyNSlicerEngine/Algorithm/mesh_clipper.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to slice a 3D model with already calculated planes.
	/*!
	
	*/
	class MeshClipperGUI : public SA::MeshClipper
	{
	public:
		//! Default constructor.
		MeshClipperGUI(vtkRenderer *p_renderer);

		//! Destructor.
		~MeshClipperGUI();

	protected:
		bool findAllSperatedMeshes(
			std::vector<CgalMesh_EPICK::Face_index> &triangle_contour, 
			const SO::Line &line, 
			const SO::Plane &plane) override;

		bool m_should_drawer_delete_in_destructer;
		ObjectDrawer *mp_drawer;
	};
}

#endif  // SYNSLICERGUI_MESHCLIPPER_H_