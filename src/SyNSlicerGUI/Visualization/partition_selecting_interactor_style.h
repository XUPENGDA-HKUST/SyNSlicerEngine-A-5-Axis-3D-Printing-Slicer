#ifndef SYNSLICERGUI_PARTITIONSELECTINGINTERACTORSTYLE_H_
#define SYNSLICERGUI_PARTITIONSELECTINGINTERACTORSTYLE_H_

#include "SyNSlicerEngine/Object/partition_collection.h"

#include "vtkPropPicker.h"

#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;

//!  A namespace used to store the Class related to Visualization.
namespace SyNSlicerGUI
{
	//! This class is used to enable user to select the partitions they want to keep.
	class PartitionSelectingInteractorStyle : public vtkInteractorStyleTrackballCamera
	{
	public:
		PartitionSelectingInteractorStyle(vtkRenderer *p_renderer);
		~PartitionSelectingInteractorStyle() override;

		virtual void OnLeftButtonDown() override;

		void setPartitions(SO::PartitionCollection<CgalMesh_EPICK> &partitions, SO::Plane base_plane);
		bool confirmSelection(SO::PartitionCollection<CgalMesh_EPICK> &partitions);

	protected:

		SO::Plane m_base_plane;
		SO::PartitionCollection<CgalMesh_EPICK> *mp_partitions;
		std::vector<vtkSmartPointer<vtkProperty>> m_property_list;
		std::vector<CgalPolyline_EPICK> m_contours;

		SO::Partition<CgalMesh_EPICK> *mp_model_cannot_be_selected;

		SO::PartitionCollection<CgalMesh_EPICK> m_result_partitions;

		vtkSmartPointer<vtkProperty> m_property_for_selected_partition;
		vtkActor *LastPickedActor;
		vtkRenderer *mp_renderer;

		std::map<vtkActor *, int> m_mapping_actor_to_partition_list;
		std::map<vtkActor *, int> m_mapping_actor_to_number_of_times_clicked;

		int m_current_key;

		bool m_should_drawer_delete_in_destructer;
		ObjectDrawer *mp_drawer;
	};


}

#endif  // SYNSLICERGUI_PARTITIONSELECTINGINTERACTORSTYLE_H_