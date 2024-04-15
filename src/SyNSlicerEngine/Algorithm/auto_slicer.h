#ifndef SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_
#define SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_

#include <vector>

#include <clipper2/clipper.h>

#include <CGAL_CORE_CLASS>

#include "Object/polygon.h"
#include <Object/partition.h>
#include <Object/printing_layer_collection.h>
#include <Object/plane.h>

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to slice a model by a b-spline guided non-parallel plane slicer.
	/*!
	
	*/
	class AutoSlicer
	{
	public:
		explicit AutoSlicer(SO::Partition<CgalMesh_EPICK> &p_partition,
			double target_layer_thickness, double side_step);
		~AutoSlicer();

	private:
		bool determineNextSlicingPlane(SO::PolygonCollection &current_contours, SO::PolygonCollection &next_contours);
		bool checkSupportNeeded(SO::PolygonCollection &contours_below, SO::PolygonCollection &contours_up, SO::PolygonCollection &support_contours);

		bool getIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below);
		bool tunePlaneUpUntilMimimumSideValid(SO::Plane &plane_up, SO::Plane plane_below, std::vector<SO::Plane> &slicing_planes);
		bool tuneConsecutivePlanesValid(SO::Plane &plane_up, SO::Plane plane_below);

		bool isSlicingPlaneValid(SO::Plane plane_up, SO::Plane plane_below);

		SO::PolygonCollection slice(SO::Plane plane);

		SO::Partition<CgalMesh_EPICK> *mp_operating_partition;
		SO::PrintingLayerCollection mp_printing_layers;

		CgalMesh_EPICK m_mesh;
		CGAL::Polygon_mesh_slicer<CgalMesh_EPICK, EPICK> m_slicer;
		CgalTree m_tree;

		double m_max_layer_thickness;
		double m_min_layer_thickness;
		double m_layer_thickness;
		double m_side_step;
		std::vector<Eigen::Vector3d> m_points_in_spline;
		SO::Plane m_intermediate_plane;
		std::vector<SO::Plane> m_intermediate_slicing_planes;
		std::vector<SO::Plane> m_slicing_planes;
		std::vector<bool> contour_status;
		std::vector<Eigen::Vector3d> m_points_on_base_plane;
		std::vector<SO::PolygonCollection> m_temp_slicing_result;
		std::vector<SO::PolygonCollection> m_slicing_result;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_