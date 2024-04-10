#ifndef SYNSLICERENGINE_ALGORITHM_TOOLPATHGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_TOOLPATHGENERATOR_H_

#include <limits>
#include <vector>

#include <CGAL_CORE_CLASS>

#include <clipper2/clipper.h>

#include "Object/line.h"
#include "Object/printing_layer.h"
#include "Object/polyline_collection.h"
#include "Object/toolpath.h"
#include "Object/toolpath_collection.h"
#include "Object/partition.h"
#include "object_drawer.h"
#include "infill_path_generator.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm {

	//! This class is used to generate toolpath to print a model.
	/*!
		path_type: 1 
	*/
	class ToolpathGenerator
	{
	public:
		ToolpathGenerator(SO::Partition<CgalMesh_EPICK> &partition, bool with_support = false, vtkRenderer *p_renderer = nullptr);
		~ToolpathGenerator();

		void setPathPropertyForModel(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step);
		void setPathPropertyForSupport(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step);
		void generatePath();

	protected:
		void determineOriginOfAllPrintingLayers();
		void determineCuttingPlanesZigzagInfill(SO::PolygonCollection &contours, int index);
		void determineCuttingPlanesGridInfill(SO::PolygonCollection &contours, int infill_density);

		void generateSurfaceForModel();
		void generateWallForModel(int wall_count);
		void generateBottomForModel(int wall_count, int bottom_count);
		void generateTopForModel(int wall_count, int top_count);
		void generateTopBottomUnionAndInfillContoursForModel(int wall_count);
		void generateInfillForModel(int wall_count, int infill_type);
		
		void generateSurfaceForSupport();
		void generateWallForSupport(int wall_count);
		void generateBottomForSupport(int wall_count, int bottom_count);
		void generateTopForSupport(int wall_count, int top_count);
		void generateTopBottomUnionAndInfillContoursForSupport(int wall_count);
		void generateInfillForSupport(int wall_count, int infill_type);
		
		int m_infill_type = 0;
		int m_wall_count = 0;
		int m_bottom_count = 3;
		int m_top_count = 3;
		int m_infill_density = 50;
		bool m_property_for_model_setup;

		int m_infill_type_support = 0;
		int m_wall_count_support = 0;
		int m_bottom_count_support = 3;
		int m_top_count_support = 3;
		int m_infill_density_support = 50;
		bool m_property_for_support_setup;

		SO::PrintingLayer *mp_current_printing_layer;
		SO::Partition<CgalMesh_EPICK> *mp_partition;
		double m_side_step;
		bool m_with_support;

		Eigen::Vector3d m_center_of_infill_cutting_planes;
		std::vector<SO::Plane> m_cutting_planes;

	private:
		Eigen::Vector3d transformPointFromPlaneToPlane(
			const Eigen::Vector3d &point, 
			const SO::Plane &source_plane, 
			const SO::Plane &target_plane);

		GUI::ObjectDrawer m_drawer;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_TOOLPATHGENERATOR_H_