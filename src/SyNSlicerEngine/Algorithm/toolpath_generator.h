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
#include "infill_path_generator.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm {

	//! This class is used to generate toolpath to print a model.
	/*!

	*/
	class ToolpathGenerator
	{
	public:
		//!  Default constructor is not allowed.
		ToolpathGenerator() = delete;

		//!  Constructor.
		/*!
			\param	partition		The diameter of the filament used by the printer.
			\param	with_support	Whether generate printing paths for support structure.
		*/
		ToolpathGenerator(SO::Partition<CgalMesh_EPICK> &partition, bool with_support = false);

		//!  Destructor.
		~ToolpathGenerator();

		//! Set properties for printing paths of the 3D model.
		/*!
			\param	wall_count		Number of wall.
			\param	bottom_count	Number of bottom layers.
			\param	top_count		Number of top layers.
			\param	path_type		Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
			\param	infill_density	Density of infill pattern.
			\param	side_step		Distance between consective paths.
		*/
		virtual void setPathPropertyForModel(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step);

		//! Set properties for printing paths of support structure.
		/*!
			\param	wall_count		Number of wall.
			\param	bottom_count	Number of bottom layers.
			\param	top_count		Number of top layers.
			\param	path_type		Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
			\param	infill_density	Density of infill pattern.
			\param	side_step		Distance between consective paths.
		*/
		virtual void setPathPropertyForSupport(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step);

		//! Call to generate paths.
		virtual void generatePath();

	protected:

		//! Determine a point on each layer to align the infill.
		virtual void determineOriginOfAllPrintingLayers();

		//! Determine the planes used to calculate zigzag path.
		/*!
			\param	contours	Contours.
			\param	index		Define planes normal direction. /b 0 or /b 1.
		*/
		virtual void determineCuttingPlanesZigzagInfill(SO::PolygonCollection &contours, int index);

		//! Determine the planes used to calculate grid infill.
		/*!
			\param	contours		Contours.
			\param	infill_density	Density of infill pattern.
		*/
		virtual void determineCuttingPlanesGridInfill(SO::PolygonCollection &contours, int infill_density);

		//! Generate printing path for the surface of the model.
		virtual void generateSurfaceForModel();

		//! Generate printing path for the wall of the model.
		/*!
			\param	wall_count	Number of wall.
		*/
		virtual void generateWallForModel(int wall_count);

		//! Generate printing path for the bottom of the model.
		/*!
			\param	wall_count		Number of wall.
			\param	bottom_count	Number of bottom layers.
		*/
		virtual void generateBottomForModel(int wall_count, int bottom_count);

		//! Generate printing path for the top of the model.
		/*!
			\param	wall_count		Number of wall.
			\param	top_count	Number of top layers. 
		*/
		virtual void generateTopForModel(int wall_count, int top_count);

		//! Generate printing path for the bottom and top of the model.
		/*!
			\param	wall_count		Number of wall.
		*/
		virtual void generateTopBottomUnionAndInfillContoursForModel(int wall_count);

		//! Generate printing path for the infill of the model.
		/*!
			\param	wall_count	Number of wall.
			\param	infill_type	Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
		*/
		virtual void generateInfillForModel(int wall_count, int infill_type);
		
		//! Generate printing path for the surface of support structure.
		virtual void generateSurfaceForSupport();

		//! Generate printing path for the wall of the support structure.
		/*!
			\param	wall_count	Number of wall.
		*/
		virtual void generateWallForSupport(int wall_count);

		//! Generate printing path for the bottom of the support structure.
		/*!
			\param	wall_count		Number of wall.
			\param	bottom_count	Number of bottom layers.
		*/
		virtual void generateBottomForSupport(int wall_count, int bottom_count);

		//! Generate printing path for the top of the model.
		/*!
			\param	wall_count	Number of wall.
			\param	top_count	Number of top layers.
		*/
		virtual void generateTopForSupport(int wall_count, int top_count);

		//! Generate printing path for the bottom and top of the model.
		/*!
			\param	wall_count	Number of wall.
		*/
		virtual void generateTopBottomUnionAndInfillContoursForSupport(int wall_count);

		//! Generate printing path for the infill of the model.
		/*!
			\param	wall_count	Number of wall.
			\param	infill_type	Infill pattern. 0: Contour parallel. 1: Zigzag. 2: Grid
		*/
		virtual void generateInfillForSupport(int wall_count, int infill_type);
		
		//! Transform a point from one plane to another plane.
		/*!
			\param	point			The point.
			\param	source_plane	Source plane.
			\param	target_plane	Target plane.
		*/
		virtual Eigen::Vector3d transformPointFromPlaneToPlane(
			const Eigen::Vector3d &point, const SO::Plane &source_plane, const SO::Plane &target_plane);

		//! Infill pattern for model. 0: Contour parallel. 1: Zigzag. 2: Grid.
		int m_infill_type = 0;

		//! Number of wall for model.
		int m_wall_count = 0;

		//! Number of bottom layers for model.
		int m_bottom_count = 3;

		//! Number of top layers for model.
		int m_top_count = 3;

		//! Infill density of the infill path for model.
		int m_infill_density = 50;

		//! Is properties of the printing paths for model set up.
		bool m_property_for_model_setup;

		//! Infill pattern for support. 0: Contour parallel. 1: Zigzag. 2: Grid.
		int m_infill_type_support = 0;

		//! Number of wall for support.
		int m_wall_count_support = 0;

		//! Number of bottom layers for support.
		int m_bottom_count_support = 3;

		//! Number of top layers for support.
		int m_top_count_support = 3;

		//! Infill density of the infill path for support.
		int m_infill_density_support = 50;

		//! Is properties of the printing paths for support set up.
		bool m_property_for_support_setup;

		//! Current operating printing layer in methods.
		SO::PrintingLayer *mp_current_printing_layer;

		//! Operating partition.
		SO::Partition<CgalMesh_EPICK> *mp_partition;

		//! Distance between consecutive paths.
		double m_side_step;

		//! Whether generate printing paths for support structure.
		bool m_with_support;

		//! Point used to align infill paths.
		Eigen::Vector3d m_center_of_infill_cutting_planes;

		//! Planes used to generate infill paths.
		std::vector<SO::Plane> m_cutting_planes;


	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_TOOLPATHGENERATOR_H_