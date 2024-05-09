#ifndef SYNSLICERENGINE_ALGORITHM_GCODEGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_GCODEGENERATOR_H_

#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include "Object/plane.h"
#include "Object/point_cloud.h"
#include "Object/partition.h"
#include "Object/partition_collection.h"
#include "Object/toolpath.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm {

	//!  This class is used to generate gcode
	/*!
		Make sure all the contours are closed.
	*/
	class GcodeGenerator
	{
	public:
		//! Default constructure is not allowed.
		GcodeGenerator() = delete;

		//! Constructor
		/*!
			\param[in] partitions All partitions to be printed.
			\param[in] side_step Distance between consecutive paths.
		*/
		GcodeGenerator(SO::PartitionCollection<CgalMesh_EPICK> partitions, double side_step = 0.4);

		//! Destructor
		~GcodeGenerator();

		//! Call to generate toolpath.
		virtual SO::Toolpath getCompletedToolpath();

		//! Call to generate toolpath for each printing layer.
		virtual void generateToolpathForEachLayer();

		//! Call to connect all toolpath in each layer.
		virtual void generateCompletedToolpath();
		
		//! Write the Gcode file
		/*!
			\param filament_diameter The diameter of the filament used by the printer.
		*/
		virtual void writeGcode(double filament_diameter = 1.75);

		//! Write the Gcode file
		/*!
			\param file_name			The location where the file should be saved.
			\param filament_diameter	The diameter of the filament used by the printer.
		*/
		virtual void writeGcode(std::string file_name, double filament_diameter = 1.75);

	protected:
		//! Genearte Tool path for parallel printing layers.
		/*!
			\param polygons			Contours.
			\param toolpaths		Toolpaths.
			\param layer_thickness	Layer thicknees used to calculate extrusion.
		*/
		virtual void generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, double layer_thickness);

		//! Genearte Tool path for parallel printing layers.
		/*!
			\param polygons_collection	Contours.
			\param toolpaths			Toolpaths.
			\param layer_thickness		Layer thicknees used to calculate extrusion.
		*/
		virtual void generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, double layer_thickness);

		//! Genearte Tool path for non parallel printing layers.
		/*!
			\param polygons				Contours.
			\param toolpaths			Toolpaths.
			\param intersecting_line	Intersecting line of two consecutive printing layers.
			\param angle_between_planes	Angle between two consecutive printing layers.
		*/
		virtual void generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes);
		
		//! Genearte Tool path for non parallel printing layers.
		/*!
			\param polygons_collection	Contours.
			\param toolpaths			Toolpaths.
			\param intersecting_line	Intersecting line of two consecutive printing layers.
			\param angle_between_planes	Angle between two consecutive printing layers.
		*/
		virtual void generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes);

		//! Computer extrustion at every point in the toolpath.
		/*!
			\param filament_diameter	Diameter of the filament used by the printer.
		*/
		virtual void computeExtrusion(double filament_diameter = 1.75);

		//! Offset a AABB.
		/*!
			\param bound			The bounding box.
			\param offset_distance	Offset distance.
		*/
		virtual void offsetBoundingBox(double bound[6], double offset_distance);

		//! Find closest intersecting point of a ray and a bounding box.
		/*!
			\param		line	The ray.
			\param		bound	The bounding box.
			\param[out] point	The intersecting point.
		*/
		virtual int findClosestIntersectionPointOfRayAndBoundingBox(const SO::Line &line, double bound[6], Eigen::Vector3d &point);

		//! All partitions to be printed.
		SO::PartitionCollection<CgalMesh_EPICK> m_partitions;

		//! Distance between consecutive paths.
		double m_side_step;	

		//! The toolpath.
		SO::Toolpath m_completed_toolpath;
	};

}

#endif  // SYNSLICERENGINE_ALGORITHM_GCODEGENERATOR_H_