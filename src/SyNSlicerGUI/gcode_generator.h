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
		Input:		Partition list, from top to down \n
		Output:		.txt G-code file. \n
	*/
	class GcodeGenerator
	{
	public:
		GcodeGenerator(SO::PartitionCollection<CgalMesh_EPICK> partitions, double side_step = 0.4);
		~GcodeGenerator();

		SO::Toolpath getCompletedToolpath();

		void generateToolpathForEachLayer();
		void generateCompletedToolpath();
		
		//!  Write the Gcode file
		/*!
			\param filament_diameter The diameter of the filament used by the printer.
		*/
		void writeGcode(double filament_diameter = 1.75);

	private:
		void generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, double layer_thickness);
		void generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, double layer_thickness);
		void generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes);
		void generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes);

		void computeExtrusion(double filament_diameter = 1.75);

		void offsetBoundingBox(double bound[6], double offset_distance);
		int findClosestIntersectionPointOfRayAndBoundingBox(const SO::Line &line, double bound[6], Eigen::Vector3d &point);

		SO::PartitionCollection<CgalMesh_EPICK> m_partitions;
		double m_side_step;	

		SO::Toolpath m_completed_toolpath;
	};

}

#endif  // SYNSLICERENGINE_ALGORITHM_GCODEGENERATOR_H_