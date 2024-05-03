#include "gcode_generator.h"

using SyNSlicerEngine::Algorithm::GcodeGenerator;

GcodeGenerator::GcodeGenerator(SO::PartitionCollection<CgalMesh_EPICK> partitions, double side_step)
	: m_partitions(partitions)
	, m_side_step(side_step)
{

}

GcodeGenerator::~GcodeGenerator()
{

}

SO::Toolpath GcodeGenerator::getCompletedToolpath()
{
	return m_completed_toolpath;
}

void GcodeGenerator::generateToolpathForEachLayer()
{
	SO::Line intersecting_line;
	double angle_between_planes = 0.0;
	double local_layer_thickness = 0.0;

	for (auto &partition : m_partitions.get())
	{
		for (int i = 0; i < partition.getPrintingLayers().size(); i++)
		{
			SO::PrintingLayer &current_layer = partition.getPrintingLayers()[i];
			SO::ToolpathCollection toolpaths;
			
			if (current_layer.getSlicingPlane().isIntersectedWithPlane(current_layer.getPrevSlicingPlane(), intersecting_line))
			{
				// non parallel consecutive layers
				angle_between_planes = abs(current_layer.getSlicingPlane().getAngleOfRotation(current_layer.getPrevSlicingPlane())); //????
				this->generateToolpath(current_layer.getPrintingPaths().getSurface(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPaths().getWall(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPaths().getBottomTopUnion(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPaths().getInfill(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getSurface(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getWall(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getBottomTopUnion(), toolpaths, intersecting_line, angle_between_planes);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getInfill(), toolpaths, intersecting_line, angle_between_planes);
			}
			else
			{
				// parallel consecutive layers
				local_layer_thickness = abs(current_layer.getSlicingPlane().d() - current_layer.getPrevSlicingPlane().d())
					/ current_layer.getPrevSlicingPlane().getNormal().norm();

				this->generateToolpath(current_layer.getPrintingPaths().getSurface(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPaths().getWall(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPaths().getBottomTopUnion(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPaths().getInfill(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getSurface(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getWall(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getBottomTopUnion(), toolpaths, local_layer_thickness);
				this->generateToolpath(current_layer.getPrintingPathsForSupport().getInfill(), toolpaths, local_layer_thickness);
			}
			current_layer.setToolpaths(toolpaths);
		}
	}
}

void GcodeGenerator::generateCompletedToolpath()
{
	SO::PointCloud point_cloud;
	
	for (int partition_index = 0; partition_index < m_partitions.numberOfPartitions(); partition_index++)
	{
		SO::Partition<CgalMesh_EPICK> &current_parition = m_partitions[partition_index];

		for (int printing_layer_index = 0; printing_layer_index < current_parition.getPrintingLayers().getNumberOfLayers(); printing_layer_index++)
		{
			const SO::PrintingLayer &current_layer = current_parition.getPrintingLayers()[printing_layer_index];

			for (int toolpath_index = 0; toolpath_index < current_layer.getToolpaths().size(); toolpath_index++)
			{
				const SO::Toolpath &current_toolpath = current_layer.getToolpaths()[toolpath_index];
				for (int tool_point_index = 0; tool_point_index < current_toolpath.size(); tool_point_index++)
				{
					m_completed_toolpath.addToolPoint(current_toolpath[tool_point_index]);
					point_cloud.addPoint(current_toolpath[tool_point_index].getPosition());
				}
			}

			// Rule of joining layers
			if (printing_layer_index < current_parition.getPrintingLayers().getNumberOfLayers() - 1)
			{
				SO::ToolPoint temp(m_completed_toolpath.back());
				temp.setPosition(temp.getPosition() + 0.3 * temp.getNormal());
				temp.setExtrusion(0.0);
				m_completed_toolpath.addToolPoint(temp);
				point_cloud.addPoint(temp.getPosition());
			}
		}

		// Rule of joining partitions
		if (partition_index < m_partitions.numberOfPartitions() - 1)
		{
			Eigen::Vector3d origin_0(m_completed_toolpath.back()[0], m_completed_toolpath.back()[1], m_completed_toolpath.back()[2]);
			Eigen::Vector3d normal_0(m_completed_toolpath.back()[3], m_completed_toolpath.back()[4], m_completed_toolpath.back()[5]);

			SO::ToolPoint first_tool_point_in_next_partition(m_partitions[partition_index + 1].getPrintingLayers()[0].getToolpaths()[0][0]);

			Eigen::Vector3d origin_1(first_tool_point_in_next_partition[0], first_tool_point_in_next_partition[1], first_tool_point_in_next_partition[2]);
			Eigen::Vector3d normal_1(first_tool_point_in_next_partition[3], first_tool_point_in_next_partition[4], first_tool_point_in_next_partition[5]);

			double bound[6];
			point_cloud.getBound(bound);

			this->offsetBoundingBox(bound, 10 * m_completed_toolpath.back()[7]);

			SO::Line line0(origin_0, normal_0, 1.0);
			Eigen::Vector3d transitional_path_pt0;
			this->findClosestIntersectionPointOfRayAndBoundingBox(line0, bound, transitional_path_pt0);

			SO::Line line1(origin_1, normal_1, 1.0);
			Eigen::Vector3d transitional_path_pt3;
			this->findClosestIntersectionPointOfRayAndBoundingBox(line1, bound, transitional_path_pt3);

			SO::Plane plane0(Eigen::Vector3d(0, 0, bound[5]), Eigen::Vector3d(0, 0, 1));  // +z bounding plane
			Eigen::Vector3d transitional_path_pt1;
			transitional_path_pt1 = plane0.getProjectionOfPointOntoPlane(transitional_path_pt0);

			Eigen::Vector3d transitional_path_pt2;
			transitional_path_pt2 = plane0.getProjectionOfPointOntoPlane(transitional_path_pt3);

			// Calculate ToolPoints

			SO::ToolPoint tp0(Eigen::Vector3d(transitional_path_pt0[0], transitional_path_pt0[1], transitional_path_pt0[2]), Eigen::Vector3d(normal_0[0], normal_0[1], normal_0[2]), 0, m_completed_toolpath.back()[7]);
			SO::ToolPoint tp1(Eigen::Vector3d(transitional_path_pt1[0], transitional_path_pt1[1], transitional_path_pt1[2]), Eigen::Vector3d(0, 0, 1), 0, m_completed_toolpath.back()[7]);
			SO::ToolPoint tp2(Eigen::Vector3d(transitional_path_pt2[0], transitional_path_pt2[1], transitional_path_pt2[2]), Eigen::Vector3d(0, 0, 1), 0, m_completed_toolpath.back()[7]);
			SO::ToolPoint tp3(Eigen::Vector3d(transitional_path_pt3[0], transitional_path_pt3[1], transitional_path_pt3[2]), Eigen::Vector3d(normal_1[0], normal_1[1], normal_1[2]), 0, m_completed_toolpath.back()[7]);

			m_completed_toolpath.addToolPoint(tp0);
			m_completed_toolpath.addToolPoint(tp1);
			m_completed_toolpath.addToolPoint(tp2);
			m_completed_toolpath.addToolPoint(tp3);
		}
	}
}

void GcodeGenerator::computeExtrusion(double filament_diameter)
{	
	double extrusion = 0.0;
	double distance = 0.0;
	for (int point_idx = 0; point_idx < m_completed_toolpath.size(); point_idx++)
	{
		if (m_completed_toolpath[point_idx][6] == 0.0)
		{
			// equal 0 means there is no extrusion from the prev point to this point
			m_completed_toolpath.setExtrusion(point_idx, extrusion);
		}
		else
		{
			const Eigen::Vector3d &p1 = m_completed_toolpath[point_idx].getPosition();
			const Eigen::Vector3d &p0 = m_completed_toolpath[point_idx - 1].getPosition();

			double area = (m_completed_toolpath[point_idx][7] + m_completed_toolpath[point_idx - 1][7])
				* (p1 - p0).norm() * 0.5; // area of trapezium
			double volume = area * m_side_step;

			distance = distance + (p1 - p0).norm();
			extrusion = extrusion + volume / (M_PI * pow(filament_diameter / 2, 2));

			m_completed_toolpath.setExtrusion(point_idx, extrusion);
		}
	}

	spdlog::info("Total length: {}, Total extrusion: {}", distance, extrusion);

}

void GcodeGenerator::writeGcode(double filament_diameter)
{
	computeExtrusion(filament_diameter);
	std::ofstream myfile;
	myfile.open("Gcode.txt");
	for (int point_idx = 0; point_idx < m_completed_toolpath.size(); point_idx++)
	{
		myfile << "X" << m_completed_toolpath[point_idx][0];
		myfile << " Y" << m_completed_toolpath[point_idx][1];
		myfile << " Z" << m_completed_toolpath[point_idx][2];
		myfile << " NX" << m_completed_toolpath[point_idx][3];
		myfile << " NY" << m_completed_toolpath[point_idx][4];
		myfile << " NZ" << m_completed_toolpath[point_idx][5];
		myfile << " E" << m_completed_toolpath[point_idx][6];
		myfile << "\n";
	}
	myfile.close();	
}

void GcodeGenerator::generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, double layer_thickness)
{
	for (auto &polygon : polygons.get())
	{
		SO::Toolpath toolpath;
		for (auto &point : polygon.get())
		{
			SO::ToolPoint temp_tool_point(point, polygons.getPlane().getNormal(), 1.0, layer_thickness);
			toolpath.addToolPoint(temp_tool_point);
		}
		// Joining starting point and ending point
		SO::ToolPoint temp_tool_point = toolpath[0];
		temp_tool_point.setExtrusion(0.0); // Setting the first point of the contour has no extrusion
		toolpath.setToolPoint(0, temp_tool_point);
		toolpaths.addToolPath(toolpath);
	}
}

void GcodeGenerator::generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, double layer_thickness)
{
	for (auto &polygons : polygons_collection)
	{
		for (auto &polygon : polygons.get())
		{
			SO::Toolpath toolpath;
			for (auto &point : polygon.get())
			{
				SO::ToolPoint temp_tool_point(point, polygon.getPlane().getNormal(), 1.0, layer_thickness);
				toolpath.addToolPoint(temp_tool_point);
			}
			// Joining starting point and ending point
			SO::ToolPoint temp_tool_point = toolpath[0];
			temp_tool_point.setExtrusion(0.0); // Setting the first point of the contour has no extrusion
			toolpath.setToolPoint(0, temp_tool_point);
			toolpaths.addToolPath(toolpath);
		}
	}
}

void GcodeGenerator::generateToolpath(const SO::PolygonCollection &polygons, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes)
{
	double local_layer_thickness = 0.0;
	for (auto &polygon : polygons.get())
	{
		SO::Toolpath toolpath;
		for (auto &point : polygon.get())
		{
			local_layer_thickness = intersecting_line.getDistanceOfPoint(point);
			local_layer_thickness = local_layer_thickness * tan(angle_between_planes);
			SO::ToolPoint temp_tool_point(point, polygons.getPlane().getNormal(), 1.0, local_layer_thickness);
			toolpath.addToolPoint(temp_tool_point);
		}
		// Joining starting point and ending point
		SO::ToolPoint temp_tool_point = toolpath[0];
		temp_tool_point.setExtrusion(0.0); // Setting the first point of the contour has no extrusion
		toolpath.setToolPoint(0, temp_tool_point);
		toolpaths.addToolPath(toolpath);
	}
}

void GcodeGenerator::generateToolpath(const std::vector<SO::PolygonCollection> &polygons_collection, SO::ToolpathCollection &toolpaths, SO::Line intersecting_line, double angle_between_planes)
{
	double local_layer_thickness = 0.0;
	for (auto &polygons : polygons_collection)
	{
		for (auto &polygon : polygons.get())
		{
			SO::Toolpath toolpath;
			for (auto &point : polygon.get())
			{
				local_layer_thickness = intersecting_line.getDistanceOfPoint(point);
				local_layer_thickness = local_layer_thickness * tan(angle_between_planes);
				SO::ToolPoint temp_tool_point(point, polygon.getPlane().getNormal(), 1.0, local_layer_thickness);
				toolpath.addToolPoint(temp_tool_point);
			}
			// Joining starting point and ending point
			SO::ToolPoint temp_tool_point = toolpath[0];
			temp_tool_point.setExtrusion(0.0); // Setting the first point of the contour has no extrusion
			toolpath.setToolPoint(0, temp_tool_point);
			toolpaths.addToolPath(toolpath);
		}
	}
}

void GcodeGenerator::offsetBoundingBox(double bound[6], double offset_distance)
{
	bound[0] = bound[0] - offset_distance; // -x;
	bound[1] = bound[1] + offset_distance; // +x;
	bound[2] = bound[2] - offset_distance; // -y;
	bound[3] = bound[3] + offset_distance; // +y;
	bound[4] = bound[4] - offset_distance; // -z;
	bound[5] = bound[5] + offset_distance; // +z;
}

int GcodeGenerator::findClosestIntersectionPointOfRayAndBoundingBox(const SO::Line &line, double bound[6], Eigen::Vector3d &point)
{
	std::vector<SO::Plane> planes;
	planes.reserve(6);

	planes.emplace_back(SO::Plane(Eigen::Vector3d(bound[0], 0, 0), Eigen::Vector3d(-1, 0, 0)));
	planes.emplace_back(SO::Plane(Eigen::Vector3d(bound[1], 0, 0), Eigen::Vector3d(1, 0, 0)));
	planes.emplace_back(SO::Plane(Eigen::Vector3d(0, bound[2], 0), Eigen::Vector3d(0, -1, 0)));
	planes.emplace_back(SO::Plane(Eigen::Vector3d(0, bound[3], 0), Eigen::Vector3d(0, 1, 0)));
	planes.emplace_back(SO::Plane(Eigen::Vector3d(0, 0, bound[4]), Eigen::Vector3d(0, 0, -1)));
	planes.emplace_back(SO::Plane(Eigen::Vector3d(0, 0, bound[5]), Eigen::Vector3d(0, 0, 1)));

	Eigen::Vector3d temp_point;
	double distance = 0.0;
	bool first_time = true;
	int index_to_be_returned;
	for (int i = 0; i < 6; i++)
	{
		if (line.getDirection().dot(planes[i].getNormal()) <= 0.0 || i == 4)
		{
			continue;
		}
		else
		{
			planes[i].isIntersectedWithRay(line, temp_point);
			if (first_time)
			{
				distance = (temp_point - line.getSource()).norm();
				point = temp_point;
				index_to_be_returned = i;
				first_time = false;
			}
			else
			{
				if ((temp_point - line.getSource()).norm() < distance)
				{
					distance = (temp_point - line.getSource()).norm();
					point = temp_point;
					index_to_be_returned = i;
				}
			}
		}
	}
	return index_to_be_returned;
}