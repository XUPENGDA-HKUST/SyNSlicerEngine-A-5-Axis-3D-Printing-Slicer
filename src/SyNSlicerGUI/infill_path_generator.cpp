#include "infill_path_generator.h"

using SyNSlicerEngine::Algorithm::InfillPathGenerator;

InfillPathGenerator::InfillPathGenerator(
	const SO::PolygonCollection &contours,
	const std::vector<SO::Plane> &cutting_planes,
	double side_step,
	int infill_type,
	vtkRenderer *renderer)
	: m_contours(contours)
	, m_cutting_planes(cutting_planes)
	, m_side_step(side_step)
	, m_infill_type(infill_type)
	, m_is_inputs_valid(true)
	, m_cgal_outer_contour(CgalPolygon2D_EPICK())
	, m_cgal_inner_contours(std::vector<CgalPolygon2D_EPICK>())
	, m_output(SO::PolygonCollection())
	, m_drawer(renderer)
{
	for (size_t i = 0; i < m_contours.numberOfPolygons(); i++)
	{
		if (m_contours[i].isClockWise() == 1)
		{
			clockwise_contours.addPolygon(m_contours[i]);
		}
		else if (m_contours[i].isClockWise() == -1)
		{
			counter_clockwise_contours.addPolygon(m_contours[i]);
		}
		else
		{
			spdlog::error("Polygon not simple {}", i);
			m_drawer.drawPolygon(m_contours[i], "error");
		}
	}

	if (counter_clockwise_contours.numberOfPolygons() < 1)
	{
		m_is_inputs_valid = false;
	}

	if (clockwise_contours.numberOfPolygons() > 0)
	{
		spdlog::error("Input contours have clockwise contour but no counter clockwise contour!");
	}
}

InfillPathGenerator::~InfillPathGenerator()
{
}

void InfillPathGenerator::generateInfillPath()
{
	switch (m_infill_type)
	{
	case 0:
		generateContourParallelInfillPath();
		break;
	case 1:
		generateZigZagInfillPath();
		break;
	case 2:
		generateGridInfillPath();
		break;
	default:
		break;
	}
}

void InfillPathGenerator::getOutput(SO::PolygonCollection &ouput)
{
	if (m_is_inputs_valid == false)
	{
		//spdlog::error("No output can be provided because the inputs are not valid!");
		return;	
	}
	ouput = m_output;
}

static int a = 0;

void InfillPathGenerator::generateGridInfillPath()
{
	if (m_is_inputs_valid == false)
	{
		//spdlog::error("Generation will not be performed because the inputs are not valid!");
		return;
	}

	SO::Polygon temp_polygon;
	temp_polygon.setPlane(m_contours.getPlane());

	int cutting_plane_index = 0;
	for (auto &cutting_plane : m_cutting_planes)
	{
		std::vector<Eigen::Vector3d> intersecting_points;
		for (size_t i = 0; i < counter_clockwise_contours.numberOfPolygons(); i++)
		{
			counter_clockwise_contours[i].isIntersectedWithPlane(cutting_plane, intersecting_points);
		}

		for (size_t i = 0; i < clockwise_contours.numberOfPolygons(); i++)
		{
			clockwise_contours[i].isIntersectedWithPlane(cutting_plane, intersecting_points);
		}

		// Arrange the points inside intersecting_points

		if (intersecting_points.size())
		{
			Eigen::Vector3d vector = cutting_plane.getNormal().cross(m_contours.getPlane().getNormal());
			vector = vector / vector.norm();
			Eigen::Vector3d origin = cutting_plane.getOrigin();

			// sorting the points so that the polyline joining them will not overlap itself
			std::sort(intersecting_points.begin(), intersecting_points.end(), [vector, origin](Eigen::Vector3d a, Eigen::Vector3d b) {
				double distance_a = (a - origin).dot(vector);
				double distance_b = (b - origin).dot(vector);
				return distance_a < distance_b;
				});

			int j = 0;
			for (int i = 0; i < intersecting_points.size(); i++)
			{
				SO::Line temp_line(intersecting_points[j], intersecting_points[i]);

				if (temp_line.getLength() > 1e-6)
				{
					int count = 0;
					for (int polyline_index = 0; polyline_index < counter_clockwise_contours.numberOfPolygons(); polyline_index++)
					{
						if (counter_clockwise_contours[polyline_index].isLineInside(temp_line))
						{
							count++;
						}
					}

					for (int polyline_index = 0; polyline_index < clockwise_contours.numberOfPolygons(); polyline_index++)
					{
						if (clockwise_contours[polyline_index].isLineInside(temp_line))
						{
							count--;
						}
					}

					if (count == 1)
					{
						temp_polygon.addPointToBack(intersecting_points[j]);
						temp_polygon.addPointToBack(intersecting_points[i]);
						m_output.addPolygon(temp_polygon);
						temp_polygon.reset();
						temp_polygon.setPlane(m_contours.getPlane());
					}
				}
				j = i;
			}
		}
		++cutting_plane_index;
	}
}

void InfillPathGenerator::generateZigZagInfillPath()
{
	SO::Polygon temp_polygon;
	temp_polygon.setPlane(m_contours.getPlane());
	int cutting_plane_index = 0;
	for (auto &cutting_plane : m_cutting_planes)
	{
		std::vector<Eigen::Vector3d> intersecting_points;
		for (size_t i = 0; i < counter_clockwise_contours.numberOfPolygons(); i++)
		{
			counter_clockwise_contours[i].isIntersectedWithPlane(cutting_plane, intersecting_points);
		}

		for (size_t i = 0; i < clockwise_contours.numberOfPolygons(); i++)
		{
			clockwise_contours[i].isIntersectedWithPlane(cutting_plane, intersecting_points);
		}

		// Arrange the points inside intersecting_points

		if (intersecting_points.size())
		{
			Eigen::Vector3d vector = cutting_plane.getNormal().cross(m_contours.getPlane().getNormal());
			vector = vector / vector.norm();
			Eigen::Vector3d origin = cutting_plane.getOrigin();

			// sorting the points so that the polyline joining them will not overlap itself
			std::sort(intersecting_points.begin(), intersecting_points.end(), [vector, origin](Eigen::Vector3d a, Eigen::Vector3d b) {
				double distance_a = (a - origin).dot(vector);
				double distance_b = (b - origin).dot(vector);
				return distance_a < distance_b;
				});

			int j = 0;
			for (int i = 0; i < intersecting_points.size(); i++)
			{
				Object::Line temp_line(intersecting_points[j], intersecting_points[i]);

				if (temp_line.getLength() > 1e-6)
				{
					int count = 0;
					for (int polyline_index = 0; polyline_index < counter_clockwise_contours.numberOfPolygons(); polyline_index++)
					{
						if (counter_clockwise_contours[polyline_index].isLineInside(temp_line))
						{
							count++;
						}
					}

					for (int polyline_index = 0; polyline_index < clockwise_contours.numberOfPolygons(); polyline_index++)
					{
						if (clockwise_contours[polyline_index].isLineInside(temp_line))
						{
							count--;
						}
					}

					if (count == 1)
					{
						temp_polygon.addPointToBack(intersecting_points[j]);
						temp_polygon.addPointToBack(intersecting_points[i]);
						m_output.addPolygon(temp_polygon);
						temp_polygon.reset();
						temp_polygon.setPlane(m_contours.getPlane());
					}
				}
				j = i;
			}
		}
		++cutting_plane_index;
	}
}

void InfillPathGenerator::generateContourParallelInfillPath()
{
	SO::PolygonCollection temp_polygons = m_contours;
	while (temp_polygons.numberOfPolygons())
	{
		temp_polygons = temp_polygons.getOffset(-m_side_step);
		temp_polygons.closePolygons();
		m_output.addPolygons(temp_polygons);
	}
}
