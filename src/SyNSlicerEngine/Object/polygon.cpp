#include "polygon.h"

using namespace SyNSlicerEngine::Object;

Polygon::Polygon()
	: m_plane(SO::Plane())
{

}

Polygon::Polygon(const Polygon &other)
{
	*this = other;
}

Polygon::~Polygon()
{

}

int Polygon::numberOfPoints() const
{
	return m_polygon.size();
}

const std::vector<Eigen::Vector3d> &Polygon::get() const
{
	return m_polygon;
}

void Polygon::setPlane(const Plane &plane)
{
	m_plane = plane;
}

const Plane &Polygon::getPlane() const
{
	return m_plane;
}

void Polygon::reset()
{
	*this = Polygon();
}

Eigen::Vector3d Polygon::centroid() const
{
	if (m_polygon.size() == 0)
	{
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}

	if (m_polygon.size() == 1)
	{
		return m_polygon[0];
	}

	if (m_polygon.size() == 2)
	{
		return (m_polygon[0] + m_polygon[1]) / 2;
	}

	std::vector<Eigen::Vector3d> new_polygon = m_polygon;
	new_polygon.emplace_back(new_polygon.front());

	Eigen::Vector3d origin(new_polygon.front());
	std::vector<Eigen::Vector3d> centroids;
	std::vector<Eigen::Vector3d> sign_areas;

	for (auto &point : new_polygon)
	{
		point = point - origin;
	}

	int j = 0;
	for (int i = 1; i < new_polygon.size(); i++)
	{
		Eigen::Vector3d centroid(0.0, 0.0, 0.0);
		Eigen::Vector3d sign_area = new_polygon[j].cross(new_polygon[i]);
		sign_areas.emplace_back(sign_area);
		if (sign_areas.back().norm() < 1e-6)
		{
			centroids.emplace_back(centroid);
		}
		else
		{
			centroid = new_polygon[i] + new_polygon[j];
			centroid = (1.0 / 3.0) * centroid;
			centroids.emplace_back(centroid);
		}
		j = i;
	}

	Eigen::Vector3d total_sign_area(0.0, 0.0, 0.0);
	for (size_t i = 0; i < sign_areas.size(); i++)
	{
		total_sign_area += sign_areas[i];
	}

	double numerator_x = 0.0;
	double numerator_y = 0.0;
	double numerator_z = 0.0;
	for (size_t i = 0; i < centroids.size(); i++)
	{
		if (sign_areas[i].dot(m_plane.getNormal()) < 0.0)
		{
			numerator_x -= centroids[i][0] * sign_areas[i].norm();
			numerator_y -= centroids[i][1] * sign_areas[i].norm();
			numerator_z -= centroids[i][2] * sign_areas[i].norm();
		}
		else
		{
			numerator_x += centroids[i][0] * sign_areas[i].norm();
			numerator_y += centroids[i][1] * sign_areas[i].norm();
			numerator_z += centroids[i][2] * sign_areas[i].norm();
		}
	}

	double x = numerator_x / total_sign_area.norm();
	double y = numerator_y / total_sign_area.norm();
	double z = numerator_z / total_sign_area.norm();

	Eigen::Vector3d result(x, y, z);
	result = result + origin;
	assert(abs(m_plane.getDistanceFromPointToPlane(result)) < 1e-3);
	return result;
}

double Polygon::area() const
{
	std::vector<Eigen::Vector3d> new_polygon = m_polygon;
	new_polygon.emplace_back(new_polygon.front());
	Eigen::Vector3d sign_area(0.0, 0.0, 0.0);
	int j = 0;
	for (int i = 1; i < new_polygon.size(); i++)
	{
		sign_area += new_polygon[j].cross(new_polygon[i]);
		j = i;
	}

	return 0.5 * sign_area.norm();
}

double Polygon::length() const
{
	double length = 0.0;
	std::vector<Eigen::Vector3d> new_polygon = m_polygon;
	new_polygon.emplace_back(new_polygon.front());

	int j = 0;
	for (int i = 1; i < new_polygon.size(); i++)
	{
		length += (new_polygon[i] - new_polygon[j]).norm();
		j = i;
	}

	return length;
}

bool Polygon::isIntersectedWithPlane(const SO::Plane &plane)
{
	bool has_points_on_positive_side = false;
	bool has_points_on_negative_side = false;
	for (auto &point : m_polygon)
	{
		if (plane.getDistanceFromPointToPlane(point) > 0.0)
		{
			has_points_on_positive_side = true;
		}
		else if (plane.getDistanceFromPointToPlane(point) < 0.0)
		{
			has_points_on_negative_side = true;
		}

		if (has_points_on_positive_side && has_points_on_negative_side)
		{
			return true;
		}
	}
	return false;
}

bool Polygon::isPointInside(const Eigen::Vector3d &point)
{
	if (!m_cgal_polygon.size())
	{
		Polygon transformed_polygon = *this;
		Plane xy_plane = Plane(m_plane.getOrigin(), Eigen::Vector3d(0, 0, 1));
		transformed_polygon = transformed_polygon.getTransformedPolygon(xy_plane);
		for (auto &point : transformed_polygon.get())
		{
			m_cgal_polygon.push_back(CgalPoint2D_EPICK(point[0], point[1]));
		}
	}

	Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
	transformation_matrix = this->computeTransformationMatrix(m_plane.getNormal(), Eigen::Vector3d(0, 0, 1));

	Eigen::Vector3d temp_point = m_plane.getProjectionOfPointOntoPlane(point);
	temp_point = transformation_matrix * (temp_point - m_plane.getOrigin());

	CgalPoint2D_EPICK cgal_point(temp_point[0], temp_point[1]);

	if (CGAL::bounded_side_2(m_cgal_polygon.begin(), m_cgal_polygon.end(), cgal_point, EPICK())
		== CGAL::ON_BOUNDED_SIDE)
	{
		return true;
	}
	
	return false;
}

bool Polygon::isOneOfTheVerticesOfTriangleInside(const SO::Triangle &triangle)
{
	bool result = false;
	result = result || this->isPointInside(triangle.m_v0);
	result = result || this->isPointInside(triangle.m_v1);
	result = result || this->isPointInside(triangle.m_v2);
	return result;
}

double Polygon::getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const
{
	double max = 0.0;
	for (auto &pt : m_polygon)
	{
		double distance = line.getDistanceOfPoint(pt);
		if (distance > max)
		{
			max = distance;
			point = pt;
		}
	}
	return max;
}

double Polygon::getMinimumDistanceFromPolygon(const Polygon &other)
{
	double temp_distance = 0.0;
	double distance_1 = std::numeric_limits<double>::max();
	for (int i = 0; i < this->m_polygon.size(); i++)
	{
		int k = 0;
		for (int j = 1; j < other.m_polygon.size(); j++)
		{
			SO::Line edge(other.m_polygon[j], other.m_polygon[k]);
			temp_distance = edge.getDistanceFromPointToLineSegment(this->m_polygon[i]);
			if (temp_distance < distance_1)
			{
				distance_1 = temp_distance;
			}
			k = j;
		}
	}

	double distance_2 = std::numeric_limits<double>::max();
	for (int i = 0; i < other.m_polygon.size(); i++)
	{
		int k = 0;
		for (int j = 1; j < this->m_polygon.size(); j++)
		{
			SO::Line edge(this->m_polygon[j], this->m_polygon[k]);
			temp_distance = edge.getDistanceFromPointToLineSegment(other.m_polygon[i]);
			if (temp_distance < distance_2)
			{
				distance_2 = temp_distance;
			}
			k = j;
		}
	}

	if (distance_1 < distance_2)
	{
		return distance_1;
	}
	else
	{
		return distance_2;
	}
}

Polygon Polygon::getTransformedPolygon(const SO::Plane &plane) const
{
	if (m_plane.isIntersectedWithPlane(plane) == false)
	{
		return *this;
	}

	Polygon transformed_polygon;
	transformed_polygon.setPlane(plane);

	Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
	transformation_matrix = this->computeTransformationMatrix(m_plane.getNormal(), plane.getNormal());
	Eigen::Vector3d origin = m_plane.getOrigin();

	for (auto &point : m_polygon)
	{
		transformed_polygon.addPointToBack(transformation_matrix * (point - origin) + origin);
	}
	
	return transformed_polygon;
}

Polygon Polygon::getConvexHullPolygon() const
{
	Polygon transformed_polygon = *this;
	Plane xy_plane = Plane(m_plane.getOrigin(), Eigen::Vector3d(0, 0, 1));
	transformed_polygon = transformed_polygon.getTransformedPolygon(xy_plane);

	std::vector<CgalPoint2D_EPICK> points;
	for (auto &point : transformed_polygon.get())
	{
		points.emplace_back(CgalPoint2D_EPICK(point[0], point[1]));		
	}

	typedef CGAL::Convex_hull_traits_adapter_2<EPICK,
		CGAL::Pointer_property_map<CgalPoint2D_EPICK>::type > Convex_hull_traits_2;

	std::vector<std::size_t> indices(points.size()), out;
	std::iota(indices.begin(), indices.end(), 0);
	CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
		Convex_hull_traits_2(CGAL::make_property_map(points)));

	Polygon convex_hull;
	convex_hull.setPlane(xy_plane);
	for (std::size_t i : out) {
		Eigen::Vector3d point(points[i].x(), points[i].y(), xy_plane.getOrigin()[2]);
		convex_hull.addPointToBack(point);
	}

	convex_hull = convex_hull.getTransformedPolygon(m_plane);
	return convex_hull;
}

void Polygon::addPointToBack(const Eigen::Vector3d &point)
{
	if (!this->m_plane.isPointOnPlane(point))
	{
		std::cout << "Point add to back is not on plane!" << std::endl;
	}
	m_polygon.emplace_back(point);
}

Eigen::Vector3d Polygon::operator[](unsigned int index) const
{
	return m_polygon[index];
}

Polygon &Polygon::operator=(const Polygon &other)
{
	m_polygon = other.m_polygon;
	m_plane = other.m_plane;
	return *this;
}

Eigen::Vector3d Polygon::getAxisOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const
{
	Eigen::Vector3d axis_of_rotation = vector_1.cross(vector_2);
	axis_of_rotation = axis_of_rotation / axis_of_rotation.norm(); // Problem of axis_of_rotation.norm() = 0 will be handle in computeTransformationMatrix();
	return axis_of_rotation;
}

double Polygon::getAngleOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const
{
	double rotation_angle = vector_1.dot(vector_2) / (vector_1.norm() * vector_2.norm());
	if (rotation_angle > 1.0 || rotation_angle < -1.0)
	{
		return 0;
	}
	else
	{
		rotation_angle = acos(rotation_angle);
		return rotation_angle;
	}
}

Eigen::Transform<double, 3, Eigen::Affine> Polygon::computeTransformationMatrix(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const
{
	Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
	// If loop for getting rid of transformation matrix with Nan
	// It happens if v1 and v2 pointing to the same direction
	if (getAngleOfRotation(vector_1, vector_2) != 0)
	{
		transformation_matrix = Eigen::AngleAxis<double>(
			getAngleOfRotation(vector_1, vector_2),
			getAxisOfRotation(vector_1, vector_2));
	}
	else
	{
		transformation_matrix.setIdentity();
	}
	return transformation_matrix;
}