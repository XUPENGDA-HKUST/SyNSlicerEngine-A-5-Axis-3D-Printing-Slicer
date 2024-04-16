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

bool Polygon::isClosed() const
{
	if ((m_polygon.back() - m_polygon.front()).norm() < 1e-6)
	{
		return true;
	}
	return false;
}

void Polygon::closePolygon()
{
	if (this->isClosed() == false)
	{
		m_polygon.emplace_back(m_polygon.front());
	}
}

int Polygon::isClockWise() const
{
	if (m_polygon.size() < 3)
	{
		return 0;
	}

	CgalPolygon2D_EPICK cgal_polygon;
	Plane xy_plane = Plane(m_plane.getOrigin(), Eigen::Vector3d(0, 0, 1));
	Polygon transformed_polygon = this->getTransformedPolygon(xy_plane);
	for (auto &point : transformed_polygon.get())
	{
		cgal_polygon.push_back(CgalPoint2D_EPICK(point[0], point[1]));
	}	

	if (!cgal_polygon.is_simple())
	{
		return 0;
	}

	if (cgal_polygon.is_clockwise_oriented())
	{
		return 1;
	}
	else if (cgal_polygon.is_counterclockwise_oriented())
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

Eigen::Vector3d Polygon::centroid() const
{
	return this->getCentroid();
}

Eigen::Vector3d Polygon::getCentroid() const
{
	assert(m_polygon.size() > 0);

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
	assert(abs(m_plane.getDistanceFromPointToPlane(result)) < 1e-2);
	return result;
}

double Polygon::area() const
{
	return this->getArea();
}

double Polygon::getArea() const
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
	return this->getLength();
}

double Polygon::getLength() const
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

void Polygon::getBoundingBox(double(&bound)[6])
{
	if (m_polygon.size() == 0)
	{
		return;
	}

	if (m_boudning_box_calculated == false)
	{
		int a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
		for (int i = 1; i < m_polygon.size(); i++)
		{
			if (m_polygon[i][0] < m_polygon[a][0]) { a = i; };
			if (m_polygon[i][0] > m_polygon[b][0]) { b = i; };
			if (m_polygon[i][1] < m_polygon[c][1]) { c = i; };
			if (m_polygon[i][1] > m_polygon[d][1]) { d = i; };
			if (m_polygon[i][2] < m_polygon[e][2]) { e = i; };
			if (m_polygon[i][2] > m_polygon[f][2]) { f = i; };
		}

		m_bounding_box[0] = m_polygon[a][0];
		m_bounding_box[1] = m_polygon[b][0];
		m_bounding_box[2] = m_polygon[c][1];
		m_bounding_box[3] = m_polygon[d][1];
		m_bounding_box[4] = m_polygon[e][2];
		m_bounding_box[5] = m_polygon[f][2];

		m_boudning_box_calculated = true;
	};

	memcpy(&bound, &m_bounding_box, sizeof(m_bounding_box));
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

bool Polygon::isIntersectedWithPlane(const SO::Plane &plane, std::vector<Eigen::Vector3d> &intersecting_points) const
{
	if (m_polygon.size() < 3)
	{
		return false;
	}

	Eigen::Vector3d point;
	int j = 0;
	for (int i = 1; i < m_polygon.size(); i++)
	{
		SO::Line line(m_polygon[i], m_polygon[j]);
		if (plane.isIntersectedWithLine(line, point))
		{
			intersecting_points.push_back(point);
		}
		j = i;
	}

	if (!this->isClosed())
	{
		Object::Line line(m_polygon[j], m_polygon[0]);
		if (plane.isIntersectedWithLine(line, point))
		{
			intersecting_points.push_back(point);
		}
	}

	if (intersecting_points.size() > 0)
	{
		return true;
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

bool Polygon::isLineInside(const SO::Line &line)
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

	Eigen::Vector3d temp_point = line.getMiddle();
	temp_point = m_plane.getProjectionOfPointOntoPlane(temp_point);
	temp_point = transformation_matrix * (temp_point - m_plane.getOrigin()) + m_plane.getOrigin();

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

double Polygon::getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const
{
	double min = std::numeric_limits<double>::max();
	for (auto &pt : m_polygon)
	{
		double distance = line.getDistanceOfPoint(pt);
		if (distance < min)
		{
			min = distance;
			point = pt;
		}
	}
	return min;
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
	Eigen::Vector3d origin = plane.getOrigin();

	for (auto &point : m_polygon)
	{
		transformed_polygon.addPointToBack(transformation_matrix * (point - origin) + origin);
	}
	
	return transformed_polygon;
}

Polygon Polygon::getTransformedPolygon(const SO::Plane &source_plane, const SO::Plane &target_plane) const
{
	if (source_plane.isIntersectedWithPlane(target_plane) == false)
	{
		return *this;
	}

	Polygon transformed_polygon;
	transformed_polygon.setPlane(m_plane);

	Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
	transformation_matrix = this->computeTransformationMatrix(source_plane.getNormal(), target_plane.getNormal());
	Eigen::Vector3d origin = m_plane.getOrigin();

	for (auto &point : m_polygon)
	{
		transformed_polygon.addPointToBack(transformation_matrix * (point - origin) + origin);
	}

	return transformed_polygon;
}

Polygon Polygon::getTranslatedPolygon(const Eigen::Vector3d &new_origin) const
{
	Polygon transformed_polygon;
	transformed_polygon.setPlane(SO::Plane(new_origin, m_plane.getNormal()));
	Eigen::Vector3d old_origin = m_plane.getOrigin();
	for (auto &point : m_polygon)
	{
		transformed_polygon.addPointToBack(point - old_origin + new_origin);
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
	if (!this->m_plane.isPointOnPlane(point, 1e-2))
	{
		std::cout << "Point add to back is not on plane! " << this->m_plane.getDistanceFromPointToPlane(point) << std::endl;
	}
	m_polygon.emplace_back(point);
}

void Polygon::push_back(const Eigen::Vector3d &point)
{
	this->addPointToBack(point);
}

void Polygon::emplace_back(const Eigen::Vector3d &point)
{
	this->addPointToBack(point);
}

void Polygon::pop_back()
{
	m_polygon.pop_back();
}

int Polygon::size()
{
	return m_polygon.size();
}

void Polygon::clear()
{
	this->reset();
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