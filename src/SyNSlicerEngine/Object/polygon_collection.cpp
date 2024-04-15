#include "polygon_collection.h"

using SyNSlicerEngine::Object::PolygonCollection;

PolygonCollection::PolygonCollection()
{
}

PolygonCollection::PolygonCollection(const PolygonCollection &other)
{
	*this = other;
}

PolygonCollection::PolygonCollection(const Clipper2Lib::PathsD &polygons, const SO::Plane &plane)
{
	double z = plane.getOrigin()[2];
	for (int i = 0; i < polygons.size(); i++)
	{
		SO::Polygon temp_polygon;
		temp_polygon.setPlane(plane);
		for (int j = 0; j < polygons[i].size(); j++)
		{
			Eigen::Vector3d point(polygons[i][j].x, polygons[i][j].y, z);
			temp_polygon.addPointToBack(point);
		}
		this->addPolygon(temp_polygon);
	}
}

PolygonCollection::PolygonCollection(const std::vector<CgalPolyline_EPICK> &polygons, const SO::Plane &plane)
{
	for (int i = 0; i < polygons.size(); i++)
	{
		SO::Polygon temp_polygon;
		temp_polygon.setPlane(plane);
		for (int j = 0; j < polygons[i].size(); j++)
		{
			temp_polygon.addPointToBack(Eigen::Vector3d(polygons[i][j].x(), polygons[i][j].y(), polygons[i][j].z()));
		}
		this->addPolygon(temp_polygon);
	}
}

PolygonCollection::~PolygonCollection()
{
}

int PolygonCollection::numberOfPolygons() const
{
	return m_polygons.size();
}

const std::vector<SO::Polygon> &PolygonCollection::get() const
{
	return m_polygons;
}

void PolygonCollection::closePolygons()
{
	for (auto &polyline : m_polygons)
	{
		polyline.closePolygon();
	}
}

Eigen::Vector3d PolygonCollection::centroid() const
{
	if (m_polygons.size() < 1)
	{
		double min = -std::numeric_limits<double>::max();
		return Eigen::Vector3d(min, min, min);
	}

	double numerator_x = 0.0;
	double numerator_y = 0.0;
	double numerator_z = 0.0;
	double total_area = 0.0;
	for (auto &polygon : m_polygons)
	{
		double area = polygon.area();
		numerator_x += polygon.centroid()[0] * area;
		numerator_y += polygon.centroid()[1] * area;
		numerator_z += polygon.centroid()[2] * area;
		total_area += area;
	}

	double x = numerator_x / total_area;
	double y = numerator_y / total_area;
	double z = numerator_z / total_area;

	Eigen::Vector3d result(x, y, z);
	assert(abs(m_plane.getDistanceFromPointToPlane(result)) < 1e-3);
	return result;
}

void PolygonCollection::getBoundingBox(double(&bound)[6])
{
	if (m_polygons.size() == 0)
	{
		return;
	}
	if (m_boudning_box_calculated == false)
	{
		m_polygons[0].getBoundingBox(m_bounding_box);
		for (int i = 1; i < m_polygons.size(); i++)
		{
			double bound[6];
			m_polygons[i].getBoundingBox(bound);
			if (bound[0] < m_bounding_box[0]) { m_bounding_box[0] = bound[0]; };
			if (bound[1] > m_bounding_box[1]) { m_bounding_box[1] = bound[1]; };
			if (bound[2] < m_bounding_box[2]) { m_bounding_box[2] = bound[2]; };
			if (bound[3] > m_bounding_box[3]) { m_bounding_box[3] = bound[3]; };
			if (bound[4] < m_bounding_box[4]) { m_bounding_box[4] = bound[4]; };
			if (bound[5] > m_bounding_box[5]) { m_bounding_box[5] = bound[5]; };
		}
		m_boudning_box_calculated = true;
	};

	memcpy(&bound, &m_bounding_box, sizeof(m_bounding_box));
}

double PolygonCollection::getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const
{
	double min = std::numeric_limits<double>::max();
	Eigen::Vector3d closest_pt(0, 0, 0);
	for (auto &polygon : m_polygons)
	{
		double distance = polygon.getClosestPointFromLine(line, closest_pt);
		if (distance < min)
		{
			min = distance;
			point = closest_pt;
		}
	}
	return min;
}

double PolygonCollection::getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const
{
	double max = 0.0;
	Eigen::Vector3d furthest_pt(0, 0, 0);
	for (auto &polygon : m_polygons)
	{
		double distance = polygon.getFurthestPointFromLine(line, furthest_pt);
		if (distance > max)
		{
			max = distance;
			point = furthest_pt;
		}
	}
	return max;
}

bool PolygonCollection::isIntersectedWithPlane(const SO::Plane &plane) const
{
	bool has_points_on_positive_side = false;
	bool has_points_on_negative_side = false;
	for (auto &polygon : m_polygons)
	{
		for (auto &point : polygon.get())
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
	}
	return false;
}

std::vector<Eigen::Vector3d> PolygonCollection::getIntersectionWithPlane(const SO::Plane &plane) const
{
	std::vector<Eigen::Vector3d> intersecting_points;

	for (auto &polygon : this->m_polygons)
	{
		std::vector<Eigen::Vector3d> contour = polygon.get();
		if ((contour.front() - contour.back()).norm() > 1e-6)
		{
			contour.emplace_back(contour.front());
		};
		int j = 0;
		for (int i = 0; i < contour.size(); i++)
		{
			SO::Line line(contour[i], contour[j]);
			if (plane.isIntersectedWithLine(line))
			{
				intersecting_points.emplace_back(plane.getIntersectionWithLine(line));
			}
			j = i;
		}
	};

	return intersecting_points;
}

double PolygonCollection::getMaximumDistanceFromPlane(const SO::Plane &plane) const
{
	double max = 0.0;
	for (auto &polygon : m_polygons)
	{
		for (auto &point : polygon.get())
		{
			double distance = plane.getDistanceFromPointToPlane(point);
			if (distance > max)
			{
				max = distance;
			}
		}
	}
	return max;
}

double PolygonCollection::getMaximumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const
{
	double max = 0.0;
	for (auto &polygon : m_polygons)
	{
		for (auto &pt : polygon.get())
		{
			double distance = plane.getDistanceFromPointToPlane(pt);
			if (distance > max)
			{
				max = distance;
				point = pt;
			}
		}
	}
	return max;
}

double PolygonCollection::getMinimumDistanceFromPlane(const SO::Plane &plane) const
{
	double min = std::numeric_limits<double>::max();
	for (auto &polygon : m_polygons)
	{
		for (auto &point : polygon.get())
		{
			double distance = plane.getDistanceFromPointToPlane(point);
			if (distance < min)
			{
				min = distance;
			}
		}
	}
	return min;
}

double PolygonCollection::getMinimumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const
{
	double min = std::numeric_limits<double>::max();
	for (auto &polygon : m_polygons)
	{
		for (auto &pt : polygon.get())
		{
			double distance = plane.getDistanceFromPointToPlane(pt);
			if (distance < min)
			{
				min = distance;
				point = pt;
			}
		}
	}
	return min;
}

PolygonCollection PolygonCollection::getTransformedPolygons(const SO::Plane &plane) const
{
	SO::PolygonCollection transformed_polygons;

	for (auto &polygon : m_polygons)
	{
		transformed_polygons.addPolygon(polygon.getTransformedPolygon(plane));
	}

	return transformed_polygons;
}

PolygonCollection PolygonCollection::getTransformedPolygons(const SO::Plane &source_plane, const SO::Plane &target_plane) const
{
	SO::PolygonCollection transformed_polygons;

	for (auto &polygon : m_polygons)
	{
		transformed_polygons.addPolygon(polygon.getTransformedPolygon(source_plane, target_plane));
	}

	return transformed_polygons;
}

PolygonCollection PolygonCollection::getTranslatedPolygons(const Eigen::Vector3d &new_origin) const
{
	SO::PolygonCollection transformed_polygons;

	for (auto &polygon : m_polygons)
	{
		transformed_polygons.addPolygon(polygon.getTranslatedPolygon(new_origin));
	}

	return transformed_polygons;
}

PolygonCollection PolygonCollection::projectToOtherPlane(const SO::Plane &plane) const
{
	SO::PolygonCollection projected_contours;
	Eigen::Vector3d direction = -this->m_plane.getNormal();
	for (int i = 0; i < this->m_polygons.size(); i++)
	{
		const SO::Polygon temp_contour = this->m_polygons[i];
		SO::Polygon projected_contour;
		projected_contour.setPlane(plane);
		for (int j = 0; j < temp_contour.numberOfPoints(); j++)
		{
			SO::Line ray(temp_contour[j], direction, 1);
			Eigen::Vector3d point = plane.getIntersectionWithRay(ray);
			projected_contour.addPointToBack(point);
		}
		projected_contours.addPolygon(projected_contour);
	}
	return projected_contours;
}

PolygonCollection PolygonCollection::getOffset(double distance)
{
	SO::PolygonCollection subject_contours = this->getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));

	Clipper2Lib::PathsD subject = subject_contours.getClipper2Polygons();
	subject = Clipper2Lib::Union(subject, Clipper2Lib::FillRule::NonZero);
	subject = Clipper2Lib::InflatePaths(subject, distance, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
	subject = Clipper2Lib::SimplifyPaths(subject, 0.1);

	SO::PolygonCollection result(subject, SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));
	result = result.getTransformedPolygons(this->m_plane);

	return result;
}

PolygonCollection PolygonCollection::getDifference(const PolygonCollection &other)
{
	// Check if this and other are located on the same plane.
	if (this->m_plane != other.m_plane)
	{
		return *this;
	}

	SO::PolygonCollection subject_contours = this->getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));
	SO::PolygonCollection clip_contours = other.getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));

	Clipper2Lib::PathsD subject = subject_contours.getClipper2Polygons();
	Clipper2Lib::PathsD clip = clip_contours.getClipper2Polygons();
	Clipper2Lib::PathsD solution = Clipper2Lib::Difference(subject, clip, Clipper2Lib::FillRule::NonZero);
	solution = Clipper2Lib::SimplifyPaths(solution, 0.01);

	SO::PolygonCollection difference(solution, SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));
	difference = difference.getTransformedPolygons(m_plane);

	return difference;
}

PolygonCollection PolygonCollection::getIntersection(const PolygonCollection &other)
{
	// Check if this and other are located on the same plane.
	if (this->m_plane != other.m_plane)
	{
		return *this;
	}

	SO::PolygonCollection contours_1 = this->getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));
	SO::PolygonCollection contours_2 = other.getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));

	Clipper2Lib::PathsD op1 = contours_1.getClipper2Polygons();
	Clipper2Lib::PathsD op2 = contours_2.getClipper2Polygons();

	Clipper2Lib::PathsD solution = Clipper2Lib::Intersect(op1, op2, Clipper2Lib::FillRule::NonZero);
	solution = Clipper2Lib::SimplifyPaths(solution, 0.1);

	SO::PolygonCollection intersected_contours(solution, SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));;
	intersected_contours = intersected_contours.getTransformedPolygons(m_plane);
	return intersected_contours;
}

PolygonCollection PolygonCollection::getUnion(const PolygonCollection &other)
{
	// Check if this and other are located on the same plane.
	if (this->m_plane != other.m_plane)
	{
		return *this;
	}

	SO::PolygonCollection contours_1 = this->getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));
	SO::PolygonCollection contours_2 = other.getTransformedPolygons(SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));

	Clipper2Lib::PathsD op1 = contours_1.getClipper2Polygons();
	Clipper2Lib::PathsD op2 = contours_2.getClipper2Polygons();

	Clipper2Lib::PathsD solution = Clipper2Lib::Union(op1, op2, Clipper2Lib::FillRule::NonZero);
	solution = Clipper2Lib::SimplifyPaths(solution, 0.1);

	SO::PolygonCollection union_contours(solution, SO::Plane(m_plane.getOrigin(), Eigen::Vector3d::UnitZ()));;
	union_contours = union_contours.getTransformedPolygons(m_plane);
	return union_contours;
}

SO::Polygon PolygonCollection::getLargestPolygon() const
{
	double max = -std::numeric_limits<double>::max();
	SO::Polygon largest_polygon;
	for (auto &polygon : this->m_polygons)
	{
		double area = polygon.area();
		if (area > max)
		{
			largest_polygon = polygon;
			max = area;
		}
	}
	return largest_polygon;
}

SO::Polygon PolygonCollection::getConvexHullPolygon() const
{
	PolygonCollection transformed_polygons = *this;
	Plane xy_plane = Plane(m_plane.getOrigin(), Eigen::Vector3d(0, 0, 1));
	transformed_polygons = transformed_polygons.getTransformedPolygons(xy_plane);

	std::vector<CgalPoint2D_EPICK> points;
	for (auto &polygon : transformed_polygons.get())
	{
		for (auto &point : polygon.get())
		{
			points.emplace_back(CgalPoint2D_EPICK(point[0], point[1]));
		}
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

SO::Polygon PolygonCollection::getPolygonCloestToPolygon(const SO::Polygon &polygon) const
{
	Eigen::Vector3d target = polygon.centroid();
	SO::Polygon result;
	double min = std::numeric_limits<double>::max();
	for (auto &poly : m_polygons)
	{
		Eigen::Vector3d source = poly.centroid();
		double distance = (target - source).norm();
		if (distance < min)
		{
			min = distance;
			result = poly;
		}
	}
	return result;
}

void PolygonCollection::addPolygon(const SO::Polygon &polygon)
{
	if (polygon.numberOfPoints() < 1)
	{
		return;
	}

	if (m_polygons.size() == 0)
	{
		m_polygons.emplace_back(polygon);
		m_plane = polygon.getPlane();
	}
	else
	{
		if (m_plane == polygon.getPlane())
		{
			m_polygons.emplace_back(polygon);
		}
		else
		{
			std::cout << "Polygon cannot be added to PolygonCollection because they are not lied on the same planes!" << std::endl;
		}
	}
}

void PolygonCollection::addPolygons(const PolygonCollection &other)
{
	if (other.numberOfPolygons() < 1)
	{
		return;
	}

	if (m_polygons.size() == 0)
	{
		*this = other;
	}
	else
	{
		if (m_plane == other.m_plane)
		{
			for (auto &polygon : other.m_polygons)
			{
				m_polygons.emplace_back(polygon);
			}
		}
		else
		{
			std::cout << "Polygons cannot be added to PolygonCollection because they are not lied on the same planes!" << std::endl;
		}
	}
}

int PolygonCollection::removePolygonsBelowPlane(const SO::Plane &plane)
{
	std::vector<Polygon> new_polygons;
	int number_of_erased_polygons = 0;
	for (auto &polygon : m_polygons)
	{
		if (plane.getPositionOfPointWrtPlane(polygon.centroid()) == -1)
		{
			number_of_erased_polygons += 1;
		}
		else
		{
			new_polygons.emplace_back(polygon);
		}
	}
	m_polygons = new_polygons;
	return number_of_erased_polygons;
}

void PolygonCollection::setPlane(const Plane &plane)
{
	m_plane = plane;
	for (auto &polygon : m_polygons)
	{
		polygon.setPlane(plane);
	}
}

const SyNSlicerEngine::Object::Plane &PolygonCollection::getPlane() const
{
	return m_plane;
}

void PolygonCollection::reset()
{
	*this = PolygonCollection();
}

PolygonCollection &PolygonCollection::operator=(const PolygonCollection &other)
{
	m_polygons = other.m_polygons;
	m_plane = other.m_plane;
	return *this;
}

SO::Polygon &PolygonCollection::operator[](unsigned int index)
{
	return m_polygons[index];
}

const SO::Polygon &PolygonCollection::operator[](unsigned int index) const
{
	return m_polygons[index];
}

Clipper2Lib::PathsD PolygonCollection::getClipper2Polygons()
{
	Clipper2Lib::PathsD clipper2_polygon;
	for (auto &polygon : m_polygons)
	{
		Clipper2Lib::PathD temp_path;
		for (auto &point : polygon.get())
		{
			Clipper2Lib::PointD cp2point(point[0], point[1]);
			temp_path.push_back(cp2point);
		}
		clipper2_polygon.push_back(temp_path);
	}
	return clipper2_polygon;
}