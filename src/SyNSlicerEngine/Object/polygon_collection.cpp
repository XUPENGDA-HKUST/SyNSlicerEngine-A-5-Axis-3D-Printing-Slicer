#include "polygon_collection.h"

using SyNSlicerEngine::Object::PolygonCollection;

PolygonCollection::PolygonCollection()
{
}

PolygonCollection::PolygonCollection(const PolygonCollection &other)
{
	*this = other;
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

Eigen::Vector3d PolygonCollection::centroid() const
{
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

PolygonCollection PolygonCollection::getTransformedPolygon(const SO::Plane &plane) const
{
	SO::PolygonCollection transformed_polygons;

	for (auto &polygon : m_polygons)
	{
		transformed_polygons.addPolygon(polygon.getTransformedPolygon(plane));
	}

	return transformed_polygons;
}

SO::Polygon PolygonCollection::getConvexHullPolygon() const
{
	PolygonCollection transformed_polygons = *this;
	Plane xy_plane = Plane(m_plane.getOrigin(), Eigen::Vector3d(0, 0, 1));
	transformed_polygons = transformed_polygons.getTransformedPolygon(xy_plane);

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

void PolygonCollection::addPolygon(const SO::Polygon &polygon)
{
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

void PolygonCollection::setPlane(const Plane &plane)
{
	m_plane = plane;
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

const SO::Polygon &PolygonCollection::operator[](unsigned int index) const
{
	return m_polygons[index];
}
