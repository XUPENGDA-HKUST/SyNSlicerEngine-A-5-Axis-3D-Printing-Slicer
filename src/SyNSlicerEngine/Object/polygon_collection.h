#ifndef SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_

#include <vector>

#include "clipper2/clipper.h"
#include "Object/polygon.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	class PolygonCollection
	{		
	public:
		PolygonCollection();
		PolygonCollection(const PolygonCollection &other);
		PolygonCollection(const Clipper2Lib::PathsD &polygons, const SO::Plane &plane);
		PolygonCollection(const std::vector<CgalPolyline_EPICK> &polygons, const SO::Plane &plane);
		~PolygonCollection();

		int numberOfPolygons() const;
		const std::vector<Polygon> &get() const;

		void closePolygons();

		Eigen::Vector3d centroid() const;
		void getBoundingBox(double(&bound)[6]);

		double getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;
		double getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		bool isIntersectedWithPlane(const SO::Plane &plane) const;

		std::vector<Eigen::Vector3d> getIntersectionWithPlane(const SO::Plane &plane) const;

		double getMaximumDistanceFromPlane(const SO::Plane &plane) const;
		double getMaximumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const;
		double getMinimumDistanceFromPlane(const SO::Plane &plane) const;
		double getMinimumDistanceFromPlane(const SO::Plane &plane, Eigen::Vector3d &point) const;

		PolygonCollection getTransformedPolygons(const SO::Plane &plane) const;
		PolygonCollection getTransformedPolygons(const SO::Plane &source_plane, const SO::Plane &target_plane) const;
		PolygonCollection getTranslatedPolygons(const Eigen::Vector3d &new_origin) const;
		PolygonCollection projectToOtherPlane(const SO::Plane &plane) const;

		PolygonCollection getOffset(double distance);
		PolygonCollection getDifference(const PolygonCollection &other);
		PolygonCollection getIntersection(const PolygonCollection &other);
		PolygonCollection getUnion(const PolygonCollection &other);

		Polygon getLargestPolygon() const;
		Polygon getConvexHullPolygon() const;
		Polygon getPolygonCloestToPolygon(const Polygon &polygon) const;

		void addPolygon(const Polygon &polygon);
		void addPolygons(const PolygonCollection &other);
		
		int removePolygonsBelowPlane(const SO::Plane &plane);

		void setPlane(const Plane &plane);
		const Plane &getPlane() const;

		void reset();

		PolygonCollection &operator=(const PolygonCollection &other);
		Polygon &operator[](unsigned int index);
		const Polygon &operator[](unsigned int index) const;

	private:

		Clipper2Lib::PathsD getClipper2Polygons();

		std::vector<Polygon> m_polygons;
		Plane m_plane;

		bool m_boudning_box_calculated = false;
		double m_bounding_box[6] = { 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 };
	};
}


#endif  // SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_