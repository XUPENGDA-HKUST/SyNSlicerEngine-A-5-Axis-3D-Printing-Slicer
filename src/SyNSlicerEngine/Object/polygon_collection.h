#ifndef SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_
#define SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_

#include <vector>

#include "clipper2/clipper.h"
#include "Object/polygon.h"

using namespace Clipper2Lib;

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	class PolygonCollection
	{		
	public:
		PolygonCollection();
		PolygonCollection(const PolygonCollection &other);
		~PolygonCollection();

		int numberOfPolygons() const;
		const std::vector<Polygon> &get() const;

		Eigen::Vector3d centroid() const;
		bool isIntersectedWithPlane(const SO::Plane &plane) const;
		std::vector<Eigen::Vector3d> getIntersectionWithPlane(const SO::Plane &plane) const;

		PolygonCollection getTransformedPolygon(const SO::Plane &plane) const;
		Polygon getConvexHullPolygon() const;

		Polygon getLargestPolygon();

		void addPolygon(const Polygon &polygon);
		void addPolygons(const PolygonCollection &other);
		
		void setPlane(const Plane &plane);
		const Plane &getPlane() const;

		void reset();

		PolygonCollection &operator=(const PolygonCollection &other);
		const Polygon &operator[](unsigned int index) const;

	private:
		std::vector<Polygon> m_polygons;
		Plane m_plane;
	};
}


#endif  // SYNSLICERENGINE_OBJECT_POLYGONCOLLECTION_H_