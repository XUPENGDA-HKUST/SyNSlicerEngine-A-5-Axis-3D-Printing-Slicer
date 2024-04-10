#ifndef SYNSLICERENGINE_OBJECT_POLYGON_H_
#define SYNSLICERENGINE_OBJECT_POLYGON_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "Object/plane.h"
#include "Object/triangle.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! This class defines a polygon.
	//! Polygon is a special class of polyline.
	//! All the points should lie on the same plane.
	//! 
	class Polygon 
	{
	public:

		Polygon();
		Polygon(const Polygon &other);
		~Polygon();

		int numberOfPoints() const;
		const std::vector<Eigen::Vector3d> &get() const;
		void setPlane(const SO::Plane &plane);
		const SO::Plane &getPlane() const;
		void reset();
		
		bool isClosed() const;
		void closePolygon();

		int isClockWise() const;

		Eigen::Vector3d centroid() const;
		void getBoundingBox(double(&bound)[6]);
		double area() const;
		double length() const;
		bool isIntersectedWithPlane(const SO::Plane &plane);
		bool isIntersectedWithPlane(const SO::Plane &plane, std::vector<Eigen::Vector3d> &intersecting_points) const;

		bool isPointInside(const Eigen::Vector3d &point);
		bool isLineInside(const SO::Line &line);
		bool isOneOfTheVerticesOfTriangleInside(const SO::Triangle &triangle);

		double getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;
		double getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		double getMinimumDistanceFromPolygon(const Polygon &other);

		Polygon getTransformedPolygon(const SO::Plane &plane) const;
		Polygon getTransformedPolygon(const SO::Plane &source_plane, const SO::Plane &target_plane) const;
		Polygon getTranslatedPolygon(const Eigen::Vector3d &new_origin) const;
		Polygon getConvexHullPolygon() const;


		void addPointToBack(const Eigen::Vector3d &point);

		Eigen::Vector3d operator[](unsigned int index) const;
		Polygon &operator=(const Polygon &other);

	private:
		Eigen::Vector3d getAxisOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;
		double getAngleOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;
		Eigen::Transform<double, 3, Eigen::Affine> computeTransformationMatrix(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;

		std::vector<Eigen::Vector3d> m_polygon;
		SO::Plane m_plane;

		CgalPolygon2D_EPICK m_cgal_polygon;

		bool m_boudning_box_calculated = false;
		double m_bounding_box[6] = { 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 };
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYGON_H_