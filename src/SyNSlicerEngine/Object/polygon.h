#ifndef SYNSLICERENGINE_OBJECT_POLYGON_H_
#define SYNSLICERENGINE_OBJECT_POLYGON_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "Object/plane.h"
#include "Object/triangle.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//! This class defines a polygon.
	/*!
		Polygon is a special class of polyline. \n
		All the points should lie on the same plane.
	*/
	class Polygon 
	{
	public:
		//! Default constructor.
		Polygon();

		//! Copy constructor.
		Polygon(const Polygon &other);

		//! Destructor.
		~Polygon();

		//! Get number of points in the polygon.
		/*!
			\return int Number of points.
		*/
		int numberOfPoints() const;

		//! Get all the points in the polygon.
		/*!
			\return std::vector<Eigen::Vector3d> Points.
		*/
		const std::vector<Eigen::Vector3d> &get() const;

		//! Set the plane this polygon lies on.
		/*!
			\param[in] plane Plane.
		*/
		void setPlane(const SO::Plane &plane);

		//! Get the plane this polygon lies on.
		/*!
			\return SO::Plane Plane.
		*/
		const SO::Plane &getPlane() const;

		//! Restore to default value.
		void reset();
		
		//! Check if the first point equals to the last point.
		/*!
			\return \b True if equals. \n \b False if not equals.
		*/
		bool isClosed() const;

		//! Close the polygon if the first point is not equal to the last point
		void closePolygon();

		//! Check if the polygon is clockwise.
		/*!
			\return \b 1 if clockwise. \n \b 0 if polygon is not simple. \n \b -1 if counter-clockwise.
		*/
		int isClockWise() const;

		//! Get the centroid of the polygon.
		/*!
			\return \b Eigen::Vector3d Centroid.
		*/
		Eigen::Vector3d centroid() const;

		//! Get the centroid of the polygon.
		/*!
			\return \b Eigen::Vector3d Centroid.
		*/
		Eigen::Vector3d getCentroid() const;

		//! Get the area of the polygon.
		/*!
			\return \b double Area.
		*/
		double area() const;

		//! Get the area of the polygon.
		/*!
			\return \b double Area.
		*/
		double getArea() const;

		//! Get the perimeter of the polygon.
		/*!
			\return \b double Perimeter.
		*/
		double perimeter() const;

		//! Get the perimeter of the polygon.
		/*!
			\return \b double Perimeter.
		*/
		double getPerimeter() const;

		//! Get bounding box of the polygon.
		/*!
			\param[out] bound Bounding box.
		*/
		void getBoundingBox(double bound[6]);

		//! Check if the polygon intersects with a plane.
		/*!
			\param[in] plane Plane.
			\return \b True if intersected. \n \b False if not intersected.
		*/
		bool isIntersectedWithPlane(const SO::Plane &plane);

		//! Check if the polygon intersects with a plane.
		/*!
			\param[in] plane Plane.
			\param[out] intersecting_points Intersecting points.
			\return \b True if intersected. \n \b False if not intersected.
		*/
		bool isIntersectedWithPlane(const SO::Plane &plane, std::vector<Eigen::Vector3d> &intersecting_points) const;

		//! Check if a point located inside the polygon.
		/*!
			\brief	If the point is not lie on the same plane with the polygon, 
					get the projectiong first and then determing whether the 
					projection lies inside the polygon.
			\param[in] point Point.
			\return \b True if inside. \n \b False if not inside.
		*/
		bool isPointInside(const Eigen::Vector3d &point);

		//! Check if the midpoint of a line located inside the polygon.
		/*!
			\param[in] line Line.
			\return \b True if inside. \n \b False if not inside.
		*/
		bool isMidpointOfLineInside(const SO::Line &line);

		//! Check if any vertices of a triangle lie inside the polygon.
		/*!
			\param[in] triangle Triangle.
			\return \b True if yes. \n \b False if no.
		*/
		bool isOneOfTheVerticesOfTriangleInside(const SO::Triangle &triangle);

		//! Find closest point on the polygon to a line.
		/*!
			\param[in] line Line.
			\param[out] point Closest point.
			\return \b double Distance between point to line.
		*/
		double getClosestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		//! Find furthest point on the polygon to a line.
		/*!
			\param[in] line Line.
			\param[out] point Furthest point.
			\return \b double Distance between point to line.
		*/
		double getFurthestPointFromLine(const SO::Line &line, Eigen::Vector3d &point) const;

		//! Find minimum distance from this polygon to other polygon.
		/*!
			\param[in] other Other polygon.
			\return \b double Minimum distance between polygons.
		*/
		double getMinimumDistanceFromPolygon(const Polygon &other);

		//! Transform polygon to other plane.
		/*!
			\param[in] plane Other plane.
			\return \b Polygon Transformed polygon.
		*/
		Polygon getTransformedPolygon(const SO::Plane &plane) const;

		//! Transform polygon to other plane.
		/*!
			\param[in] source_plane		Source plane.
			\param[in] target_plane		Target plane.
			\return \b Polygon Transformed polygon.
		*/
		Polygon getTransformedPolygon(const SO::Plane &source_plane, const SO::Plane &target_plane) const;

		//! Translate polygon to other origin.
		/*!
			\param[in] new_origin Other origin.
			\return \b Polygon Translated polygon.
		*/
		Polygon getTranslatedPolygon(const Eigen::Vector3d &new_origin) const;

		//! Get convex hull of the polygon.
		/*!
			\return \b Polygon The convex hull.
		*/
		Polygon getConvexHullPolygon() const;

		//! Add point to polygon.
		/*!
			\param point Point.
		*/
		void addPointToBack(const Eigen::Vector3d &point);

		//! Add point to polygon.
		/*!
			\param point Point.
		*/
		void push_back(const Eigen::Vector3d &point);

		//! Add point to polygon.
		/*!
			\param point Point.
		*/
		void emplace_back(const Eigen::Vector3d &point);

		//! Remove the last point in polygon.
		void pop_back();

		//! Get number of points in polygon.
		/*!
			\return \n int Number of points.
		*/
		int size();

		//! Restore to default value.
		void clear();

		//! Get the n-th point in polygon.
		/*!
			\param index Index.
		*/
		Eigen::Vector3d operator[](unsigned int index) const;

		//! Copy assignment operator.
		Polygon &operator=(const Polygon &other);

	protected:
		//! Get cross vector of two vectors.
		Eigen::Vector3d getAxisOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;

		//! Get angle between two vectors.
		double getAngleOfRotation(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;

		//! Get transformation matrix from two vectors.
		Eigen::Transform<double, 3, Eigen::Affine> computeTransformationMatrix(const Eigen::Vector3d &vector_1, const Eigen::Vector3d &vector_2) const;

		//! Store all the points in the polygon.
		std::vector<Eigen::Vector3d> m_polygon;

		//! Store the plane this polygon lies on.
		SO::Plane m_plane;

		//! Cgal polygon used to check is point inside this polygon.
		CgalPolygon2D_EPICK m_cgal_polygon;

		//! Store whether bounding box is calculated.
		bool m_boudning_box_calculated = false;

		//! Store the bounding box.
		double m_bounding_box[6] = { 0.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 };
	};
}

#endif  // SYNSLICERENGINE_OBJECT_POLYGON_H_