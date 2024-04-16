#ifndef SYNSLICERENGINE_OBJECT_PLANE_H_
#define SYNSLICERENGINE_OBJECT_PLANE_H_

#include <Eigen/Core>
#include "Object/line.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//!  This class defines a plane.
	/*!
		A plane is defined by a origin and a unit vector point from this origin. \n
		General equation of the plane: ax + by + cz + d = 0
	*/
	class Plane
	{
	public:

		Plane();
		Plane(const Plane &other);
		Plane(const Eigen::Vector3d &input_origin, const Eigen::Vector3d &input_normal);
		Plane(double a, double b, double c, double d);
		~Plane();

		bool isValid();

		void printInfo();

		void setOrigin(const Eigen::Vector3d &input_origin);
		const Eigen::Vector3d &getOrigin() const;

		void setNormal(const Eigen::Vector3d &input_normal);
		const Eigen::Vector3d &getNormal() const;

		void setPlaneInGeneralForm(double a, double b, double c, double d);

		void offset(double input_distance);

		const double a() const;
		const double b() const;
		const double c() const;
		const double d() const;

		// Algorithm related to point
		bool isPointOnPlane(const Eigen::Vector3d &point, double epsilon = 1e-6) const;
		bool isPointOnPlane(const CgalPoint_EPICK &point, double epsilon = 1e-6) const;
		double getDistanceFromPointToPlane(const Eigen::Vector3d &point) const;
		double getDistanceFromPointToPlane(const CgalPoint_EPICK &point) const;
		int getPositionOfPointWrtPlane(const Eigen::Vector3d &point, double epsilon = 1e-6) const;
		Eigen::Vector3d getProjectionOfPointOntoPlane(const Eigen::Vector3d &point) const;

		// Algorithm related to points
		bool getMostPositivePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const;
		bool getMostNegativePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const;

		bool isLineOnPlane(const SO::Line &line, double epsilon = 1e-6) const;
		bool isIntersectedWithLine(const SO::Line &line) const;
		bool isIntersectedWithLine(const SO::Line &line, Eigen::Vector3d &point) const;
		bool isIntersectedWithRay(const SO::Line &ray) const;
		bool isIntersectedWithRay(const SO::Line &ray, Eigen::Vector3d &point) const;
		bool isIntersectedWithPlane(const Plane &other) const;
		bool isIntersectedWithPlane(const Plane &other, SO::Line &intersecting_line) const;

		double getAngleOfRotation(const Plane &destination) const;
		Eigen::Vector3d getAxisOfRotation(const Plane &destination) const;
		Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix(const Plane &destination) const;

		Plane &operator = (const Plane &other);
		bool operator == (const Plane &other);
		bool operator == (const Plane &other) const;

		bool operator != (const Plane &other);
		bool operator != (const Plane &other) const;

	protected:
		Eigen::Vector3d m_origin;
		Eigen::Vector3d m_normal;
		double m_distance;
		bool m_is_valid;

	private:
		Eigen::Vector3d findPointLocatedOnPlane();

	};
	
}

#endif  // SYNSLICERENGINE_OBJECT_PLANE_H_