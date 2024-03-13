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


		bool isPointOnPlane(const Eigen::Vector3d &point, double epsilon = 1e-6) const;
		bool isPointOnPlane(const CgalPoint_EPICK &point, double epsilon = 1e-6) const;

		double getDistanceFromPointToPlane(const Eigen::Vector3d &point) const;
		double getDistanceFromPointToPlane(const CgalPoint_EPICK &point) const;

		Eigen::Vector3d getProjectionOfPointOntoPlane(const Eigen::Vector3d &point) const;

		bool isLineOnPlane(const SO::Line &line, double epsilon = 1e-6) const;

		bool isIntersectedWithPlane(const Plane &other) const;

		

		Plane &operator = (const Plane &other);
		bool operator == (const Plane &other);
		bool operator == (const Plane &other) const;

		bool operator != (const Plane &other);
		bool operator != (const Plane &other) const;

	protected:
		Eigen::Vector3d m_origin;
		Eigen::Vector3d m_normal;
		double m_distance;

	private:
		Eigen::Vector3d findPointLocatedOnPlane();

	};
	
}

#endif  // SYNSLICERENGINE_OBJECT_PLANE_H_