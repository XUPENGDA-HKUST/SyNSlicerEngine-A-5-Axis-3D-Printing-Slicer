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
		//! Default constructor.
		Plane();

		//! Copy constructor.
		Plane(const Plane &other);

		//! Constructor.
		/*!
			\param[in]	input_origin	Origin of the plane.
			\param[in]	input_normal	Normal of the plane.
		*/
		Plane(const Eigen::Vector3d &input_origin, const Eigen::Vector3d &input_normal);

		//! Constructor.
		/*!
			Construct by giving the general form. General equation of the plane: ax + by + cz + d = 0.
			\param[in]	a
			\param[in]	b
			\param[in]	c
			\param[in]	d
		*/
		Plane(double a, double b, double c, double d);

		//! Constructor.
		/*!
			\param[in]	file_name	Path of the save file.
		*/
		Plane(std::string file_name);

		//! Destructor.
		~Plane();

		//! Save the plane to a .txt file.
		/*!
			\param[in]	file_name	Path of the save file.
		*/
		bool save(std::string file_name);

		//! Load a plane from a .txt file.
		/*!
			\param[in]	file_name	Path of the save file.
		*/
		bool load(std::string file_name);

		//! Get validity of this plane.
		/*!
			\return	\b True if valid.\n \b False if not valid.
		*/
		bool isValid();

		//! Print origin and normal in console.
		void printInfo();

		//! Set origin.
		/*!
			\param[in]	input_origin	Origin.
		*/
		void setOrigin(const Eigen::Vector3d &input_origin);

		//! Get origin.
		/*!
			\return	Eigen::Vector3d.
		*/
		const Eigen::Vector3d &getOrigin() const;

		//! Set normal.
		/*!
			\param[in]	input_normal	Normal.
		*/
		void setNormal(const Eigen::Vector3d &input_normal);

		//! Get normal.
		/*!
			\return	Eigen::Vector3d.
		*/
		const Eigen::Vector3d &getNormal() const;

		//! Set plane by general form.
		/*!
			Construct by giving the general form. General equation of the plane: ax + by + cz + d = 0.
			\param[in]	a
			\param[in]	b
			\param[in]	c
			\param[in]	d
		*/
		void setPlaneInGeneralForm(double a, double b, double c, double d);

		//! Offset the plane.
		/*!
			\param[in]	input_distance	Offset distance. Could be negative.
		*/
		void offset(double input_distance);

		//! Get a in general equation.
		const double a() const;

		//! Get b in general equation.
		const double b() const;

		//! Get c in general equation.
		const double c() const;

		//! Get d in general equation.
		const double d() const;

		//! Check if a point is locate on plane.
		/*!
			\param[in]	point	The point.
			\param[in]	epsilon	The epsilon in comparison.
		*/
		bool isPointOnPlane(const Eigen::Vector3d &point, double epsilon = 1e-6) const;

		//! Check if a point is locate on plane.
		/*!
			\param[in]	point	The point.
			\param[in]	epsilon	The epsilon in comparison.
		*/
		bool isPointOnPlane(const CgalPoint_EPICK &point, double epsilon = 1e-6) const;

		//! Calculate the distance from point to plane.
		/*!
			\param[in]	point	The point.
		*/
		double getDistanceFromPointToPlane(const Eigen::Vector3d &point) const;

		//! Calculate the distance from point to plane.
		/*!
			\param[in]	point	The point.
		*/
		double getDistanceFromPointToPlane(const CgalPoint_EPICK &point) const;

		//! Get position of point w.r.t plane.
		/*!
			\param[in]	point	The point.
			\param[in]	epsilon	The epsilon in comparison.
			\return \b 1 if on positive side.\n \b 0 if on the plane. \b -1 if on the negative side.
		*/
		int getPositionOfPointWrtPlane(const Eigen::Vector3d &point, double epsilon = 1e-6) const;

		//! Get position of point w.r.t plane.
		/*!
			\param[in]	point	The point.
			\return Eigen::Vector3d.
		*/
		Eigen::Vector3d getProjectionOfPointOntoPlane(const Eigen::Vector3d &point) const;

		//! Get furthest point in a point cloud from plane on the postive side.
		/*!
			\param[in]	points	Point cloud.
			\param[out] result	The furthest point.
			\return \b True if the point is found.\n \b False if the point is not found.
		*/
		bool getMostPositivePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const;

		//! Get furthest point in a point cloud from plane on the negative side.
		/*!
			\param[in]	points	Point cloud.
			\param[out] result	The furthest point.
			\return \b True if the point is found.\n \b False if the point is not found.
		*/
		bool getMostNegativePoint(const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &result) const;

		//! Check if a line lies on the plane.
		/*!
			\param[in]	line	The line.
			\param[in]	epsilon	The epsilon in comparison.
			\return \b True if the line lies on the plane.\n \b False if the line dose not lie on the plane.
		*/
		bool isLineOnPlane(const SO::Line &line, double epsilon = 1e-6) const;

		//! Check if a line intersect the plane.
		/*!
			\param[in]	line	The line.
			\return \b True if the line intersects the plane.\n \b False if the line dose not intersect on the plane.
		*/
		bool isIntersectedWithLine(const SO::Line &line) const;

		//! Check if a line intersect the plane.
		/*!
			\param[in]	line	The line.
			\param[out]	point	The intersecting point.
			\return \b True if the line intersects the plane.\n \b False if the line dose not intersect on the plane.
		*/
		bool isIntersectedWithLine(const SO::Line &line, Eigen::Vector3d &point) const;

		//! Check if a ray intersect the plane.
		/*!
			\param[in]	ray		The ray.
			\return \b True if the ray intersects the plane.\n \b False if the ray dose not intersect on the plane.
		*/
		bool isIntersectedWithRay(const SO::Line &ray) const;

		//! Check if a ray intersect the plane.
		/*!
			\param[in]	ray		The ray.
			\param[out]	point	The intersecting point.
			\return \b True if the ray intersects the plane.\n \b False if the ray dose not intersect on the plane.
		*/
		bool isIntersectedWithRay(const SO::Line &ray, Eigen::Vector3d &point) const;

		//! Check if another plane intersect this plane.
		/*!
			\param[in]	other	Another plane.
			\return \b True if another plane intersects the plane.\n \b False if another plane dose not intersect on the plane.
		*/
		bool isIntersectedWithPlane(const Plane &other) const;

		//! Check if another plane intersect this plane.
		/*!
			\param[in]	other				Another plane.
			\param[out]	intersecting_line	The intersecting line.
			\return \b True if another plane intersects the plane.\n \b False if another plane dose not intersect on the plane.
		*/
		bool isIntersectedWithPlane(const Plane &other, SO::Line &intersecting_line) const;

		//! Get angle between two planes.
		/*!
			\param[in]	destination			Another plane.
			\return <b> double </b> Angel between two planes.
		*/
		double getAngleOfRotation(const Plane &destination) const;

		//! Get the cross vector of the normals from two planes.
		/*!
			\param[in]	destination			Another plane.
			\return <b> Eigen::Vector3d </b> Cross vector.
		*/
		Eigen::Vector3d getAxisOfRotation(const Plane &destination) const;

		//! Get the transformation matrix to transfrom a object from this plane to another plane.
		/*!
			\param[in]	destination			Another plane.
			\return <b> Eigen::Transform<double, 3, Eigen::Affine> </b> Transformation matrix.
		*/
		Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix(const Plane &destination) const;

		//! Copy assignment operators
		Plane &operator = (const Plane &other);

		//! Check if this plane equal to another plane.
		/*!
			\param[in]	other	Another plane.
		*/
		bool operator == (const Plane &other);

		//! Check if this plane equal to another plane.
		/*!
			\param[in]	other	Another plane.
		*/
		bool operator == (const Plane &other) const;

		//! Check if this plane is not equal to another plane.
		/*!
			\param[in]	other	Another plane.
		*/
		bool operator != (const Plane &other);

		//! Check if this plane is not equal to another plane.
		/*!
			\param[in]	other	Another plane.
		*/
		bool operator != (const Plane &other) const;

	protected:
		//! Store the origin.
		Eigen::Vector3d m_origin;

		//! Store the normal.
		Eigen::Vector3d m_normal;

		//! Store the distance of plane origin from origin.
		/*!
			/brief Check the definition of d in the general equation.
		*/
		double m_distance;

		//! Store validity.
		bool m_is_valid;

	private:
		//! Determine the origin of the plane by giving the general equation. 
		Eigen::Vector3d findPointLocatedOnPlane();
	};	
}

#endif  // SYNSLICERENGINE_OBJECT_PLANE_H_