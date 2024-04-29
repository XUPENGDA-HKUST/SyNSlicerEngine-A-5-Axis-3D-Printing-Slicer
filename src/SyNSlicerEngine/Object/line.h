#ifndef SYNSLICERENGINE_OBJECT_LINE_H_
#define SYNSLICERENGINE_OBJECT_LINE_H_

#include <Eigen/Core>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a line or a ray.
	/*!
	Line is defined by \n
	(1) source point, target point \n\n
	Ray is defined by \n
	(1) source point, unit pointing direction \n
	*/
	class Line
	{
	public:
		//! Default constructor.
		Line();

		//! Copy constructor.
		Line(const Line &other);

		//! Constructor.
		/*!
			\param[in] source The source of the line.
			\param[in] target The target of the line.
		*/
		Line(const Eigen::Vector3d &source, const Eigen::Vector3d &target);

		//! Constructor.
		/*!
			\param[in] source		The source of the ray.
			\param[in] direction	The shooting direction of the ray.
			\param[in] length		The length of the ray.
		*/
		Line(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length);

		//! Constructor.
		/*!
			Convert a half-edge in CgalMesh_EPICK to Line.
			\param[in] he		The half-edge index.
			\param[in] mesh		The mesh.
		*/
		Line(CgalMesh_EPICK::Halfedge_index he, CgalMesh_EPICK &mesh);

		//! Constructor.
		/*!
			Convert a half-edge in CgalMesh_EPECK to Line.
			\param[in] he		The half-edge index.
			\param[in] mesh		The mesh.
		*/
		Line(CgalMesh_EPECK::Halfedge_index he, CgalMesh_EPECK &mesh);

		//! Destructor.
		~Line();

		//! Store validity.
		bool isValid() const;

		//! Print information of this line in console.
		void printInfo();

		//! Construct line with location of source and target.
		/*!
			\param[in] source The source of the line.
			\param[in] target The target of the line.
		*/
		void setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &target);

		//! Construct line with location of source, direction of the target w.r.t. the source and length.
		/*!
			\param[in] source		The source of the ray.
			\param[in] direction	The shooting direction of the ray.
			\param[in] length		The length of the ray.
		*/
		void setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length);

		//! Call to get the source.
		const Eigen::Vector3d &getSource() const;

		//! Call to get the target.
		const Eigen::Vector3d &getTarget() const;

		//! Call to get the direction.
		const Eigen::Vector3d &getDirection() const;

		//! Call to get the middle of the line.
		Eigen::Vector3d getMiddle() const;

		//! Call to get the length of the line.
		const double getLength() const;
		
		//! Swap source and target, revert line diretion.
		void reverseDirection();

		//! Get distance from point to ray.
		/*!
			\param[in]	point	The point.
			\return Distance.
		*/
		double getDistanceOfPoint(const Eigen::Vector3d &point) const;

		//! Get distance from point to line segment.
		/*!
			\param[in]	point	The point.
			\return Distance.
		*/
		double getDistanceFromPointToLineSegment(const Eigen::Vector3d &point) const;

		//! Get projection of point on ray.
		/*!
			\param[in]	point	The point.
			\return The projection.
		*/
		Eigen::Vector3d getProjectionOfPointOntoRay(const Eigen::Vector3d &point);

		//! Copy operation.
		Line &operator=(const Line &other);

	protected:
		//! Store the source.
		Eigen::Vector3d m_source;

		//! Store the target.
		Eigen::Vector3d m_target;

		//! Store the direction.
		Eigen::Vector3d m_direction;

		//! store the length.
		double m_length;

		//! store validity.
		bool m_is_valid;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_LINE_H_