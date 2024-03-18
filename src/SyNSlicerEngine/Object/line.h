#ifndef SYNSLICERENGINE_OBJECT_LINE_H_
#define SYNSLICERENGINE_OBJECT_LINE_H_

#include <Eigen/Core>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a line or a ray.
	/*!
	Line is defined by \n
	(1) source point, target point \n
	(2) source point, unit pointing direction, length of the line \n
	Ray is defined by \n
	(1) source point, unit pointing direction \n
	*/
	class Line
	{
	public:
		Line();
		Line(const Line &other);
		Line(const Eigen::Vector3d &source, const Eigen::Vector3d &target);
		Line(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length);
		Line(CgalMesh_EPICK::Halfedge_index he, CgalMesh_EPICK &mesh);
		Line(CgalMesh_EPECK::Halfedge_index he, CgalMesh_EPECK &mesh);
		~Line();

		void printInfo();

		void setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &target);
		void setLine(const Eigen::Vector3d &source, const Eigen::Vector3d &direction, double length);
		const Eigen::Vector3d &getSource() const;
		const Eigen::Vector3d &getTarget() const;
		const Eigen::Vector3d &getDirection() const;
		const double getLength() const;
		
		//!  Swap source and target, revert line diretion.
		void reverseDirection();

		Eigen::Vector3d findInterpolationOnLine(double middle, double start = 0.0, double end = 1.0);
		std::vector<Eigen::Vector3d> getRefinedLine(double gap);

		double getDistanceOfPoint(const Eigen::Vector3d &point) const;

		Line &operator=(const Line &other);

	protected:
		Eigen::Vector3d m_source;
		Eigen::Vector3d m_target;
		Eigen::Vector3d m_direction;
		double m_length;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_LINE_H_