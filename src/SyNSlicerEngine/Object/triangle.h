#ifndef SYNSLICERENGINE_OBJECT_TRIANGLE_H_
#define SYNSLICERENGINE_OBJECT_TRIANGLE_H_

#include <Eigen/Core>

#include "Object/plane.h"

namespace SO = SyNSlicerEngine::Object;

//!  The largest namespace.
namespace SyNSlicerEngine 
{
	namespace Object
	{
		//!  This class defines a line or a ray.
		/*!
		Line is defined by \n
		(1) source point, target point \n
		(2) source point, unit pointing direction, length of the line \n
		Ray is defined by \n
		(1) source point, unit pointing direction \n
		*/
		class Triangle
		{
		public:
			Triangle();
			Triangle(const Triangle &other);
			Triangle(Eigen::Vector3d v0 = Eigen::Vector3d(0, 0, 0),
				Eigen::Vector3d v1 = Eigen::Vector3d(1, 0, 0),
				Eigen::Vector3d v2 = Eigen::Vector3d(0, 1, 0));
			Triangle(const CgalMesh_EPICK::Face_index &f, const CgalMesh_EPICK &mesh);
			~Triangle();

			double getArea();
			Eigen::Vector3d getNormal();
			double getOverhangingAngle(const SO::Plane &plane);
			bool isOneOfTheVerticesOnPlane(const SO::Plane &plane, double epsilon = 1e-6) const;

			Triangle &operator=(const Triangle &other);

			Eigen::Vector3d m_v0;
			Eigen::Vector3d m_v1;
			Eigen::Vector3d m_v2;
		};
	}
}


#endif  // SYNSLICERENGINE_OBJECT_TRIANGLE_H_