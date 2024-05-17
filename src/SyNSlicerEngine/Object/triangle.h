#ifndef SYNSLICERENGINE_OBJECT_TRIANGLE_H_
#define SYNSLICERENGINE_OBJECT_TRIANGLE_H_

#include <Eigen/Core>

#include "Object/plane.h"

namespace SO = SyNSlicerEngine::Object;

//!  The backend namespace.
namespace SyNSlicerEngine 
{
	namespace Object
	{
		//!  This class defines a line or a ray.
		/*!
			Line is defined by 
			\n (1) source point, target point 
			\n (2) source point, unit pointing direction, length of the line 
			\n Ray is defined by 
			\n (1) source point, unit pointing direction \n
		*/
		class Triangle
		{
		public:
			//! Default constructor.
			Triangle();

			//! Copy constructor.
			Triangle(const Triangle &other);

			//! Constructor
			/*!
				\param[in]	v0	Vertex 0.
				\param[in]	v1	Vertex 1.
				\param[in]	v2	Vertex 2.
			*/
			Triangle(
				Eigen::Vector3d v0 = Eigen::Vector3d(0, 0, 0),
				Eigen::Vector3d v1 = Eigen::Vector3d(1, 0, 0),
				Eigen::Vector3d v2 = Eigen::Vector3d(0, 1, 0));

			//! Constructor
			/*!
				\param[in]	f		Face id in a cgal mesh.
				\param[in]	mesh	The cgal mesh.
			*/
			Triangle(const CgalMesh_EPICK::Face_index &f, const CgalMesh_EPICK &mesh);

			//! Destructor.
			~Triangle();

			//! Get v0.
			/*!
				\return	\b Eigen::Vector3d Vertex 0.
			*/
			Eigen::Vector3d &v0();
			const Eigen::Vector3d &v0() const;

			//! Get v1.
			/*!
				\return	\b Eigen::Vector3d Vertex 1.
			*/
			Eigen::Vector3d &v1();
			const Eigen::Vector3d &v1() const;

			//! Get v2.
			/*!
				\return	\b Eigen::Vector3d Vertex 2.
			*/
			Eigen::Vector3d &v2();
			const Eigen::Vector3d &v2() const;

			//! Get area of the triangle.
			/*!
				\return	\b double Area.
			*/
			double getArea();

			//! Get noraml of the triangle.
			/*!
				\return	\b Eigen::Vector3d Noraml.
			*/
			Eigen::Vector3d getNormal();

			//! Get overhanging angle.
			/*!
				\param[in]	plane	Plane.
				\return	\b double Overhanging angle in radian.
			*/
			double getOverhangingAngle(const SO::Plane &plane);

			//! Check if one of the vertices on plane.
			/*!
				\param[in]	plane	Plane.
				\param[in]	epsilon	Epsilon in comparison.
				\return	\b True if yes. \n \b False if not.
			*/
			bool isOneOfTheVerticesOnPlane(const SO::Plane &plane, double epsilon = 1e-6) const;

			// Copy assignment operator.
			Triangle &operator=(const Triangle &other);

		protected:
			//! Store the vertices.
			Eigen::Vector3d m_vertices[3];
		};
	}
}

#endif  // SYNSLICERENGINE_OBJECT_TRIANGLE_H_