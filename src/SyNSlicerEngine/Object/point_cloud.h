#ifndef SYNSLICERENGINE_OBJECT_POINTCLOUD_H_
#define SYNSLICERENGINE_OBJECT_POINTCLOUD_H_

#include <vector>

#include <Eigen/Dense>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a point cloud.
	class PointCloud
	{
	public:
		//! Default constructor.
		PointCloud();

		//! Copy constructor.
		PointCloud(const PointCloud &other);

		//! Constructor
		/*!
			\brief Construct PointCloud by giving multiple points.
			\param[in] points Points.
		*/
		PointCloud(const std::vector<Eigen::Vector3d> &points);

		//! Destructor.
		~PointCloud();
		
		//! Restore to default value.
		void reset();

		//! Check if two point cloud has common point.
		/*!
			\param[in] other Another point cloud.
			\return \b True if has common point. \n \b False if has no common point.
		*/
		bool hasCommonPoints(const PointCloud &other);

		//! Add point.
		/*!
			\param[in] point Point.
		*/
		void addPoint(Eigen::Vector3d point);

		//! Get bounding box of this point cloud.
		/*!
			\param[out] bound Bounding box.
		*/
		void getBound(double bound[6]);

		//! Get all points in the point cloud.
		/*!
			\return std::vector<Eigen::Vector3d> Points.
		*/
		std::vector<Eigen::Vector3d> points();

		//! Get number of points in the point cloud.
		/*!
			\return int Number of points.
		*/
		int size() const;

		//! Get the index-th point in the point cloud.
		/*!
			\param[in] index Index. 
			\return Eigen::Vector3d Point.
		*/
		Eigen::Vector3d operator[](int index);

		//! Get the index-th point in the point cloud.
		/*!
			\param[in] index Index.
			\return const Eigen::Vector3d & Point.
		*/
		const Eigen::Vector3d &operator[](int index) const;

		//! Copy assignment operators
		PointCloud &operator = (const PointCloud &other);

	protected:
		//! Update bound after adding new point.
		/*!
			\param[in] point Point.
		*/
		void updateBound(Eigen::Vector3d point);

		//! Store all the points.
		std::vector<Eigen::Vector3d> m_points;

		//! Store the bounding box.
		double m_bound[6];

		//! Store the status.
		bool m_is_empty;
	};
}


#endif //SYNSLICERENGINE_OBJECT_POINTCLOUD_H_