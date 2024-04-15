#ifndef SYNSLICERENGINE_OBJECT_POINTCLOUD_H_
#define SYNSLICERENGINE_OBJECT_POINTCLOUD_H_

#include <vector>

#include <Eigen/Dense>

namespace SyNSlicerEngine::Object
{
	//!  This class defines a point cloud.
	class PointCloud
	{
		enum Status {
			Empty,
			FirstPointAdded
		};

	public:
		PointCloud();
		PointCloud(const PointCloud &other);
		~PointCloud();
		
		void reset();

		void addPoint(Eigen::Vector3d point);
		void getBound(double bound[6]);

		std::vector<Eigen::Vector3d> points();

		int size() const;

		Eigen::Vector3d operator[](int index);
		const Eigen::Vector3d &operator[](int index) const;

		PointCloud &operator = (const PointCloud &other);

	protected:
		void updateBound(Eigen::Vector3d point);

		std::vector<Eigen::Vector3d> m_points;
		double m_bound[6];
		Status m_status;
	};
}


#endif //SYNSLICERENGINE_OBJECT_POINTCLOUD_H_