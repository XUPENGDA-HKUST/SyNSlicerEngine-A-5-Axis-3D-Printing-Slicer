#ifndef SYNSLICERENGINE_OBJECT_TOOLPOINT_H_
#define SYNSLICERENGINE_OBJECT_TOOLPOINT_H_

#include <limits>

#include <Eigen/Core>

namespace SyNSlicerEngine::Object {

	//! This class is used to describe a tool point.
	/*!
		A tool point has 4 members. \n
		(1) Tool position \n
		(2) Tool orientation \n
		(3) Amount of material extruded from the previous tool point to this tool point \n
		(4) Local layer thickness used to calculate member 3.
	*/
	class ToolPoint
	{
	public:

		ToolPoint();
		ToolPoint(const ToolPoint &other);
		ToolPoint(Eigen::Vector3d position, Eigen::Vector3d nomral, double extrusion = 0.0, double layer_thickness = 0.0);
		~ToolPoint();

		void setPosition(const Eigen::Vector3d &position);
		const Eigen::Vector3d &getPosition() const; //!< Read Only
		void setNormal(const Eigen::Vector3d &normal);
		const Eigen::Vector3d &getNormal() const; //!< Read Only
		void setExtrusion(double extrusion);
		const double &getExtrusion() const; //!< Read Only
		void setLayerThickness(double local_layer_thickness);
		const double &getLayerThickness() const; //!< Read Only

		const double operator[](int index) const; //!< Read Only
		ToolPoint &operator = (const ToolPoint &other);

	protected:
		Eigen::Vector3d m_position;
		Eigen::Vector3d m_normal;
		double m_extrusion;
		double m_local_layer_thickness;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPOINT_H_