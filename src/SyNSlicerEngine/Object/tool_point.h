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
		//! Default constructor.
		ToolPoint();

		//! Copy constructor.
		ToolPoint(const ToolPoint &other);

		//! Constructor.
		ToolPoint(Eigen::Vector3d position, Eigen::Vector3d nomral, double extrusion = 0.0, double layer_thickness = 0.0);

		//! Destructor.
		~ToolPoint();

		//! Set position.
		/*!
			\param	position	Position.
		*/
		void setPosition(const Eigen::Vector3d &position);

		//! Get position.
		/*!
			\return <b> const Eigen::Vector3d &</b> Position.
		*/
		const Eigen::Vector3d &getPosition() const;

		//! Set normal.
		/*!
			\param	normal	Normal.
		*/
		void setNormal(const Eigen::Vector3d &normal);

		//! Get normal.
		/*!
			\return <b> const Eigen::Vector3d & </b> Normal.
		*/
		const Eigen::Vector3d &getNormal() const;

		//! Set extrusion.
		/*!
			\param	extrusion	Extrusion.
		*/
		void setExtrusion(double extrusion);

		//! Get extrusion.
		/*!
			\return <b> const double & </b> Extrusion.
		*/
		const double &getExtrusion() const;

		//! Set local layer thickness.
		/*!
			\param	local_layer_thickness	Local layer thickness.
		*/
		void setLayerThickness(double local_layer_thickness);

		//! Get local layer thickness.
		/*!
			\return <b> const double & </b> Local layer thickness.
		*/
		const double &getLayerThickness() const;

		//! Get value depends on the given index.
		/*!
			\param	index	0 to 7.
		*/
		const double operator[](int index) const;

		//! Copy assignment operator.
		ToolPoint &operator = (const ToolPoint &other);

	protected:
		//! Store the position.
		Eigen::Vector3d m_position;

		//! Store the normal.
		Eigen::Vector3d m_normal;

		//! Store the extrusion.
		double m_extrusion;

		//! Store the layer thickness.
		double m_local_layer_thickness;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_TOOLPOINT_H_