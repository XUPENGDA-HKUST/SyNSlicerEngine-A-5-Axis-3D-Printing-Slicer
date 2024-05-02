#ifndef SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_
#define SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_

#include <CGAL_CORE_CLASS>
#include <Eigen/Core>

#include "Object/line.h"
#include "Object/plane.h"
#include "Object/polygon_collection.h"
#include "Object/printing_path_collection.h"
#include "Object/toolpath_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object {

	//!  This class is used to store the toolpaths associated with the printing layers.
	/*!
		<b> Definition of the contour/contours: </b> 
		\n The intersection of a triangle mesh 3D model and a plane. 
		\n The intersection should be a polygon.
	*/
	class PrintingLayer
	{
	public:
		//! Default constructor.
		PrintingLayer();

		//! Copy constructor.
		PrintingLayer(const PrintingLayer &other);

		//! Construct a printing layer has no contours inside.
		/*!
			\param	slicing_plane		Current slicing plane.
			\param	prev_slicing_plane	Previous slicing plane.
		*/
		PrintingLayer(const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane);

		//! Construct a printing layer with contours inside.
		/*!
			\param	contours			Contours.
			\param	slicing_plane		Current slicing plane, this plane should be same with the plane of the contours.
			\param	prev_slicing_plane	Previous slicing plane.
		*/
		PrintingLayer(const SO::PolygonCollection &contours, const SO::Plane &slicing_plane, const SO::Plane &prev_slicing_plane);

		//! Destructor.
		~PrintingLayer();

		// Get number of contours.
		/*!
			\return	\b int Number of contours.
		*/
		int getNumberOfContours();

		//! Get the n-th contour in contours.
		/*!
			\param	index	n.
			\return	\b SO::Polygon The n-th contour.
		*/
		SO::Polygon &getContour(int index);

		//! Get all the contours.
		/*!
			\return	\b SO::PolygonCollection The n-th contour.
		*/
		SO::PolygonCollection &getContours();

		//! Replace the old support structure contours with the new one.
		/*!
			\param	input_support_structure_contours	New support structure contours.
		*/
		void setSupportStructureContours(const SO::PolygonCollection &input_support_structure_contours);

		//! Add a support structure contours.
		/*!
			\param	input_support_structure_contours	Support structure contours to be added.
		*/
		void addSupportStructureContours(SO::PolygonCollection &input_support_structure_contours);

		//! Get support structure contours.
		/*!
			\return	\b SO::PolygonCollection The support structure contours.
		*/
		SO::PolygonCollection &getSupportStructureContours();

		//! Get the origin.
		/*!
			\brief	This origin is defined as the center of infill structure.
			\return	\b Eigen::Vector3d The origin of the infill structure.
		*/
		Eigen::Vector3d &getOrigin();

		//! Get direction 1.
		/*!
			\brief	Direction 1 and 2 are used to align the infill structure across difference printing layers.
			\return	\b SO::Line Direction 1.
		*/
		SO::Line &getDirection1();

		//! Get direction 2.
		/*!
			\brief	Direction 1 and 2 are used to align the infill structure across difference printing layers.
			\return	\b SO::Line Direction 2.
		*/
		SO::Line &getDirection2();

		//! Get the printing paths for the model.
		/*!
			\return	\b SO::PrintingPathCollection Printing path for the model.
		*/
		SO::PrintingPathCollection &getPrintingPaths();

		//! Get the printing paths for the support structure.
		/*!
			\return	\b SO::PrintingPathCollection Printing paths for the support structure.
		*/
		SO::PrintingPathCollection &getPrintingPathsForSupport();

		//! Replace the old toolpaths with a new toolpaths.
		/*!
			\param	toolpaths	New toolpaths.
		*/
		void setToolpaths(const ToolpathCollection &toolpaths);

		//! Get the toolpaths.
		/*!
			\return	\b SO::ToolpathCollection The toolpaths.
		*/
		const ToolpathCollection &getToolpaths() const;

		//! Clear the toolpaths.
		void clearToolpath();

		//! Get the slicing plane.
		/*!
			\return	\b SO::Plane The slicing plane.
		*/
		SO::Plane getSlicingPlane();

		//! Get the previous slicing plane.
		/*!
			\return	\b SO::Plane The previous slicing plane.
		*/
		SO::Plane getPrevSlicingPlane();

		//! Copy assignment operator.
		PrintingLayer &operator=(const PrintingLayer &other);

	protected:
		//! Store the outer contours
		SO::PolygonCollection m_contours;

		//! Store the outer contours of the support structure.	
		SO::PolygonCollection m_support_structure_contours;

		//! The slicing plane of this printing layer.
		Plane m_slicing_plane;

		//! The slicing plane of the previous printing layer.
		Plane m_prev_slicing_plane;

		//! Store the origin of the infill printing paths.
		Eigen::Vector3d m_origin_for_infill_alignment;

		//! Store direction 1 of the infill printing paths.
		SO::Line m_direction_1_for_infill_alignment;

		//! Store direction 2 of the infill printing paths.
		SO::Line m_direction_2_for_infill_alignment;

		//! Store the printing paths.
		PrintingPathCollection m_printing_paths;

		//! Store the printing paths for support structure.
		PrintingPathCollection m_printing_paths_support;

		//! Store the toolpath to printing this layer
		ToolpathCollection m_toolpaths;
	};
}

#endif  // SYNSLICERENGINE_OBJECT_PRINTINGLAYER_H_