#ifndef SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_
#define SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_

#include <vector>

#include <clipper2/clipper.h>

#include <CGAL_CORE_CLASS>

#include "Object/polygon.h"
#include <Object/partition.h>
#include <Object/printing_layer_collection.h>
#include <Object/plane.h>

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to slice a 3D model automatically with non-parallel planes.
	/*!
	
	*/
	class AutoSlicer
	{
	public:
		//! Default constructor not allowed.
		AutoSlicer() = delete;

		//! Constructor.
		/*!
			\param p_partition				The partition to be sliced.
			\param target_layer_thickness	The average layer thickness want to achieve.
			\param side_step				The distance between consecutive paths.
			\param min_layer_thickness		The minimum layer thickness that the printer can print.
			\param max_layer_thickness		The maximum layer thickness that the printer can print.
		*/
		explicit AutoSlicer(SO::Partition<CgalMesh_EPICK> &p_partition,
			double target_layer_thickness = 0.3, double side_step = 0.4, double min_layer_thickness = 0.25,
			double max_layer_thickness = 0.35);

		//! Destructor.
		~AutoSlicer();

		//! Call to perform slicing.
		virtual void slice();

	protected:
		//! Offset slicing plane and search for .
		/*!
			Part 1: Offset the current slicing plane by target_layer_thickness 
				continuously until support structure is needed. \n
			Part 2: Base on the starting slicing plane and the last slicing plane, 
				determine all the intermediate planes that can minimize support structure. \n
			\param current_starting_contours	The contours used to start offsetting this time.
			\param next_starting_contours		The contours used to start offsetting next time.
		*/
		virtual bool determineNextSlicingPlane(SO::PolygonCollection &current_starting_contours, SO::PolygonCollection &next_starting_contours);

		//! Check if support structure is needed during printing contours_up.
		/*!
			\param contours_below	The printing layer below.
			\param contours_up		The printing layer above.
			\param support_contours The support structure needs to hold contour up.
			\param coefficient		The coefficient related to overhanging angle.
		*/
		virtual bool checkSupportNeeded(SO::PolygonCollection &contours_below, SO::PolygonCollection &contours_up, SO::PolygonCollection &support_contours, double coefficient = 0.5);

		//! Determine all intermediate planes between plane_up and plane_below.
		/*!
			Make sure all the planes generate a printing layer that the layer thickness is between
			m_min_layer_thickness and m_max_layer_thickness. \n
			\param plane_up			The plane above.
			\param plane_below		The plane below.
		*/
		virtual bool calculateIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below);

		//! Determine all default intermediate planes between plane_up and plane_below.
		/*!
			Default intermediate planes may not meet the requirement of layer thickness in between 
			m_min_layer_thickness and m_max_layer_thickness. \n
			All default intermediate planes will adjust in calculateIntermediatePlanes() so that all of them 
			can meet the requirement.
			\param plane_up			The plane above.
			\param plane_below		The plane below.
			\param slicing_planes	The default intermediate planes.
		*/
		virtual bool computeDefaultIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below, std::vector<SO::Plane> &slicing_planes);
		
		//! Determine all default intermediate planes between plane_up and plane_below.
		/*!
			Default intermediate planes may not meet the requirement of layer thickness in between
			m_min_layer_thickness and m_max_layer_thickness. \n
			Tune them one by one until they all meet the requirement of layer thickness is between
			m_min_layer_thickness and m_max_layer_thickness. \n
			\param plane_up		The plane to tune.
			\param plane_below	The plane below.
		*/
		virtual bool tuneConsecutivePlanesValid(SO::Plane &plane_up, SO::Plane plane_below);

		//! Check if plane_up meet the requirement of layer thickness is in between m_min_layer_thickness and m_max_layer_thickness.
		/*!
			\param plane_up		The plane to check.
			\param plane_below	The plane below.
		*/
		virtual bool isSlicingPlaneValid(SO::Plane plane_up, SO::Plane plane_below);

		//! Slice the 3D model with a plane, obtain the contours.
		/*!
			\param plane	The plane used to slice the 3D model.
		*/
		SO::PolygonCollection slice(SO::Plane plane);

		//! 3D model to be sliced.
		SO::Partition<CgalMesh_EPICK> *mp_operating_partition;

		//! 3D model to be sliced.
		CgalMesh_EPICK m_mesh;

		//! Algorithm used to slice the 3D model.
		CGAL::Polygon_mesh_slicer<CgalMesh_EPICK, EPICK> m_slicer;

		//! Maximum layer thickness that the printer can print.
		double m_max_layer_thickness;

		//! Minimum layer thickness that the printer can print.
		double m_min_layer_thickness;

		//! Average layer thickness want to achieve.
		double m_target_layer_thickness;

		//! Distance between consecutive paths.
		double m_side_step;

		//! Member variable for transimitting variable between member function
		std::vector<SO::PolygonCollection> m_temp_slicing_result;

		//! Member variable for transimitting variable between member function
		std::vector<SO::PolygonCollection> m_slicing_result;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_AUTOSLICER_H_