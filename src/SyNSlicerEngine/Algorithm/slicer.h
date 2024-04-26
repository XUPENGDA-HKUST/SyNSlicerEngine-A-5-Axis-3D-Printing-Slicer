#ifndef SYNSLICERENGINE_ALGORITHM_SLICER_H_
#define SYNSLICERENGINE_ALGORITHM_SLICER_H_

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
	class Slicer
	{
	public:
		//! Default constructor not allowed.
		Slicer() = delete;

		//! Constructor.
		/*!
			\param partition				The partition to be sliced.
			\param target_layer_thickness	The average layer thickness want to achieve.
			\param side_step				The distance between consecutive paths.
			\param min_layer_thickness		The minimum layer thickness that the printer can print.
			\param m_max_layer_thickness	The maximum layer thickness that the printer can print.
		*/
		explicit Slicer(SO::Partition<CgalMesh_EPICK> &p_partition);

		//! Destructor.
		~Slicer();

		//! Call to perform slicing.
		virtual void slice();

	protected:
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
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_SLICER_H_