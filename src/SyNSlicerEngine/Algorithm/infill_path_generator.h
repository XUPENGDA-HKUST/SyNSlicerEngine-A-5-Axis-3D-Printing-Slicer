#ifndef SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_

#include <vector>

#include <CGAL_CORE_CLASS>

#include "spdlog/spdlog.h"

#include "Object/polygon_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to slice a 3D model with already calculated planes.
	/*!

	*/
	class InfillPathGenerator
	{
	public:
		//! Default constructore is not allowed.
		InfillPathGenerator() = delete;

		//! Constructor.
		/*!
			\param contours[in]			Contours to be filled in.
			\param cutting_planes[in]	Planes used to generate infills.
			\param side_step[in]		Distance between two consecutive paths.
			\param infill_type[in]		Infill pattern.
		*/
		InfillPathGenerator(
			const SO::PolygonCollection &contours, 
			const std::vector<SO::Plane> &cutting_planes,
			double side_step,
			int infill_type);

		//! Destructor.
		~InfillPathGenerator();

		//! Call to generate infill path.
		virtual void generateInfillPath();

		//! Call to get output.
		/*!
			\param[out] output	Output.
		*/
		virtual void getOutput(SO::PolygonCollection &output);

	protected:
		//! Generate grid infill path.
		virtual void generateGridInfillPath();

		//! Generate zigzag infill path.
		virtual void generateZigZagInfillPath();

		//! Generate contour parallel path.
		virtual void generateContourParallelInfillPath();

		//! Contours to be filled in.
		const SO::PolygonCollection m_contours;

		//! Planes used to generate infills.
		const std::vector<SO::Plane> m_cutting_planes;

		//! Distance between two consecutive paths.
		double m_side_step;

		//! Infill pattern.
		int m_infill_type;

		//! Store validity of input.
		bool m_is_inputs_valid;

		//! Store clockwise contours.
		SO::PolygonCollection clockwise_contours;

		//! Store counter-clockwise contours.
		SO::PolygonCollection counter_clockwise_contours;

		//! Store the output.
		SO::PolygonCollection m_output;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_