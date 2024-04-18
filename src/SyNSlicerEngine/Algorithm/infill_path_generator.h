#ifndef SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_

#include <vector>

#include <CGAL_CORE_CLASS>

#include "spdlog/spdlog.h"

#include "Object/polygon_collection.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	//! This class is used to generate infill path.
	class InfillPathGenerator
	{
	public:
		InfillPathGenerator(
			const SO::PolygonCollection &contours, 
			const std::vector<SO::Plane> &cutting_planes,
			double side_step,
			int infill_type);
		~InfillPathGenerator();

		virtual void generateInfillPath();
		virtual void getOutput(SO::PolygonCollection &ouput);

	protected:
		virtual void generateGridInfillPath();
		virtual void generateZigZagInfillPath();
		virtual void generateContourParallelInfillPath();

		const SO::PolygonCollection &m_contours;
		const std::vector<SO::Plane> &m_cutting_planes;
		double m_side_step;
		int m_infill_type;
		bool m_is_inputs_valid;

		SO::PolygonCollection clockwise_contours;
		SO::PolygonCollection counter_clockwise_contours;

		CgalPolygon2D_EPICK m_cgal_outer_contour;
		std::vector<CgalPolygon2D_EPICK> m_cgal_inner_contours;

		SO::PolygonCollection m_output;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_