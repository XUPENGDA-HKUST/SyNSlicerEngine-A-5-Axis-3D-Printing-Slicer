#ifndef SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_
#define SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_

#include <vector>

#include <CGAL_CORE_CLASS>

#include "spdlog/spdlog.h"
#include "vtkRenderer.h"

#include "Object/polygon_collection.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Algorithm
{
	class InfillPathGenerator
	{
	public:
		InfillPathGenerator(
			const SO::PolygonCollection &contours, 
			const std::vector<SO::Plane> &cutting_planes,
			double side_step,
			int infill_type,
			vtkRenderer *renderer = nullptr);
		~InfillPathGenerator();

		void generateInfillPath();
		void getOutput(SO::PolygonCollection &ouput);

	protected:
		void generateGridInfillPath();
		void generateZigZagInfillPath();
		void generateContourParallelInfillPath();

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

	private:
		GUI::ObjectDrawer m_drawer;
	};
}

#endif  // SYNSLICERENGINE_ALGORITHM_INFILLPATHGENERATOR_H_