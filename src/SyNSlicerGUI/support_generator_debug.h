#ifndef SYNSLICERENGINE_GUI_SUPPORTGENERATORDEBUG_H_
#define SYNSLICERENGINE_GUI_SUPPORTGENERATORDEBUG_H_

#include "Algorithm/support_generator.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerEngine::GUI {

	//! This class is used to generate support structure.
	class SupportGeneratorDebug : public SA::SupportGenerator
	{
	public:
		//! Default constructor not allowed.
		SupportGeneratorDebug() = delete;

		//! Constructor.
		/*!
			\param input_paritions	All the partitions.
		*/
		SupportGeneratorDebug(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions, vtkRenderer *p_renderer);

		//! Destructor.
		~SupportGeneratorDebug();

		//! Call to generate support structure.
		virtual void generateSupportStructure() override;

	protected:

		virtual void generateSupportStructureForSupportStructure(SO::PolygonCollection &contours,
			const SO::Plane &plane, std::vector<SO::Polyhedron<CgalMesh_EPICK>> &support_structures) override;

		void addSupportStructureofSupportStructureToParition(SO::Partition<CgalMesh_EPICK> &parition, const SO::Polyhedron<CgalMesh_EPICK> &support_structure) override;

		void addContoursToPrintingLayer(SO::PolygonCollection &contours, SO::PrintingLayer &printing_layer) override;

		ObjectDrawer m_drawer;

		int m_debug = 0;
	};
}

#endif  // SYNSLICERENGINE_GUI_SUPPORTGENERATORDEBUG_H_