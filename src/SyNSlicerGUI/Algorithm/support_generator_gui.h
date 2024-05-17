#ifndef SYNSLICERENGINE_GUI_SUPPORTGENERATOR_H_
#define SYNSLICERENGINE_GUI_SUPPORTGENERATOR_H_

#include "SyNSlicerEngine/Algorithm/support_generator.h"
#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI
{
	//! This class is used to generate support structure.
	class SupportGeneratorGUI : public SA::SupportGenerator
	{
	public:
		//! Default constructor not allowed.
		SupportGeneratorGUI() = delete;

		SupportGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions,
			vtkRenderer *p_renderer);

		SupportGeneratorGUI(SO::PartitionCollection<CgalMesh_EPICK> &input_paritions,
			SyNSlicerGUI::ObjectDrawer *p_drawer);

		//! Destructor.
		~SupportGeneratorGUI();

	protected:
		bool m_should_drawer_delete_in_destructer;

		SyNSlicerGUI::ObjectDrawer *m_drawer;
	};
}

#endif  // SYNSLICERENGINE_GUI_SUPPORTGENERATOR_H_